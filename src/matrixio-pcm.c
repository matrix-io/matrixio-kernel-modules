/*
 * matrix-codec.c -- MATRIX microphone array audio driver
 *
 * Copyright 2018 MATRIX Labs
 *
 * Author: Andres Calderon <andres.calderon@admobilize.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include "matrixio-pcm.h"
#include "matrixio-core.h"

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define MATRIXIO_CHANNELS_MAX 8
#define MATRIXIO_RATES SNDRV_PCM_RATE_8000_96000
#define MATRIXIO_FORMATS SNDRV_PCM_FMTBIT_S16_LE
#define MATRIXIO_MICARRAY_BUFFER_SIZE (512 * 2)

static struct matrixio_substream *ms;

static uint16_t matrixio_buf[MATRIXIO_CHANNELS_MAX][8192];

static const uint32_t matrixio_params[][3] = {
    {8000, 374, 32}, {12000, 249, 2}, {16000, 186, 3},
    {22050, 135, 5}, {24000, 124, 5}, {32000, 92, 6},
    {44100, 67, 7},  {48000, 61, 7},  {96000, 30, 10}};

static struct snd_pcm_hardware matrixio_pcm_capture_hw = {
    .info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_PAUSE,
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = SNDRV_PCM_RATE_8000_96000,
    .rate_min = 8000,
    .rate_max = 96000,
    .channels_min = 1,
    .channels_max = 8,
    .buffer_bytes_max = 32768,
    .period_bytes_min = 4096,
    .period_bytes_max = 32768,
    .periods_min = 4,
    .periods_max = 8,
};

static void matrixio_pcm_capture_work(struct work_struct *wk)
{
	int c;
	int ret;
	struct matrixio_substream *ms;

	ms = container_of(wk, struct matrixio_substream, work);

	mutex_lock(&ms->lock);

	for (c = 0; c < ms->channels; c++)
		ret = matrixio_read(
		    ms->mio, MATRIXIO_MICARRAY_BASE +
				 c * (MATRIXIO_MICARRAY_BUFFER_SIZE >> 1),
		    MATRIXIO_MICARRAY_BUFFER_SIZE,
		    &matrixio_buf[c][ms->position]);

	ms->position += MATRIXIO_MICARRAY_BUFFER_SIZE >> 1;
	mutex_unlock(&ms->lock);

	snd_pcm_period_elapsed(ms->capture_substream);
}

static irqreturn_t matrixio_pcm_interrupt(int irq, void *irq_data)
{
	struct matrixio_substream *ms = irq_data;

	if (ms->capture_substream == 0)
		return IRQ_NONE;

	queue_work(ms->wq, &ms->work);

	return IRQ_HANDLED;
}

static int matrixio_pcm_open(struct snd_pcm_substream *substream)
{
	int ret;

	char workqueue_name[12];

	snd_soc_set_runtime_hwparams(substream, &matrixio_pcm_capture_hw);

	snd_pcm_set_sync(substream);

	if (ms->capture_substream != NULL) {
		return -EBUSY;
	}

	ms->capture_substream = substream;

	ms->position = 0;

	sprintf(workqueue_name, "matrixio_pcm");

	ms->wq = create_singlethread_workqueue(workqueue_name);

	if (!ms->wq) {
		return -ENOMEM;
	}

	INIT_WORK(&ms->work, matrixio_pcm_capture_work);

	ret = request_irq(ms->irq, matrixio_pcm_interrupt, 0,
			  "matrixio-capture", ms);
	if (ret) {
		destroy_workqueue(ms->wq);
		return -EBUSY;
	}

	return 0;
}

static int matrixio_pcm_close(struct snd_pcm_substream *substream)
{
	free_irq(ms->irq, ms);

	flush_workqueue(ms->wq);

	destroy_workqueue(ms->wq);

	ms->capture_substream = 0;

	return 0;
}

static int matrixio_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	int i;
	int rate;

	ms->channels = params_channels(hw_params);

	if (snd_pcm_format_width(params_format(hw_params)) != 16)
		return -EINVAL;

	rate = params_rate(hw_params);

	for (i = 0; i < ARRAY_SIZE(matrixio_params); i++) {
		if (rate == matrixio_params[i][0]) {
			regmap_write(ms->mio->regmap, MATRIXIO_CONF_BASE + 0x06,
				     matrixio_params[i][1]);

			regmap_write(ms->mio->regmap, MATRIXIO_CONF_BASE + 0x07,
				     matrixio_params[i][2]);
			return 0;
		}
	}

	return -EINVAL;
}

static int matrixio_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int matrixio_pcm_prepare(struct snd_pcm_substream *substream)
{
	ms->position = 0;
	return 0;
}

static snd_pcm_uframes_t
matrixio_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	mutex_lock(&ms->lock);
	ms->position = ms->position < runtime->buffer_size ? ms->position : 0;
	mutex_unlock(&ms->lock);

	return ms->position;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 13, 0)
static int matrixio_pcm_copy(struct snd_pcm_substream *substream, int channel,
			     snd_pcm_uframes_t pos, void __user *buf,
			     snd_pcm_uframes_t count)
{
	int i, c;
	static int16_t buf_interleaved[MATRIXIO_CHANNELS_MAX * 8192];

	for (i = 0; i < count; i++)
		for (c = 0; c < ms->channels; c++)
			buf_interleaved[i * ms->channels + c] =
			    matrixio_buf[c][pos + i];
	return copy_to_user(buf, buf_interleaved, count * 2 * ms->channels);
}
#else
static int matrixio_pcm_copy(struct snd_pcm_substream *substream, int channel,
			     snd_pcm_uframes_t pos, void __user *buf,
			     snd_pcm_uframes_t bytes)
{
	int i, c;
	static int16_t buf_interleaved[MATRIXIO_CHANNELS_MAX * 8192];
	struct snd_pcm_runtime *runtime = substream->runtime;
	int frame_pos = bytes_to_frames(runtime, pos);
	int frame_count = bytes_to_frames(runtime, bytes);

	for (i = 0; i < frame_count; i++)
		for (c = 0; c < ms->channels; c++)
			buf_interleaved[i * ms->channels + c] =
			    matrixio_buf[c][frame_pos + i];
	return copy_to_user(buf, buf_interleaved, bytes);
}
#endif

static struct snd_pcm_ops matrixio_pcm_ops = {
    .open = matrixio_pcm_open,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = matrixio_pcm_hw_params,
    .hw_free = matrixio_pcm_hw_free,
    .prepare = matrixio_pcm_prepare,
    .pointer = matrixio_pcm_pointer,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 13, 0)
    .copy = matrixio_pcm_copy,
#else
    .copy_user = matrixio_pcm_copy,
#endif
    .close = matrixio_pcm_close,
};

static int matrixio_pcm_new(struct snd_soc_pcm_runtime *rtd) { return 0; }

static const struct snd_soc_platform_driver matrixio_soc_platform = {
    .ops = &matrixio_pcm_ops, .pcm_new = matrixio_pcm_new,
};

static int matrixio_pcm_platform_probe(struct platform_device *pdev)
{
	int ret;

	ms = devm_kzalloc(&pdev->dev, sizeof(struct matrixio_substream),
			  GFP_KERNEL);
	if (!ms) {
		dev_err(&pdev->dev, "data allocation");
		return -ENOMEM;
	}

	ms->mio = dev_get_drvdata(pdev->dev.parent);

	ms->capture_substream = 0;

	mutex_init(&ms->lock);

	ms->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);

	ret =
	    devm_snd_soc_register_platform(&pdev->dev, &matrixio_soc_platform);
	if (ret) {
		dev_err(&pdev->dev,
			"MATRIXIO sound SoC register platform error: %d", ret);
		return ret;
	}

	dev_set_drvdata(&pdev->dev, ms);

	dev_notice(&pdev->dev, "MATRIXIO audio drive loaded (IRQ=%d)", ms->irq);
	return 0;
}

static const struct of_device_id snd_matrixio_pcm_of_match[] = {
    {
	.compatible = "matrixio-pcm",
    },
    {},
};
MODULE_DEVICE_TABLE(of, snd_matrixio_pcm_of_match);

static struct platform_driver matrixio_codec_driver = {
    .driver = {.name = "matrixio-pcm",
	       .owner = THIS_MODULE,
	       .of_match_table = snd_matrixio_pcm_of_match},
    .probe = matrixio_pcm_platform_probe,
};

module_platform_driver(matrixio_codec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO PCM module");
