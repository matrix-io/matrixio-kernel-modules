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

#include "matrixio-core.h"
#include "matrixio-pcm.h"

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
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

const uint32_t kMaxWriteLength = 1024;

const uint32_t kFIFOSize = 4096;

static struct matrixio_substream *ms;

static uint16_t matrixio_buf[MATRIXIO_CHANNELS_MAX][8192];

static const struct playback_params pcm_sampling_frequencies[] = {
    {8000, 1000000 / 8000, 975},   {16000, 1000000 / 16000, 492},
    {32000, 1000000 / 32000, 245}, {44100, 1000000 / 44100, 177},
    {48000, 1000000 / 48000, 163}, {88200, 1000000 / 88200, 88},
    {96000, 1000000 / 96000, 81}};

static struct snd_pcm_hardware matrixio_playback_capture_hw = {
    .info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_PAUSE,
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = SNDRV_PCM_RATE_8000_48000,
    .rate_min = 8000,
    .rate_max = 48000,
    .channels_min = 2,
    .channels_max = 2,
    .buffer_bytes_max = 32768,
    .period_bytes_min = 4096,
    .period_bytes_max = 32768,
    .periods_min = 4,
    .periods_max = 8,
};

static void matrixio_pcm_playback_work(struct work_struct *wk)
{
	struct matrixio_substream *ms;
	uint16_t fifo_status;

	ms = container_of(wk, struct matrixio_substream, work);

	mutex_lock(&ms->lock);

	if (fifo_status > kFIFOSize * 3 / 4) {
		udelay(ms->playback_params->period * kMaxWriteLength);
	}

	ms->position += kMaxWriteLength;

	mutex_unlock(&ms->lock);

	snd_pcm_period_elapsed(ms->substream);
}

static int matrixio_playback_open(struct snd_pcm_substream *substream)
{
	int ret;

	char workqueue_name[32];

	printk(
	    " open open open -----------------------------------------------");

	snd_soc_set_runtime_hwparams(substream, &matrixio_playback_capture_hw);

	snd_pcm_set_sync(substream);

	if (ms->substream != NULL) {
		return -EBUSY;
	}

	ms->substream = substream;

	ms->position = 0;

	sprintf(workqueue_name, "matrixio_playback");

	ms->wq = create_singlethread_workqueue(workqueue_name);

	if (!ms->wq) {
		return -ENOMEM;
	}

	INIT_WORK(&ms->work, matrixio_pcm_playback_work);

	return 0;
}

static int matrixio_playback_close(struct snd_pcm_substream *substream)
{
	flush_workqueue(ms->wq);

	destroy_workqueue(ms->wq);

	ms->substream = 0;

	return 0;
}

static int matrixio_playback_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *hw_params)
{
	int i;
	int rate;

	printk(
	    "hw params *****************************************************");
	ms->channels = params_channels(hw_params);

	if (snd_pcm_format_width(params_format(hw_params)) != 16)
		return -EINVAL;

	rate = params_rate(hw_params);
	/*
		for (i = 0; i < ARRAY_SIZE(matrixio_params); i++) {
			if (rate == matrixio_params[i][0]) {
				regmap_write(ms->mio->regmap, MATRIXIO_CONF_BASE
	   + 0x06,
					     matrixio_params[i][1]);

				regmap_write(ms->mio->regmap, MATRIXIO_CONF_BASE
	   + 0x07,
					     matrixio_params[i][2]);
				break;
			}
		}
	*/
	return 0;
	return -EINVAL;
}

static int matrixio_playback_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int matrixio_playback_prepare(struct snd_pcm_substream *substream)
{
	ms->position = 0;
	return 0;
}

static snd_pcm_uframes_t
matrixio_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	mutex_lock(&ms->lock);
	ms->position = ms->position < runtime->buffer_size ? ms->position : 0;
	mutex_unlock(&ms->lock);

	return ms->position;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 13, 0)
static int matrixio_playback_copy(struct snd_pcm_substream *substream,
				  int channel, snd_pcm_uframes_t pos,
				  void __user *buf, snd_pcm_uframes_t count)
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
static int matrixio_playback_copy(struct snd_pcm_substream *substream,
				  int channel, snd_pcm_uframes_t pos,
				  void __user *buf, snd_pcm_uframes_t bytes)
{
	int i, c;

	struct snd_pcm_runtime *runtime = substream->runtime;

	int frame_pos = bytes_to_frames(runtime, pos);
	int frame_count = bytes_to_frames(runtime, bytes);
	printk("copy %d %d", pos, bytes);

	/*
	for (i = 0; i < frame_count; i++)
		for (c = 0; c < ms->channels; c++)
			buf_interleaved[i * ms->channels + c] =
			    matrixio_buf[c][frame_pos + i];
	// return copy_to_user(buf, buf_interleaved, bytes);
*/
	queue_work(ms->wq, &ms->work);

	return frame_count;
}
#endif

static struct snd_pcm_ops matrixio_playback_ops = {
    .open = matrixio_playback_open,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = matrixio_playback_hw_params,
    .hw_free = matrixio_playback_hw_free,
    .prepare = matrixio_playback_prepare,
    .pointer = matrixio_playback_pointer,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 13, 0)
    .copy = matrixio_playback_copy,
#else
    .copy_user = matrixio_playback_copy,
#endif
    .close = matrixio_playback_close,
};

static int matrixio_playback_new(struct snd_soc_pcm_runtime *rtd) { return 0; }

static const struct snd_soc_platform_driver matrixio_soc_platform = {
    .ops = &matrixio_playback_ops, .pcm_new = matrixio_playback_new,
};

static int matrixio_playback_platform_probe(struct platform_device *pdev)
{
	int ret;

	ms = devm_kzalloc(&pdev->dev, sizeof(struct matrixio_substream),
			  GFP_KERNEL);
	if (!ms) {
		dev_err(&pdev->dev, "data allocation");
		return -ENOMEM;
	}

	ms->mio = dev_get_drvdata(pdev->dev.parent);

	ms->substream = 0;

	mutex_init(&ms->lock);

	ret =
	    devm_snd_soc_register_platform(&pdev->dev, &matrixio_soc_platform);
	if (ret) {
		dev_err(&pdev->dev,
			"MATRIXIO sound SoC register platform error: %d", ret);
		return ret;
	}

	dev_set_drvdata(&pdev->dev, ms);

	return 0;
}

static const struct of_device_id snd_matrixio_playback_of_match[] = {
    {
	.compatible = "matrixio-playback",
    },
    {},
};
MODULE_DEVICE_TABLE(of, snd_matrixio_playback_of_match);

static struct platform_driver matrixio_codec_driver = {
    .driver = {.name = "matrixio-playback",
	       .owner = THIS_MODULE,
	       .of_match_table = snd_matrixio_playback_of_match},
    .probe = matrixio_playback_platform_probe,
};

module_platform_driver(matrixio_codec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO PCM module");
