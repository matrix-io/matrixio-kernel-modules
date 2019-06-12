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
#include <linux/kfifo.h>
#include <linux/kthread.h>
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
#define kFIFOSize 4096
#define KERNEL_FIFO_SIZE 32768

static struct semaphore sem;

const uint16_t kConfBaseAddress = 0x0000;

static struct matrixio_substream *ms;

static DECLARE_KFIFO(pcm_fifo, unsigned char, KERNEL_FIFO_SIZE);

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

struct task_struct *playback_task;

static uint16_t matrixio_fifo_status(void)
{
	uint16_t write_pointer;
	uint16_t read_pointer;

	matrixio_read(ms->mio, MATRIXIO_PLAYBACK_BASE + 0x802, sizeof(uint16_t),
		      &read_pointer);
	matrixio_read(ms->mio, MATRIXIO_PLAYBACK_BASE + 0x803, sizeof(uint16_t),
		      &write_pointer);

	if (write_pointer >= read_pointer)
		return write_pointer - read_pointer;
	return kFIFOSize - read_pointer + write_pointer;
}

static uint16_t matrixio_flush(void)
{
	uint16_t flush_data = 0x0001;

	matrixio_write(ms->mio, MATRIXIO_CONF_BASE + 12, sizeof(uint16_t),
		       &flush_data);

	flush_data = 0x0000;

	return matrixio_write(ms->mio, MATRIXIO_CONF_BASE + 12,
			      sizeof(uint16_t), &flush_data);
}

static int thread_pcm_playback(void *data)
{
	int ret;
	uint16_t fifo_status;
	static unsigned char matrixio_pb_buf[MATRIXIO_MICARRAY_BUFFER_SIZE];

	while (!kthread_should_stop()) {
		down(&sem);

		if (ms->playback_params == 0)
			continue;

		do {
			fifo_status = matrixio_fifo_status();

			/* When FIFO is empty go sem.down to wait for more frames */
			if (kfifo_len(&pcm_fifo) == 0)
				break;

			if (fifo_status > kFIFOSize * 3 / 4) {
				udelay(MATRIXIO_MICARRAY_BUFFER_SIZE *
				       ms->playback_params->bit_time);
				continue;
			}

			ret = kfifo_out(&pcm_fifo, matrixio_pb_buf,
					MATRIXIO_MICARRAY_BUFFER_SIZE);

			matrixio_write(ms->mio, MATRIXIO_PLAYBACK_BASE,
				       MATRIXIO_MICARRAY_BUFFER_SIZE,
				       (void *)matrixio_pb_buf);


			ms->position += MATRIXIO_MICARRAY_BUFFER_SIZE;

			snd_pcm_period_elapsed(ms->substream);
		} while (kfifo_len(&pcm_fifo) >= MATRIXIO_MICARRAY_BUFFER_SIZE);
	}

	return 0;
}

static int matrixio_playback_open(struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &matrixio_playback_capture_hw);

	snd_pcm_set_sync(substream);

	if (ms->substream != NULL) {
		return -EBUSY;
	}

	ms->substream = substream;

	ms->position = 0;

	matrixio_flush();

	sema_init(&sem, 1);

	kfifo_reset(&pcm_fifo);

	playback_task =
	    kthread_run(&thread_pcm_playback, (void *)ms, "matrixio_playback");

	return 0;
}

static int matrixio_playback_close(struct snd_pcm_substream *substream)
{
	up(&sem);

	kthread_stop(playback_task);

	ms->substream = 0;

	ms->playback_params = 0;

	return 0;
}

static int matrixio_playback_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *hw_params)
{
	int i;
	int rate;

	ms->channels = params_channels(hw_params);

	if (snd_pcm_format_width(params_format(hw_params)) != 16)
		return -EINVAL;

	rate = params_rate(hw_params);
	for (i = 0; i < ARRAY_SIZE(pcm_sampling_frequencies); i++) {
		if (rate == pcm_sampling_frequencies[i].rate) {
			ms->playback_params =
			    (struct playback_params
				 *)&pcm_sampling_frequencies[i];
			return matrixio_reg_write(
			    ms->mio, MATRIXIO_CONF_BASE + 9,
			    pcm_sampling_frequencies[i].bit_time);
		}
	}
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
	ms->position =
	    ms->position < frames_to_bytes(runtime, runtime->buffer_size)
		? ms->position
		: 0;

	mutex_unlock(&ms->lock);

	return bytes_to_frames(runtime, ms->position);
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 13, 0)
static int matrixio_playback_copy(struct snd_pcm_substream *substream,
				  int channel, snd_pcm_uframes_t pos,
				  void __user *buf, snd_pcm_uframes_t count)
{
	return -EINVAL;
}
#else
static int matrixio_playback_copy(struct snd_pcm_substream *substream,
				  int channel, snd_pcm_uframes_t pos,
				  void __user *buf, snd_pcm_uframes_t bytes)
{
	int ret;
	unsigned int copied;

	struct snd_pcm_runtime *runtime = substream->runtime;

	// int frame_pos = bytes_to_frames(runtime, pos);
	int frame_count = bytes_to_frames(runtime, bytes);

	ret = kfifo_from_user(&pcm_fifo, buf, bytes, &copied);
	up(&sem);

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

static int matrixio_playback_select_info(struct snd_kcontrol *kcontrol,
                                         struct snd_ctl_elem_info *uinfo)
{
    static const char *texts[2] = {
        "Speaker", "Headphone"
    };
    return snd_ctl_enum_info(uinfo, 1, 2, texts);
}

static int matrixio_playback_select_get(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
	uint16_t config = 0;
	matrixio_read(ms->mio, MATRIXIO_CONF_BASE + 11,
		      sizeof(uint16_t), &config);
	ucontrol->value.integer.value[0] = config;
	return 0;
}

static int matrixio_playback_select_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	uint16_t config = ucontrol->value.integer.value[0] ? 0x0001 : 0x0000;
	matrixio_write(ms->mio, MATRIXIO_CONF_BASE + 11,
			sizeof(uint16_t), &config);
	return 1;
}

#define MAX_VOLUME 32

static int matrixio_volume_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = MAX_VOLUME;
	uinfo->value.integer.step = 1;
	return 0;
}

static int matrixio_volume_get(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
	uint16_t volume_regvalue;
	matrixio_read(ms->mio, MATRIXIO_CONF_BASE + 0x08,
			      sizeof(uint16_t), &volume_regvalue);
	ucontrol->value.integer.value[0] = MAX_VOLUME - volume_regvalue;
	return 0;
}

static int matrixio_volume_put(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
	uint16_t volume_regvalue = MAX_VOLUME - ucontrol->value.integer.value[0]; 
	matrixio_write(ms->mio, MATRIXIO_CONF_BASE + 0x08,
			      sizeof(uint16_t), &volume_regvalue);
	return 1;
}

//static DECLARE_TLV_DB_SCALE(matrixio_volume_db_scale, 2000, -125, 0);

static const struct snd_kcontrol_new matrixio_snd_controls[] = {
{
    .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
    .name = "Master_Playback_Switch",
    .index = 0,
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
    .info = matrixio_playback_select_info,
    .get = matrixio_playback_select_get,
    .put = matrixio_playback_select_put,
},
{
    .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
    .name = "Playback_Volume",
    .index = 1,
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
    .info = matrixio_volume_info,
    .get = matrixio_volume_get,
    .put = matrixio_volume_put,
}};

static int matrixio_playback_new(struct snd_soc_pcm_runtime *rtd) { return 0; }

static const struct snd_soc_component_driver matrixio_soc_platform = {
    .ops = &matrixio_playback_ops, .pcm_new = matrixio_playback_new,
	.controls = matrixio_snd_controls,
	.num_controls = ARRAY_SIZE(matrixio_snd_controls),
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
	    devm_snd_soc_register_component(&pdev->dev, &matrixio_soc_platform, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev,
			"MATRIXIO sound SoC register platform error: %d", ret);
		return ret;
	}

	dev_set_drvdata(&pdev->dev, ms);

	INIT_KFIFO(pcm_fifo);

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
