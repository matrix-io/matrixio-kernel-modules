/*
 * matrix-codec.c -- MATRIX microphone array audio driver
 *
 * Copyright 2017 MATRIX Labs
 *
 * Author: Andres Calderon <andres.calderon@admobilize.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include "matrixio-core.h"
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define MATRIXIO_CHANNELS_MAX 8
#define MATRIXIO_RATES SNDRV_PCM_RATE_8000_48000
#define MATRIXIO_FORMATS SNDRV_PCM_FMTBIT_S16_LE
#define MATRIXIO_MICARRAY_BASE 0x1800
#define MATRIXIO_MICARRAY_BUFFER_SIZE (128 * MATRIXIO_CHANNELS_MAX * 2)
#define MATRIXIO_FIFO_SIZE (MATRIXIO_MICARRAY_BUFFER_SIZE * 4)

struct matrixio_soc_device {
	struct matrixio *mio;
};

struct matrixio_substream {
	struct matrixio *mio;
	int irq;
	spinlock_t lock;
	struct snd_pcm_substream *substream;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	int force_end_work;
};

struct matrixio_soc_device matrixio_soc_device;

struct kfifo_rec_ptr_2 pcm_fifo;

struct matrixio_substream *ms;

static struct class *cl;

static struct snd_pcm *matrixio_pcm;

static struct cdev matrixio_pcm_cdev;

static DEFINE_MUTEX(read_lock);

static DECLARE_WAIT_QUEUE_HEAD(wq);

static struct snd_pcm_hardware matrixio_pcm_hardware = {
    .info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
	    SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_PAUSE |
	    SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_NO_PERIOD_WAKEUP,
    .period_bytes_min = 32,
    .period_bytes_max = 64 * 1024,
    .periods_min = 2,
    .periods_max = 255,
    .buffer_bytes_max = 128 * 1024,
};

static irqreturn_t matrixio_dai_interrupt(int irq, void *irq_data)
{
	queue_work(ms->workqueue, &ms->work);
	return IRQ_HANDLED;
}

static void matrixio_pcm_work(struct work_struct *w)
{
	unsigned long flags;

	spin_lock_irqsave(&ms->lock, flags);

	matrixio_hw_read_enqueue(ms->mio, MATRIXIO_MICARRAY_BASE,
				 MATRIXIO_MICARRAY_BUFFER_SIZE, &pcm_fifo);

	spin_unlock_irqrestore(&ms->lock, flags);

	wake_up_interruptible(&wq);
}

static int pcm_fifo_open(struct inode *inode, struct file *file)
{
	kfifo_reset(&pcm_fifo);

	return 0;
}

static ssize_t pcm_fifo_read(struct file *file, char __user *buf, size_t count,
			     loff_t *ppos)
{
	int ret;
	unsigned int chunks, copied;

	if (count > MATRIXIO_FIFO_SIZE)
		return -EIO;

	if (mutex_lock_interruptible(&read_lock))
		return -ERESTARTSYS;

	if (wait_event_interruptible(wq, kfifo_len(&pcm_fifo) != 0))
		goto erestartsys;

	chunks = count / MATRIXIO_MICARRAY_BUFFER_SIZE;

	ret = kfifo_to_user(&pcm_fifo, buf,
			    chunks * MATRIXIO_MICARRAY_BUFFER_SIZE, &copied);

	mutex_unlock(&read_lock);

	return ret ? ret : copied;

erestartsys:
	mutex_unlock(&read_lock);
	return -ERESTARTSYS;
}

struct file_operations matrixio_pcm_file_ops = {.owner = THIS_MODULE,
						.read = pcm_fifo_read,
						.open = pcm_fifo_open,
						.llseek = noop_llseek};

static int matrixio_pcm_open(struct snd_pcm_substream *substream)
{
	int ret;
	printk(KERN_INFO "-------------pcm_open");

	ret = snd_soc_set_runtime_hwparams(substream, &matrixio_pcm_hardware);

	if (ret)
		return ret;

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	return 0;
}

static int matrixio_pcm_close(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "-------------pcm_close");
	return 0;
}

static int matrixio_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	printk(KERN_INFO "-------------pcm hw params");
	return 0;
}

static int matrixio_pcm_hw_free(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "-------------pcm hw free");

	// snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int matrixio_pcm_prepare(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "-------------pcm prepare");
	return 0;
}

static int matrixio_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	printk(KERN_INFO "-------------pcm trigger");
	return 0;
}

static snd_pcm_uframes_t
matrixio_pcm_pointer(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "-------------pcm pointer");
	snd_pcm_uframes_t offset;

	return offset;
}

static struct snd_pcm_ops matrixio_pcm_ops = {
    .open = matrixio_pcm_open,
    .close = matrixio_pcm_close,
    .ioctl = snd_pcm_lib_ioctl,
    .prepare = matrixio_pcm_prepare,
    .hw_params = matrixio_pcm_hw_params,
    .hw_free = matrixio_pcm_hw_free,
    .trigger = matrixio_pcm_trigger,
    .pointer = matrixio_pcm_pointer,
    // .mmap = matrixio_pcm_mmap,
};

static int matrixio_startup(struct snd_pcm_substream *substream)
{
	ms->substream = substream;
	printk(KERN_INFO "startup %p", ms->substream->pcm);
	/*
	snd_pcm_set_ops(ms->substream->pcm, SNDRV_PCM_STREAM_CAPTURE,
					     &matrixio_pcm_ops);

	*/
	return 0;
}

static int matrixio_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	printk(KERN_INFO "hw_paramsi %p", substream->pcm);
	return 0;
}

static int matrixio_codec_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{

	printk(KERN_INFO "::rate %d", params_rate(params));

	return 0;
}

static int matrixio_codec_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	printk(KERN_INFO "::fmt %d", fmt);
	return 0;
}

static int matrixio_codec_prepare(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	printk(KERN_INFO "::prepare");
	return 0;
}

/* codec dai component */
static int matrixio_codec_dai_startup(struct snd_pcm_substream *substream,
				      struct snd_soc_dai *codec_dai)
{
	printk(KERN_INFO "::DAI STARTUP");
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm *pcm = rtd->pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &matrixio_pcm_ops);

	pcm->info_flags = 0;
	return 0;
}

static int matrixio_codec_dai_digital_mute(struct snd_soc_dai *codec_dai,
					   int mute)
{
	printk(KERN_INFO "::DIGITAL MUTE");
	return 0;
}

static int matrixio_codec_dai_trigger(struct snd_pcm_substream *substream,
				      int cmd, struct snd_soc_dai *codec_dai)
{
	printk(KERN_INFO "::trigger");
	return 0;
}

static void matrixio_codec_dai_shutdown(struct snd_pcm_substream *substream,
					struct snd_soc_dai *codec_dai)
{
	printk(KERN_INFO "::shutdown");
}


static const struct snd_soc_dapm_route matrixio_dapm_routes[] = {};

static int matrixio_codec_probe(struct snd_soc_codec *codec)
{
	//	struct matrixio_substream *ms =
	// snd_soc_codec_get_drvdata(codec);

	snd_soc_codec_init_regmap(codec, ms->mio->regmap);
	return 0;
}

static int matrixio_pcm_probe(struct snd_soc_platform *platform)
{
	snd_soc_platform_set_drvdata(platform, &matrixio_soc_device);
	return 0;
}

static int matrixio_pcm_remove(struct snd_soc_platform *platform) { return 0; }

static int matrixio_pcm_new(struct snd_soc_pcm_runtime *rtd) { return 0; }

static const struct snd_soc_platform_driver matrixio_soc_platform = {
    .probe = matrixio_pcm_probe,
    .remove = matrixio_pcm_remove,
    .ops = &matrixio_pcm_ops,
    .pcm_new = matrixio_pcm_new,
};

static int matrixio_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_platform(&pdev->dev,
					      &matrixio_soc_platform);
}

static const struct of_device_id snd_matrixio_codec_of_match[] = {
    {
	.compatible = "matrixio-pcm",
    },
    {},
};
MODULE_DEVICE_TABLE(of, snd_matrixio_codec_of_match);

static struct platform_driver matrixio_codec_driver = {
    .driver = {.name = "matrixio-pcm",
	       .owner = THIS_MODULE,
	       .of_match_table = snd_matrixio_codec_of_match},
    .probe = matrixio_probe,
};

module_platform_driver(matrixio_codec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO PCM module");
