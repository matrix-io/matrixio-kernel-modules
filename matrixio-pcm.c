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

#include "matrixio-pcm.h"
#include "matrixio-core.h"

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
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
#include <linux/dma-mapping.h>
#include <asm/dma.h>

#define MATRIXIO_CHANNELS_MAX 8
#define MATRIXIO_RATES SNDRV_PCM_RATE_8000_48000
#define MATRIXIO_FORMATS SNDRV_PCM_FMTBIT_S16_LE
#define MATRIXIO_MICARRAY_BASE 0x1800
#define MATRIXIO_MICARRAY_BUFFER_SIZE (128 * MATRIXIO_CHANNELS_MAX * 2)
#define MATRIXIO_FIFO_SIZE (MATRIXIO_MICARRAY_BUFFER_SIZE * 4)

unsigned int ptr;

static struct snd_pcm *matrixio_pcm;

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

static irqreturn_t matrixio_pcm_interrupt(int irq, void *irq_data)
{
	struct matrixio_substream *ms = irq_data;

	queue_work(ms->wq, &ms->work);

	return IRQ_HANDLED;
}

static void matrixio_pcm_work(struct work_struct *work)
{
	unsigned long flags;
	struct matrixio_substream *ms;
	//	spin_lock_irqsave(&ms->lock, flags)
	ms = container_of(work, struct matrixio_substream, work);

	// printk(KERN_INFO "%d\n", ms->stamp);
	//
	//							    event_work);
	/*
		matrixio_hw_read_enqueue(ms->mio, MATRIXIO_MICARRAY_BASE,
					 MATRIXIO_MICARRAY_BUFFER_SIZE,
	   &pcm_fifo);

	*/
	//	spin_unlock_irqrestore(&ms->lock, flags);

	//	wake_up_interruptible(&wq);
}

static int matrixio_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	void *data;

	printk(KERN_INFO "-------------pcm_open");

	snd_soc_set_runtime_hwparams(substream, &matrixio_pcm_hardware);
	data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	runtime->private_data = data;

	return 0;
}

static int matrixio_pcm_close(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "-------------pcm_close");
	synchronize_rcu();
	return 0;
}

static int matrixio_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	int ret;
	printk(KERN_INFO "-------------pcm hw params");

	// ret = snd_pcm_lib_malloc_pages(substream,
	// params_buffer_bytes(hw_params));
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
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct matrixio_substream *ms = runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		rcu_assign_pointer(ms->rx_substream, substream);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		rcu_assign_pointer(ms->rx_substream, NULL);
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static snd_pcm_uframes_t
matrixio_pcm_pointer(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "-------------pcm pointer ");
	ptr += 128;
	snd_pcm_uframes_t offset = ptr;

	return offset;
}

static int matrixio_pcm_ioctl(struct snd_pcm_substream *substream,
			      unsigned int cmd, void *arg)
{
	printk(KERN_INFO "---------------pcm ioctl : [%d]", cmd);
	switch (cmd) {
	case SNDRV_PCM_IOCTL1_INFO:
		return 0;
	case SNDRV_PCM_IOCTL1_RESET:
		printk(KERN_INFO "reset");
		return 0; // snd_pcm_lib_ioctl_reset(substream, arg);
	case SNDRV_PCM_IOCTL1_CHANNEL_INFO:
		printk(KERN_INFO "info");
		return 0; // snd_pcm_lib_ioctl_channel_info(substream, arg);
	case SNDRV_PCM_IOCTL1_FIFO_SIZE:
		printk(KERN_INFO "size");
		return 0; // snd_pcm_lib_ioctl_fifo_size(substream, arg);
	}
	return -ENXIO;

	return 0;
}

static int matrixio_pcm_copy(struct snd_pcm_substream *substream, int channel,
			     snd_pcm_uframes_t pos, void __user *buf,
			     snd_pcm_uframes_t count)
{
	printk(KERN_INFO "-----------------pcm copy channel=%d pos=%d count=%d",
	       channel, pos, count);
	return 0;
}

static struct snd_pcm_ops matrixio_pcm_ops = {
    .open = matrixio_pcm_open,
    .close = matrixio_pcm_close,
    //   .copy = matrixio_pcm_copy,
    .ioctl = snd_pcm_lib_ioctl,
    //    .prepare = matrixio_pcm_prepare,
    .hw_params = matrixio_pcm_hw_params,
    //  .hw_free = matrixio_pcm_hw_free,
    .trigger = matrixio_pcm_trigger,
    .pointer = matrixio_pcm_pointer,
};

static int matrixio_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	size_t size = matrixio_pcm_hardware.buffer_bytes_max;
	int ret;

	printk(KERN_INFO "matrixio_pcm_new");

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	return snd_pcm_lib_preallocate_pages_for_all(
	    rtd->pcm, SNDRV_DMA_TYPE_DEV, card->dev, size, size);
}

static const struct snd_soc_platform_driver matrixio_soc_platform = {
    .ops = &matrixio_pcm_ops, .pcm_new = matrixio_pcm_new,
};

static int matrixio_pcm_platform_probe(struct platform_device *pdev)
{
	int ret;
	char workqueue_name[12];
	struct matrixio_substream *ms;

	sprintf(workqueue_name, "matrixio_pcm");

	ms = devm_kzalloc(&pdev->dev, sizeof(struct matrixio_substream),
			  GFP_KERNEL);
	if (!ms)
		return -ENOMEM;

	ms->stamp = 1010101;
	ms->wq = create_singlethread_workqueue(workqueue_name);

	if (!ms->wq) {
		dev_err(&pdev->dev, "cannot create workqueue");
		return -EBUSY;
	}

	INIT_WORK(&ms->work, matrixio_pcm_work);

	ms->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);

	dev_notice(&pdev->dev, "MATRIXIO pcm irq=%d", ms->irq);

	ret = devm_request_irq(&pdev->dev, ms->irq, matrixio_pcm_interrupt, 0,
			       dev_name(&pdev->dev), ms);
	if (ret) {
		dev_err(&pdev->dev, "can't request irq %d\n", ms->irq);
		destroy_workqueue(ms->wq);
		return -EBUSY;
	}

	dev_notice(&pdev->dev, "MATRIXIO audio drive loaded (IRQ=%d)", ms->irq);

	return devm_snd_soc_register_platform(&pdev->dev,
					      &matrixio_soc_platform);
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
