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

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "matrixio-core.h"

#define MATRIXIO_CHANNELS_MAX 8
#define MATRIXIO_RATES SNDRV_PCM_RATE_8000_48000
#define MATRIXIO_FORMATS SNDRV_PCM_FMTBIT_S16_LE
#define MATRIXIO_MICARRAY_BASE 0x2000
#define MATRIXIO_MICARRAY_BUFFER_SIZE (256 * MATRIXIO_CHANNELS_MAX * 2)
#define MATRIXIO_FIFO_SIZE (MATRIXIO_MICARRAY_BUFFER_SIZE * 4)

struct matrixio_substream {
	struct matrixio *mio;
	int irq;
	spinlock_t lock;
	struct snd_pcm_substream *substream;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	int force_end_work;
};

struct kfifo_rec_ptr_2 pcm_fifo;

struct matrixio_substream *ms;

static struct class *cl;

static struct cdev matrixio_pcm_cdev;

static DEFINE_MUTEX(read_lock);

static DECLARE_WAIT_QUEUE_HEAD(wq);

static irqreturn_t matrixio_dai_interrupt(int irq, void *irq_data)
{
	queue_work(ms->workqueue, &ms->work);
	return IRQ_HANDLED;
}

static void matrixio_pcm_work(struct work_struct *w)
{
	unsigned long flags;

	spin_lock_irqsave(&ms->lock, flags);
/*
	matrixio_hw_read_enqueue(ms->mio, MATRIXIO_MICARRAY_BASE,
				 MATRIXIO_MICARRAY_BUFFER_SIZE, &pcm_fifo);

**/
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

static int matrixio_startup(struct snd_pcm_substream *substream)
{
	ms->substream = substream;

	printk(KERN_INFO "startup");
	return 0;
}

static int matrixio_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	return 0;
}

static struct snd_soc_ops matrixio_snd_ops = {
    .startup = matrixio_startup, .hw_params = matrixio_hw_params,
};

static struct snd_soc_dai_link matrixio_snd_soc_dai[] = {{
    .name = "matrixio.0",
    .stream_name = "matrixio.0",
    .codec_dai_name = "snd-soc-dummy-dai",
    .cpu_dai_name = "matrixio-dai.0",
    .codec_name = "snd-soc-dummy",
    .ops = &matrixio_snd_ops,
}};

static struct snd_soc_card matrixio_soc_card = {
    .name = "MATRIXIO_HAT",
    .owner = THIS_MODULE,
    .dai_link = matrixio_snd_soc_dai,
    .num_links = ARRAY_SIZE(matrixio_snd_soc_dai),
};

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

	// snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &matrixio_pcm_ops);

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

static const struct snd_soc_dai_ops matrixio_dai_ops = {
    .hw_params = matrixio_codec_hw_params,
    .prepare = matrixio_codec_prepare,
    .set_fmt = matrixio_codec_set_fmt,
    .digital_mute = matrixio_codec_dai_digital_mute,
    .startup = matrixio_codec_dai_startup,
    .shutdown = matrixio_codec_dai_shutdown,
    .trigger = matrixio_codec_dai_trigger,
};

static const DECLARE_TLV_DB_SCALE(inpga_tlv, -1000, 100, 0);

static const struct snd_kcontrol_new matrixio_snd_controls[] = {
    SOC_SINGLE_TLV("MIC ARRAY Volume", 0x001, 6, 7, 0, inpga_tlv),
};

static const struct snd_soc_dapm_widget matrixio_dapm_widgets[] = {
    SND_SOC_DAPM_INPUT("MIC0"), SND_SOC_DAPM_INPUT("MIC1"),
    SND_SOC_DAPM_INPUT("MIC2"), SND_SOC_DAPM_INPUT("MIC3"),
};

static const struct snd_soc_dapm_route matrixio_dapm_routes[] = {};

static int matrixio_codec_probe(struct snd_soc_codec *codec)
{
	//	struct matrixio_substream *ms =
	// snd_soc_codec_get_drvdata(codec);

	snd_soc_codec_init_regmap(codec, ms->mio->regmap);
	return 0;
}

static const struct snd_soc_codec_driver matrixio_soc_codec_driver = {

    .probe = matrixio_codec_probe,
    .component_driver =
	{
	    .controls = matrixio_snd_controls,
	    .num_controls = ARRAY_SIZE(matrixio_snd_controls),
	    .dapm_widgets = matrixio_dapm_widgets,
	    .num_dapm_widgets = ARRAY_SIZE(matrixio_dapm_widgets),
	    .dapm_routes = matrixio_dapm_routes,
	    .num_dapm_routes = ARRAY_SIZE(matrixio_dapm_routes),

	},
};

static struct snd_soc_dai_driver matrixio_dai_driver[] = {{
    .name = "matrixio-dai.0",
    .capture =
	{
	    .stream_name = "matrixio.mic.0",
	    .channels_min = 1,
	    .channels_max = 1,
	    .rates = MATRIXIO_RATES,
	    .rate_min = 8000,
	    .rate_max = 48000,
	    .formats = MATRIXIO_FORMATS,
	},
    .ops = &matrixio_dai_ops,
}};

static int matrixio_probe(struct platform_device *pdev)
{
	int ret;
	char workqueue_name[12];
	dev_t devt;
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &matrixio_soc_card;

	card->dev = &pdev->dev;

	ms = devm_kzalloc(&pdev->dev, sizeof(struct matrixio_substream),
			  GFP_KERNEL);
	if (!ms)
		return -ENOMEM;

	ms->substream = 0;

	spin_lock_init(&ms->lock);

	platform_set_drvdata(pdev, ms);

	ms->mio = dev_get_drvdata(pdev->dev.parent);

	ret = snd_soc_register_codec(&pdev->dev, &matrixio_soc_codec_driver,
				     matrixio_dai_driver,
				     ARRAY_SIZE(matrixio_dai_driver));

	if (ret) {
		dev_err(&pdev->dev, "Failed to register MATRIXIO codec: %d\n",
			ret);
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);

	if (ret) {
		dev_err(&pdev->dev, "Failed to register MATRIXIO card: %d\n",
			ret);
		return ret;
	}

	sprintf(workqueue_name, "matrixio_pcm");

	ms->workqueue = create_freezable_workqueue(workqueue_name);

	if (!ms->workqueue) {
		dev_err(&pdev->dev, "cannot create workqueue");
		return -EBUSY;
	}

	ms->force_end_work = 0;

	INIT_WORK(&ms->work, matrixio_pcm_work);

	ms->irq = irq_of_parse_and_map(np, 0);

	ret = devm_request_irq(&pdev->dev, ms->irq, matrixio_dai_interrupt, 0,
			       "matrixio-audio", ms);

	if (ret) {
		dev_err(&pdev->dev, "can't request irq %d\n", ms->irq);
		destroy_workqueue(ms->workqueue);
		return -EBUSY;
	}

	dev_notice(&pdev->dev, "MATRIXIO audio drive loaded (IRQ=%d)", ms->irq);

	ret = kfifo_alloc(&pcm_fifo, MATRIXIO_FIFO_SIZE, GFP_KERNEL);

	if (ret)
		dev_err(&pdev->dev, "error PCM kfifo allocation");

	alloc_chrdev_region(&devt, 0, 1, "matrixio_pcm");
	cl = class_create(THIS_MODULE, "matrixio_pcm");

	device_create(cl, NULL, devt, NULL, "matrixio_pcm");

	cdev_init(&matrixio_pcm_cdev, &matrixio_pcm_file_ops);
	cdev_add(&matrixio_pcm_cdev, devt, 1);

	return ret;
}

static int matrixio_codec_remove(struct platform_device *pdev) { return 0; }

static const struct of_device_id snd_matrixio_codec_of_match[] = {
    {
	.compatible = "matrixio-codec",
    },
    {},
};
MODULE_DEVICE_TABLE(of, snd_matrixio_codec_of_match);

static struct platform_driver matrixio_codec_driver = {
    .driver = {.name = "matrixio-codec",
	       .owner = THIS_MODULE,
	       .of_match_table = snd_matrixio_codec_of_match},
    .probe = matrixio_probe,

    .remove = matrixio_codec_remove,
};

module_platform_driver(matrixio_codec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO audio module");
