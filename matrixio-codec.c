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

#include <linux/init.h>
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
#define MATRIXIO_MICARRAY_BASE 0x1800
#define MATRIXIO_MICARRAY_BUFFER_SIZE 1024

static struct snd_pcm_hardware matrixio_hw_capture = {
    .info = SNDRV_PCM_INFO_INTERLEAVED,
    .formats = MATRIXIO_FORMATS,
    .rates = MATRIXIO_RATES,
    .rate_min = 8000,
    .rate_max = 48000,
    .channels_min = 1,
    .channels_max = 1,
    .buffer_bytes_max = 128*8*2,
    .period_bytes_min = 128*8*2,
    .period_bytes_max = 128*8*2,
    .periods_min = 1,
    .periods_max = 1,
    .fifo_size = 128*8*2,
};

static int matrixio_pcm_open(struct snd_pcm_substream *substream)
{
	int ret;
	printk(KERN_INFO "matrixio_pcm_open");
	struct snd_pcm_runtime *runtime = substream->runtime;
	// runtime->hw = matrixio_hw_capture;

	ret = snd_pcm_limit_hw_rates(runtime);
	if (ret)
		return ret;

	snd_soc_set_runtime_hwparams(substream, &matrixio_hw_capture);

	return 0;
}

/* close callback */
static int matrixio_pcm_close(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "matrixio_pcm_close");
	//	struct mychip *chip = snd_pcm_substream_chip(substream);
	/* the hardware-specific codes will be here */
	return 0;
}

/* hw_params callback */
static int matrixio_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	printk(KERN_INFO "matrixio_pcm_hw_params");
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

/* hw_free callback */
static int matrixio_pcm_hw_free(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "matrixio_pcm_hw_free");
	return snd_pcm_lib_free_pages(substream);
}

/* prepare callback */
static int matrixio_pcm_prepare(struct snd_pcm_substream *substream)
{
	printk(KERN_INFO "matrixio_pcm_prepare");
	/* irqreturn_t
	struct mychip *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	/* set up the hardware with the current configuration
	 *            * for example...
	 *                       */
	/*
		mychip_set_sample_format(chip, runtime->format);
		mychip_set_sample_rate(chip, runtime->rate);
		mychip_set_channels(chip, runtime->channels);
		mychip_set_dma_setup(chip, runtime->dma_addr, chip->buffer_size,
				     chip->period_size);
	*/
	return 0;
}

/* trigger callback */
static int matrixio_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	printk(KERN_INFO "matrixio_pcm_trigger");
	return 0;
#if 0
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* do something to start the PCM engine */
		....break;
	case SNDRV_PCM_TRIGGER_STOP:
		/* do something to stop the PCM engine */
		....break;
	default:
		return -EINVAL;
	}
#endif
}

/* pointer callback */
static snd_pcm_uframes_t
matrixio_pcm_pointer(struct snd_pcm_substream *substream)
{
	// struct mychip *chip = snd_pcm_substream_chip(substream);
	unsigned int current_ptr;

	printk(KERN_INFO "matrixio_pcm_pointer");
	/* get the current hardware pointer */
	// current_ptr = mychip_get_hw_pointer(chip);
	return 0; // current_ptr;
}

static struct snd_pcm_ops matrixio_pcm_ops = {
    .open = matrixio_pcm_open,
    .close = matrixio_pcm_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = matrixio_pcm_hw_params,
    .hw_free = matrixio_pcm_hw_free,
    .prepare = matrixio_pcm_prepare,
    .trigger = matrixio_pcm_trigger,
    .pointer = matrixio_pcm_pointer,
};

struct matrixio_substream {
	struct matrixio *mio;
	int irq;
	spinlock_t lock;
	struct snd_pcm_substream *substream;
};

struct matrixio_substream *ms;

static int16_t raw_data[MATRIXIO_MICARRAY_BUFFER_SIZE];

static irqreturn_t matrixio_dai_interrupt(int irq, void *irq_data)
{
	unsigned long flags;

	spin_lock_irqsave(&ms->lock, flags);

	if (ms->substream) {
		matrixio_hw_read_burst(
		    ms->mio, MATRIXIO_MICARRAY_BASE,
		    MATRIXIO_MICARRAY_BUFFER_SIZE * sizeof(int16_t), raw_data);

		//		printk(KERN_INFO ".");

		//		snd_pcm_stop_xrun(ms->substream);
	}
	spin_unlock_irqrestore(&ms->lock, flags);

	return IRQ_HANDLED;
}

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

static struct snd_soc_dai_link matrixio_snd_soc_dai = {
    .name = "matrixio",
    .stream_name = "matrixio",
    .codec_dai_name = "snd-soc-dummy-dai",
    .cpu_dai_name = "matrixio-dai",
    //	.platform_name	= "rpi-matrixio-pcm",
    .codec_name = "snd-soc-dummy",
    .ops = &matrixio_snd_ops,

};

static struct snd_soc_card matrixio_soc_card = {
    .name = "MATRIXIO_HAT",
    .owner = THIS_MODULE,
    .dai_link = &matrixio_snd_soc_dai,
    .num_links = 1,
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

//snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &matrixio_pcm_ops);

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

	printk(KERN_INFO "matrixio_codec_probe");
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

static struct snd_soc_dai_driver matrixio_dai_driver = {
    .name = "matrixio-dai",
    .capture =
	{
	    .stream_name = "Micarray Capture",
	    .channels_min = 1,
	    .channels_max = 1,
	    .rates = MATRIXIO_RATES,
	    .rate_min = 8000,
	    .rate_max = 48000,
	    .formats = MATRIXIO_FORMATS,
	},
    .ops = &matrixio_dai_ops,
};

static struct snd_soc_platform_driver matrixio_snd_soc_platform = {
    .ops = &matrixio_pcm_ops,
    //.pcm_new	= bf5xx_pcm_i2s_new,
};

static int matrixio_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &matrixio_soc_card;

	card->dev = &pdev->dev;

	printk(KERN_INFO "matrixio_probe");

	ms = devm_kzalloc(&pdev->dev, sizeof(struct matrixio_substream),
			  GFP_KERNEL);
	if (!ms)
		return -ENOMEM;

	ms->substream = 0;

	spin_lock_init(&ms->lock);

	platform_set_drvdata(pdev, ms);

	ms->mio = dev_get_drvdata(pdev->dev.parent);

	ret = snd_soc_register_codec(&pdev->dev, &matrixio_soc_codec_driver,
				     &matrixio_dai_driver, 1);

	if (ret) {
		dev_err(&pdev->dev, "Failed to register MATRIXIO codec: %d\n",
			ret);

		return ret;
	}

	printk(KERN_INFO "MATRIXIO codec registered\n");

	ret = devm_snd_soc_register_card(&pdev->dev, card);

	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to register MATRIXIO card: %d\n",
			ret);
		return ret;
	}

	ms->irq = irq_of_parse_and_map(np, 0);

	ret = devm_request_irq(&pdev->dev, ms->irq, matrixio_dai_interrupt, 0,
			       "matrixio-audio", ms);

	if (ret)
		dev_err(&pdev->dev, "can't request irq %d\n", ms->irq);

	printk(KERN_INFO "MATRIX AUDIO has been loaded (IRQ=%d,%d)", ms->irq,
	       ret);

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
