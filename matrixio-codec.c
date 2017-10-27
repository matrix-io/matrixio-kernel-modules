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

static struct matrixio *matrixio;

static int matrixio_startup(struct snd_pcm_substream *substream)
{
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

static const struct snd_soc_dai_ops matrixio_dai_ops = {
    .hw_params = matrixio_codec_hw_params,
    .prepare = matrixio_codec_prepare,
    .set_fmt = matrixio_codec_set_fmt,
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

static int matrixio_probe(struct snd_soc_codec *codec)
{

	printk(KERN_INFO "matrixio_probe");

	dev_set_drvdata(codec->dev, matrixio);

	snd_soc_codec_init_regmap(codec, matrixio->regmap);

	return 0;
}

static const struct snd_soc_codec_driver matrixio_soc_codec_driver = {

    .probe = matrixio_probe,
    .component_driver =
	{
	    .controls = matrixio_snd_controls,
	    .num_controls = ARRAY_SIZE(matrixio_snd_controls),
	    .dapm_widgets = matrixio_dapm_widgets,
	    .num_dapm_widgets = ARRAY_SIZE(matrixio_dapm_widgets),
	    .dapm_routes = matrixio_dapm_routes,
	    .num_dapm_routes = ARRAY_SIZE(matrixio_dapm_routes),

	},

    /*
    .set_bias_level =
    .read =
    .write =
*/
};


static struct snd_soc_dai_driver matrixio_dai_driver = {
    .name = "matrixio-dai",
    .capture =
	{
	    .stream_name = "Micarray Capture",
	    .channels_min = 1,
	    .channels_max = MATRIXIO_CHANNELS_MAX,
	    .rates = MATRIXIO_RATES,
	    .formats = MATRIXIO_FORMATS,
	},
    .ops = &matrixio_dai_ops,
};

static int matrixio_codec_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &matrixio_soc_card;

        matrixio = dev_get_drvdata(pdev->dev.parent);

	card->dev = &pdev->dev;

	printk(KERN_INFO "matrixio_codec_probe");

	ret = snd_soc_register_codec(matrixio->dev, &matrixio_soc_codec_driver,
				     &matrixio_dai_driver, 1);

	if (ret != 0) {
		dev_err(matrixio->dev,
			"Failed to register MATRIXIO codec: %d\n", ret);
	}

	printk(KERN_INFO "MATRIXIO codec registered\n");

        ret = devm_snd_soc_register_card(&pdev->dev, card);

        if (ret != 0) {
                dev_err(matrixio->dev, "Failed to register MATRIXIO card: %d\n",
                        ret);
        }

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
    .probe = matrixio_codec_probe,

    .remove = matrixio_codec_remove,
};

module_platform_driver(matrixio_codec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO audio module");
