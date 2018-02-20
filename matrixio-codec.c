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

#define MATRIXIO_RATES SNDRV_PCM_RATE_8000_48000
#define MATRIXIO_FORMATS SNDRV_PCM_FMTBIT_S16_LE

static struct snd_soc_dai_link matrixio_snd_soc_dai[] = {
    /*  {
	  .name = "matrixio.pcm.0",
	  .stream_name = "matrixio.pcm.0",
	  .codec_dai_name = "snd-soc-dummy-dai",
	  .cpu_dai_name = "matrixio-pcm-out.0",
	  .platform_name = "matrixio-pcm",
	  .codec_name = "snd-soc-dummy",
	  .ops = &matrixio_snd_ops,
      },*/
    {
	.name = "matrixio.mic.0",
	.stream_name = "matrixio.mic.0",
	.codec_dai_name = "snd-soc-dummy-dai",
	.cpu_dai_name = "matrixio-mic.0",
	.platform_name = "matrixio-pcm-capture",
	.codec_name = "snd-soc-dummy",
    }};

static struct snd_soc_card matrixio_soc_card = {
    .name = "MATRIXIO SOUND",
    .owner = THIS_MODULE,
    .dai_link = matrixio_snd_soc_dai,
    .num_links = ARRAY_SIZE(matrixio_snd_soc_dai),
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
	// snd_soc_codec_init_regmap(codec, ms->mio->regmap);
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

static struct snd_soc_dai_driver matrixio_dai_driver[] = {
    /* {
	.name = "matrixio-pcm-out.0",
	.playback =
	    {
		.stream_name = "matrixio-pcm-out.0",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MATRIXIO_RATES,
		.rate_min = 8000,
		.rate_max = 48000,
		.formats = MATRIXIO_FORMATS,
	    },
	.ops = &matrixio_dai_ops,
    },*/
    {
	.name = "matrixio-mic.0",
	.capture =
	    {
		.stream_name = "matrixio-mic.0",
		.channels_min = 1,
		.channels_max = 8,
		.rates = MATRIXIO_RATES,
		.rate_min = 8000,
		.rate_max = 48000,
		.formats = MATRIXIO_FORMATS,
	    },
    }};

static int matrixio_probe(struct platform_device *pdev)
{
	int ret;

	struct snd_soc_card *card = &matrixio_soc_card;

	ret = snd_soc_register_codec(&pdev->dev, &matrixio_soc_codec_driver,
				     matrixio_dai_driver,
				     ARRAY_SIZE(matrixio_dai_driver));
	if (ret) {
		dev_err(&pdev->dev, "Failed to register MATRIXIO codec: %d\n",
			ret);
		goto out;
	}

	card->dev = &pdev->dev;

	platform_set_drvdata(pdev, card);

//	snd_soc_card_set_drvdata(card, matrixio_stuff);

	ret = snd_soc_of_parse_card_name(card, "matrixio,model");
	if (ret)
		goto out;

//	ret = snd_soc_of_parse_audio_routing(card, "matrixio,audio-routing");
//	if (ret)
//		goto out;

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register MATRIXIO card (%d)\n",
			ret);
	}
out:
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
