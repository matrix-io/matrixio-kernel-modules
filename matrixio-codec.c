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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "matrixio-core.h"

#define MATRIXIO_CHANNELS_MAX  8
#define MATRIXIO_RATES         SNDRV_PCM_RATE_8000_48000
#define MATRIXIO_FORMATS       SNDRV_PCM_FMTBIT_S16_LE

static struct matrixio* matrixio;

static int matrixio_codec_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai) {

	return 0;
}

static const struct snd_soc_dai_ops matrixio_dai_ops = {
	.hw_params = matrixio_codec_hw_params,
};

static const struct snd_kcontrol_new matrixio_snd_controls[] = {
};

static const struct snd_soc_dapm_widget matrixio_dapm_widgets[] = {
};

static const struct snd_soc_dapm_route matrixio_dapm_routes[] = {
};

static int matrixio_add_widgets(struct snd_soc_codec *codec) {
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	snd_soc_add_codec_controls(codec, matrixio_snd_controls,
			ARRAY_SIZE(matrixio_snd_controls));

	snd_soc_dapm_new_controls(dapm, matrixio_dapm_widgets,
			ARRAY_SIZE(matrixio_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, matrixio_dapm_routes, 
			ARRAY_SIZE(matrixio_dapm_routes));

	return 0;
}


static int matrixio_probe(struct snd_soc_codec *codec) {

	dev_set_drvdata(codec->dev, matrixio);

	matrixio_add_widgets(codec);

	snd_soc_codec_init_regmap(codec, matrixio->regmap);

	return 0;
}

static const struct snd_soc_codec_driver matrixio_soc_codec_driver = {

	.probe = matrixio_probe,
/*
	.set_bias_level =
	.read =
	.write =
*/
};

static  struct snd_soc_dai_driver matrixio_dai_driver = {
        .name = "matrixio-codec",
        .capture = {
                .stream_name = "Capture",
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
	matrixio = dev_get_drvdata(pdev->dev.parent);

	ret = snd_soc_register_codec(matrixio->dev, 
			&matrixio_soc_codec_driver, 
			&matrixio_dai_driver, 1);

	if (ret != 0) {
		dev_err(matrixio->dev, "Failed to register MATRIXIO codec: %d\n", ret);
	}

	return ret;
}

static int  matrixio_codec_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver matrixio_codec_driver = {
	.driver = {
		.name = "matrixio-codec",
	},
	.probe = matrixio_codec_probe,
	.remove = matrixio_codec_remove,
};

module_platform_driver(matrixio_codec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO audio module");

