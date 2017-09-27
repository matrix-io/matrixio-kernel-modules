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

#include "matrixio-core.h"

static struct matrixio* matrixio;

static int matrixio_codec_probe(struct platform_device *pdev)
{
	matrixio = dev_get_drvdata(pdev->dev.parent);

	return 0;
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

