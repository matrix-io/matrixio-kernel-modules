/*
 * matrix-core.h -- MATRIX core functions to talk with the FPGA internal bus
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

#ifndef __MATRIXIO_CORE_H__
#define __MATRIXIO_CORE_H__

#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/mutex.h>

#define MATRIXIO_UART_BASE 0x0800
#define MATRIXIO_MICROPHONE_ARRAY_BASE 0x1800
#define MATRIXIO_EVERLOOP_BASE 0x2000

struct matrixio {
	struct device *dev;
	struct spi_device *spi;
	struct regmap *regmap;
	struct mutex lock;
};


int matrixio_hw_reg_read(void *context, unsigned int reg, unsigned int *val);
int matrixio_hw_reg_write(void *context, unsigned int reg, unsigned int val);

#endif
