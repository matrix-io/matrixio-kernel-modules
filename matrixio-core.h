/*
 * matrix-core.h -- MATRIX core functions to talk with the FPGA internal
 *                  bus
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

#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#define MATRIXIO_CONF_BASE 0x0000
#define MATRIXIO_UART_BASE 0x1000
#define MATRIXIO_MICARRAY_BASE 0x2000
#define MATRIXIO_EVERLOOP_BASE 0x3000
#define MATRIXIO_GPIO_BASE 0x4000
#define MATRIXIO_MCU_BASE 0x5000

struct matrixio {
	struct device *dev;
	struct regmap *regmap;
	struct mutex reg_lock;
	spinlock_t spi_lock;
	struct spi_device *spi;
	u8 *tx_buffer;
	u8 *rx_buffer;
	u32 speed_hz;
};

struct matrixio_platform_data {
	int (*platform_init)(struct device *dev);
};

int matrixio_reg_read(void *context, unsigned int reg, unsigned int *val);

int matrixio_reg_write(void *context, unsigned int reg, unsigned int val);

int matrixio_read(struct matrixio *matrixio, unsigned int add, int length,
		     void *data);

int matrixio_write(struct matrixio *matrixio, unsigned int add, int length,
		      void *data);

#endif
