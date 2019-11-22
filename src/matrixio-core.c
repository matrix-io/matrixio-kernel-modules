/*
 * matrix-core.c -- MATRIX core functions to talk with the FPGA internal
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "matrixio-core.h"


/* We need to send the address before reading/writing the data after it.  This
 * can be done with two transfers in one message: one for the address followed
 * by one for the data.  Or with one transfer: with the address and data copied
 * in/out of a single buffer.  One transfer is faster for small messages, while
 * two transfers are faster for large messages.  The two transfer method also
 * avoids needing to impose a max size limit on the data as it goes directly
 * to/from a user supplied buffer.
 *
 * This value is the threshold where the driver switches from one xfer to the
 * two xfer method.  It also provides the largest bounce buffer size needed.
 */
#define MATRIXIO_SPI_BOUNCE_SIZE 2048

struct hardware_cmd {
	uint8_t readnwrite : 1;
	uint16_t reg : 15;
};

/* For a large read, does not use rx_buffer to bounce the data */
static int matrixio_large_read(struct matrixio *matrixio, unsigned int add, int length,
		               void *data)
{
	/* Don't use stack hw_cmd as it must be dma-safe */
	struct hardware_cmd* hw_cmd = (struct hardware_cmd*)matrixio->tx_buffer;
	struct spi_transfer t[] = {
		{
			.tx_buf = hw_cmd,
			.len = sizeof(*hw_cmd),
		},
		{
			.rx_buf = data,
			.len = length,
		}
	};
	struct spi_message m;

	hw_cmd->reg = add;
	hw_cmd->readnwrite = 1;
	spi_message_init_with_transfers(&m, t, ARRAY_SIZE(t));

	return spi_sync(matrixio->spi, &m);
}

/* For small reads, bounces the data through tx/rx buffer */
static int matrixio_small_read(struct matrixio *matrixio, unsigned int add, int length,
		               void *data)
{
	struct hardware_cmd* hw_cmd = (struct hardware_cmd*)matrixio->tx_buffer;
	struct spi_transfer t[] = {
		{
			.tx_buf = matrixio->tx_buffer,
			.rx_buf = matrixio->rx_buffer,
			.len = length + sizeof(*hw_cmd),
		}
	};
	struct spi_message m;
	int ret;

	hw_cmd->reg = add;
	hw_cmd->readnwrite = 1;
	memset(matrixio->tx_buffer + sizeof(*hw_cmd), 0, length);

	spi_message_init_with_transfers(&m, t, ARRAY_SIZE(t));
	ret = spi_sync(matrixio->spi, &m);

	memcpy(data, matrixio->rx_buffer + sizeof(*hw_cmd), length);

	return ret;
}

int matrixio_read(struct matrixio *matrixio, unsigned int add, int length,
		  void *data)
{
	int ret;

	mutex_lock(&matrixio->reg_lock);
	if (length > MATRIXIO_SPI_BOUNCE_SIZE - sizeof(struct hardware_cmd)) {
		ret = matrixio_large_read(matrixio, add, length, data);
	} else {
		ret = matrixio_small_read(matrixio, add, length, data);
	}
	mutex_unlock(&matrixio->reg_lock);

	return ret;
}
EXPORT_SYMBOL(matrixio_read);

int matrixio_write(struct matrixio *matrixio, unsigned int add, int length,
		   void *data)
{
	int ret;
	struct hardware_cmd *hw_cmd = (struct hardware_cmd*)matrixio->tx_buffer;
	struct spi_transfer xfers[2] = {
		{
			.tx_buf = matrixio->tx_buffer,
		},
	};
	struct spi_message m;

	mutex_lock(&matrixio->reg_lock);

	hw_cmd->reg = add;
	hw_cmd->readnwrite = 0;
	if (length > MATRIXIO_SPI_BOUNCE_SIZE - sizeof(*hw_cmd)) {
		xfers[0].len = sizeof(*hw_cmd);
		xfers[1].tx_buf = data;
		xfers[1].len = length;
		spi_message_init_with_transfers(&m, xfers, 2);
	} else {
		xfers[0].len = length + sizeof(*hw_cmd);
		memcpy(matrixio->tx_buffer + sizeof(*hw_cmd), data, length);
		spi_message_init_with_transfers(&m, xfers, 1);
	}

	ret = spi_sync(matrixio->spi, &m);
	mutex_unlock(&matrixio->reg_lock);

	return ret;
}
EXPORT_SYMBOL(matrixio_write);

static int matrixio_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	return matrixio_read((struct matrixio *)(context), reg, sizeof(int16_t),
			     &val);
}

int matrixio_reg_write(void *context, unsigned int reg, unsigned int val)
{
	int16_t data = val;
	return matrixio_write((struct matrixio *)(context), reg,
			      sizeof(int16_t), &data);
}
EXPORT_SYMBOL(matrixio_reg_write);

static int matrixio_register_devices(struct matrixio *matrixio)
{
	const struct mfd_cell cells[] = {
	    {
		.name = "matrixio-everloop",
		.of_compatible = "matrixio-everloop",
		.platform_data = matrixio,
		.pdata_size = sizeof(*matrixio),
	    },
	    {
		.name = "matrixio-mic",
		.of_compatible = "matrixio-mic",
		.platform_data = matrixio,
		.pdata_size = sizeof(*matrixio),
	    },
	    {
		.name = "matrixio-playback",
		.of_compatible = "matrixio-playback",
		.platform_data = matrixio,
		.pdata_size = sizeof(*matrixio),
	    },
	    {
		.name = "matrixio-codec",
		.of_compatible = "matrixio-codec",
		.platform_data = matrixio,
		.pdata_size = sizeof(*matrixio),
	    },
	    {
		.name = "matrixio-uart",
		.of_compatible = "matrixio-uart",
		.platform_data = matrixio,
		.pdata_size = sizeof(*matrixio),
	    },
	    {
		.name = "matrixio-gpio",
		.of_compatible = "matrixio-gpio",
		.platform_data = matrixio,
		.pdata_size = sizeof(*matrixio),
	    },
	    {
		.name = "matrixio-env",
		.of_compatible = "matrixio-env",
		.platform_data = matrixio,
		.pdata_size = sizeof(*matrixio),
	    },
	    {
		.name = "matrixio-imu",
		.of_compatible = "matrixio-imu",
		.platform_data = matrixio,
		.pdata_size = sizeof(*matrixio),
	    },
	    {
		.name = "matrixio-regmap",
		.of_compatible = "matrixio-regmap",
		.platform_data = matrixio,
		.pdata_size = sizeof(*matrixio),
	    }};

	return devm_mfd_add_devices(matrixio->dev, -1, cells, ARRAY_SIZE(cells),
				    NULL, 0, NULL);
}

static int matrixio_init(struct matrixio *matrixio,
			 struct matrixio_platform_data *pdata)
{
	int ret;

	dev_set_drvdata(matrixio->dev, matrixio);

	/* TODO: Check that this is actually a MATRIX FPGA */
	ret = matrixio_register_devices(matrixio);

	if (ret) {
		dev_err(matrixio->dev, "Failed to register MATRIX FPGA \n");
		return ret;
	}

	return 0;
}

static const struct regmap_config matrixio_regmap_config = {
    .reg_bits = 16,
    .val_bits = 16,
    .reg_read = matrixio_reg_read,
    .reg_write = matrixio_reg_write,
};

static int matrixio_core_probe(struct spi_device *spi)
{
	int ret;
	struct matrixio *matrixio;

	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);

	if (ret)
		return ret;

	matrixio = devm_kzalloc(&spi->dev, sizeof(struct matrixio), GFP_KERNEL);

	if (matrixio == NULL)
		return -ENOMEM;

	matrixio->dev = &spi->dev;

	matrixio->spi = spi;

	mutex_init(&matrixio->reg_lock);

	matrixio->rx_buffer = devm_kzalloc(&spi->dev, MATRIXIO_SPI_BOUNCE_SIZE, GFP_KERNEL);
	if (matrixio->rx_buffer == NULL)
		return -ENOMEM;

	matrixio->tx_buffer = devm_kzalloc(&spi->dev, MATRIXIO_SPI_BOUNCE_SIZE, GFP_KERNEL);
	if (matrixio->tx_buffer == NULL)
		return -ENOMEM;

	spi_set_drvdata(spi, matrixio);

	matrixio->regmap = devm_regmap_init(&spi->dev, NULL, matrixio,
					    &matrixio_regmap_config);

	if (IS_ERR(matrixio->regmap)) {
		ret = PTR_ERR(matrixio->regmap);
		dev_err(matrixio->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	return matrixio_init(matrixio, dev_get_platdata(&spi->dev));
}

static const struct of_device_id matrixio_core_dt_ids[] = {
    {.compatible = "matrixio-core", .data = (void *)0}, {}};

MODULE_DEVICE_TABLE(of, matrixio_core_dt_ids);

static struct spi_driver matrixio_core_driver = {
    .driver =
	{
	    .name = "matrixio-core",
	    .of_match_table = of_match_ptr(matrixio_core_dt_ids),
	},
    .probe = matrixio_core_probe};

module_spi_driver(matrixio_core_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO core module");
