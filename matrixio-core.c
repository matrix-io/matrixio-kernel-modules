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

struct hardware_cmd {
	uint8_t readnwrite : 1;
	uint8_t burst : 1;
	uint16_t reg : 14;
	uint16_t value;
};

static struct regmap_config matrixio_regmap_config = {
    .reg_bits = 16,
    .val_bits = 16,
    .reg_read = matrixio_hw_reg_read,
    .reg_write = matrixio_hw_reg_write,
};

static ssize_t matrixio_spi_sync(struct matrixio *matrixio,
				 struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;
	struct spi_device *spi;

	spin_lock_irq(&matrixio->spi_lock);
	spi = matrixio->spi;
	spin_unlock_irq(&matrixio->spi_lock);

	status = spi_sync(spi, message);

	if (status == 0)
		status = message->actual_length;

	return status;
}

static int matrixio_spi_transfer(struct matrixio *matrixio, 
				 unsigned int size)
{
	struct spi_transfer t = {.rx_buf = matrixio->rx_buffer,
				 .tx_buf = matrixio->tx_buffer,
				 .len = size,
				 .speed_hz = matrixio->speed_hz};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return matrixio_spi_sync(matrixio, &m);
}

int matrixio_hw_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	int ret;
	struct matrixio *matrixio = context;
	uint16_t *recv_buf;
	struct hardware_cmd *hw_addr;

	mutex_lock(&matrixio->reg_lock);

	recv_buf = (uint16_t *)matrixio->rx_buffer;
	hw_addr = (struct hardware_cmd *)matrixio->tx_buffer;

	hw_addr->reg = reg;
	hw_addr->burst = 0;
	hw_addr->readnwrite = 1;
	hw_addr->value = 0;

	ret = matrixio_spi_transfer(matrixio, 4);

	if (!ret)
		*val = recv_buf[1];

	mutex_unlock(&matrixio->reg_lock);

	return ret;
}

int matrixio_hw_reg_write(void *context, unsigned int reg, unsigned int val)
{
	int ret;
	struct matrixio *matrixio = context;
	struct hardware_cmd *hw_cmd;

	mutex_lock(&matrixio->reg_lock);

	hw_cmd = (struct hardware_cmd *)matrixio->tx_buffer;

	hw_cmd->reg = reg;
	hw_cmd->burst = 0;
	hw_cmd->readnwrite = 0;
	hw_cmd->value = val;

	ret = matrixio_spi_transfer(matrixio, 4);

	mutex_unlock(&matrixio->reg_lock);

	return ret;
}

int matrixio_hw_buf_read(struct matrixio *matrixio, unsigned int add,
			 int length, void *data)
{
	int ret;
	int offset;
	unsigned int val;
	uint16_t *words;

	mutex_lock(&matrixio->buf_lock);

	words = (uint16_t *)data;
	for (offset = 0; offset < (length / 2); offset++) {
		ret = matrixio_hw_reg_read(matrixio, add + offset, &val);

		if (ret)
			break;

		words[offset] = val;
	}

	mutex_unlock(&matrixio->buf_lock);

	return ret;
}
EXPORT_SYMBOL(matrixio_hw_buf_read);

int matrixio_hw_read_enqueue(struct matrixio *matrixio, unsigned int add,
			     int length, struct kfifo_rec_ptr_2 *fifo)
{
	int ret;
	struct hardware_cmd *hw_addr;

	mutex_lock(&matrixio->buf_lock);

	hw_addr = (struct hardware_cmd *)matrixio->tx_buffer;
	hw_addr->reg = add;
	hw_addr->burst = 1;
	hw_addr->readnwrite = 1;

	ret = matrixio_spi_transfer(matrixio,
				    length + 2);

	if (ret >= 0)
		kfifo_in(fifo, &matrixio->rx_buffer[2], length);

	mutex_unlock(&matrixio->buf_lock);

	return ret;
}
EXPORT_SYMBOL(matrixio_hw_read_enqueue);

int matrixio_hw_buf_write(struct matrixio *matrixio, unsigned int add,
			  int length, void *data)
{
	int ret;
	int offset;
	uint16_t *words;

	mutex_lock(&matrixio->buf_lock);

	words = (uint16_t *)data;

	for (offset = 0; offset < (length / 2); offset++) {
		ret = matrixio_hw_reg_write(matrixio, add + offset,
					    words[offset]);

		if (ret)
			break;
	}

	mutex_unlock(&matrixio->buf_lock);

	return ret;
}

EXPORT_SYMBOL(matrixio_hw_buf_write);

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
	    }

	};

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

	spin_lock_init(&matrixio->spi_lock);

	mutex_init(&matrixio->reg_lock);

	mutex_init(&matrixio->buf_lock);

	matrixio->speed_hz = spi->max_speed_hz;

	matrixio->rx_buffer = devm_kzalloc(&spi->dev, 4096, GFP_KERNEL);

	matrixio->tx_buffer = devm_kzalloc(&spi->dev, 4096, GFP_KERNEL);

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
