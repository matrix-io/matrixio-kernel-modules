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
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/mfd/core.h>

#include "matrixio-core.h"

static DEFINE_MUTEX(matrixio_spi_lock);

struct hardware_address {
	uint8_t readnwrite : 1;
	uint8_t burst : 1;
	uint16_t reg : 14;
};

static struct regmap_config matrixio_regmap_config = {
	.reg_bits  = 16,
	.val_bits  = 16,
	.reg_read  = matrixio_hw_reg_read,
	.reg_write = matrixio_hw_reg_write,
};

static int matrixio_spi_transfer(struct spi_device *spi, uint8_t *send_buffer,
				 uint8_t *receive_buffer, unsigned int size)
{
	int ret;
	struct spi_transfer transfer;
	struct spi_message msg;
	
	mutex_lock(&matrixio_spi_lock);

	memset(&transfer, 0, sizeof(transfer));

	spi_message_init(&msg);

	transfer.tx_buf = send_buffer;
	transfer.rx_buf = receive_buffer;
	transfer.len = size;

	spi_message_add_tail(&transfer, &msg);

	ret = spi_sync(spi, &msg);

	mutex_unlock(&matrixio_spi_lock);

	return ret;
}

int matrixio_hw_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	int ret;

	struct matrixio *matrixio = context;
	struct hardware_address *hw_addr;
	uint8_t send_buf[4];
	uint16_t recv_buf[2];

	hw_addr = (struct hardware_address *)send_buf;
	hw_addr->reg = reg;
	hw_addr->burst = 0;
	hw_addr->readnwrite = 1;

	ret = matrixio_spi_transfer(matrixio->spi, send_buf, (uint8_t *)recv_buf, 4);

	if (ret < 0) {
		*val = 0;
		return ret;
	}

	*val = recv_buf[1];
	return 0;
}

int matrixio_hw_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct matrixio *matrixio = context;

	struct hardware_address *hw_addr;
	uint16_t send_buf[2];
	uint8_t recv_buf[4];

	hw_addr = (struct hardware_address *)send_buf;
	hw_addr->reg = reg;
	hw_addr->burst = 0;
	hw_addr->readnwrite = 0;

	send_buf[1] = val;

	return matrixio_spi_transfer(matrixio->spi, (uint8_t *)send_buf, (uint8_t *)recv_buf, 4);
}

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
		 .name = "matrixio-uv",
		 .of_compatible = "matrixio-uv",
		 .platform_data = matrixio,
		 .pdata_size = sizeof(*matrixio),
		}
	};

	return devm_mfd_add_devices(matrixio->dev, -1, cells, ARRAY_SIZE(cells), NULL, 0, NULL);
}


static int matrixio_init(struct matrixio *matrixio,
			 struct matrixio_platform_data *pdata)
{
	int ret;

	dev_set_drvdata(matrixio->dev, matrixio);

	/* TODO: Check that this is actually a MATRIX FPGA */

	ret = matrixio_register_devices(matrixio);

	if (ret != 0) {
		dev_err(matrixio->dev, "Failed to register MATRIX FPGA \n");
		return ret;
	}

	if(pdata && pdata->platform_init) {
		ret = pdata->platform_init(matrixio->dev);
		if(ret != 0) {
			dev_err(matrixio->dev, "Platform init failed: %d\n",
						ret);
			return ret;
		}
	} else
		dev_warn(matrixio->dev, "No platform initialization provided\n");

	return 0;
}

static int matrixio_core_probe(struct spi_device *spi)
{
	int ret;
	struct matrixio *matrixio;
        
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	

	if(ret<0)
		return ret;

	matrixio = devm_kzalloc(&spi->dev, sizeof(struct matrixio), GFP_KERNEL);
	
	if(matrixio == NULL)
		return -ENOMEM;

	matrixio->dev = &spi->dev;
	matrixio->spi = spi;
	matrixio->stamp = 0x1221;
	
	matrixio->regmap = devm_regmap_init(&spi->dev, NULL, matrixio, &matrixio_regmap_config);

	if(IS_ERR(matrixio->regmap)) {
		ret = PTR_ERR(matrixio->regmap);
		dev_err(matrixio->dev, "Failed to allocate register map: %d\n",
					ret);
		return ret;
	}

	return matrixio_init(matrixio, dev_get_platdata(&spi->dev));
}

static const struct of_device_id matrixio_core_dt_ids[] = {
	{ 
		.compatible = "matrixio-core", 
		.data = (void *) 0 
	}, 
	{}
};

MODULE_DEVICE_TABLE(of, matrixio_core_dt_ids);

static struct spi_driver matrixio_core_driver = {
    .driver = {
	    .name           = "matrixio-core",
	    .of_match_table = of_match_ptr(matrixio_core_dt_ids),
    },
    .probe = matrixio_core_probe
};

module_spi_driver(matrixio_core_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO core module");
