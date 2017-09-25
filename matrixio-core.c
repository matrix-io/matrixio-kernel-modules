#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/mfd/core.h>
#include "matrixio-core.h"

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

enum matrixio_dev_idx {
	MATRIXIO_EVERLOOP = 0,
};

static struct mfd_cell matrixio_devs[] = {
	[MATRIXIO_EVERLOOP] = {
		.name = "matrixio-everloop",
		.of_compatible = "matrixio-everloop",
	},	
};

static int matrixio_spi_transfer(struct spi_device *spi, uint8_t *send_buffer,
				 uint8_t *receive_buffer, unsigned int size)
{
	struct spi_transfer transfer;
	struct spi_message msg;

	memset(&transfer, 0, sizeof(transfer));

	spi_message_init(&msg);

	transfer.tx_buf = send_buffer;
	transfer.rx_buf = receive_buffer;
	transfer.len = size;

	spi_message_add_tail(&transfer, &msg);

	return spi_sync(spi, &msg);
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
	uint8_t send_buf[4];
	uint8_t recv_buf[4];

	hw_addr = (struct hardware_address *)send_buf;
	hw_addr->reg = reg;
	hw_addr->burst = 0;
	hw_addr->readnwrite = 0;

	return matrixio_spi_transfer(matrixio->spi, send_buf, (uint8_t *)recv_buf, 4);
}

static int  matrixio_core_probe(struct spi_device *spi)
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

	spi_set_drvdata(spi, matrixio);

	matrixio->dev = &spi->dev;
	matrixio->spi = spi;

	matrixio->regmap = devm_regmap_init(&spi->dev, NULL, matrixio, &matrixio_regmap_config);

	if(IS_ERR(matrixio->regmap)) {
		ret = PTR_ERR(matrixio->regmap);
		dev_err(matrixio->dev, "Failed to allocate register map: %d\n",
					ret);
		return ret;
	}

	ret = mfd_add_devices(&spi->dev, -1, matrixio_devs,
			ARRAY_SIZE(matrixio_devs), NULL,
			0, NULL);
	if(ret){
		dev_err(matrixio->dev, "Failed to add child devices: %d\n",
					ret);
	}

	return ret;
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
