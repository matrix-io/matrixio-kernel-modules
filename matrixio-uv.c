/*
 * matrixio_uv.c - Support for Vishay VEML6070 UV A light sensor
 *
 * Copyright 2016 Peter Meerwald-Stadler <pmeerw@pmeerw.net>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for VEML6070 (7-bit I2C slave addresses 0x38 and 0x39)
 *
 * TODO: integration time, ACK signal
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "matrixio-core.h"

#define VEML6070_DRV_NAME "matrixio_uv"

#define VEML6070_FPGA_BASE 0x30    /*VEML6070 Base addr */

struct matrixio_uv_data {
	struct matrixio *mio;
	struct mutex lock;
};

static const struct iio_chan_spec matrixio_uv_channels[] = {
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_UV,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_UVINDEX,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	}
};

static int matrixio_uv_to_uv_index(unsigned val)
{
	/*
	 * conversion of raw UV intensity values to UV index depends on
	 * integration time (IT) and value of the resistor connected to
	 * the RSET pin (default: 270 KOhm)
	 */
	unsigned uvi[11] = {
		187, 373, 560, /* low */
		746, 933, 1120, /* moderate */
		1308, 1494, /* high */
		1681, 1868, 2054}; /* very high */
	int i;

	for (i = 0; i < ARRAY_SIZE(uvi); i++)
		if (val <= uvi[i])
			return i;

	return 11; /* extreme */
}

static int matrixio_uv_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct matrixio_uv_data *data = iio_priv(indio_dev);
	int ret;

	printk(KERN_INFO "por acá pasó %d... ", data->mio->stamp);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		/*
		ret = matrixio_uv_read(data);
		if (ret < 0)
			return ret;
		if (mask == IIO_CHAN_INFO_PROCESSED)
			*val = matrixio_uv_to_uv_index(ret);
		else
			*val = ret;
			*/
		regmap_read(data->mio->regmap, MATRIXIO_MCU_BASE + (VEML6070_FPGA_BASE>>1), val);

		//*val = 100;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info matrixio_uv_info = {
	.read_raw = matrixio_uv_read_raw,
	.driver_module = THIS_MODULE,
};

static int matrixio_uv_probe(struct platform_device *pdev)
{
	struct matrixio_uv_data *data;
	struct iio_dev *indio_dev;

	printk(KERN_INFO "el probe %s", pdev->name);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);

	data->mio = dev_get_drvdata(pdev->dev.parent);

	platform_set_drvdata(pdev, indio_dev);

	mutex_init(&data->lock);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &matrixio_uv_info;
	indio_dev->channels = matrixio_uv_channels;
	indio_dev->num_channels = ARRAY_SIZE(matrixio_uv_channels);
	indio_dev->name = VEML6070_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	data->mio = dev_get_drvdata(pdev->dev.parent);
	       
	return iio_device_register(indio_dev);
}

static int matrixio_uv_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&pdev->dev);

	iio_device_unregister(indio_dev);

	return 0;
}

static struct platform_driver matrixio_gpio_driver = {
	    .driver = {
            	.name = "matrixio-uv",
	     },
	    .probe = matrixio_uv_probe,
	    .remove = matrixio_uv_remove,
};

module_platform_driver(matrixio_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO UV module");

