/*
 * matrixio_imu.c - Support for Vishay VEML6070 UV A light sensor
 *
 * Copyright 2017 Andres Calderon <andres.calderon@admobilize.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for VEML6070 (MATRIX CREATOR UV Sensor)
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include "matrixio-core.h"

#define MATRIXIO_UV_DRV_NAME "matrixio_imu"

#define MATRIXIO_SRAM_OFFSET_IMU 0x30

struct matrixio_bus {
	struct matrixio *mio;
	struct mutex lock;
};


static const struct iio_chan_spec matrixio_imu_channels[] = {
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.address = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.address = 2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.address = 4,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_ANGL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.address = 6,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_ANGL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.address = 8,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_ANGL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.address = 10,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.address = 12,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.address = 14,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.address = 16,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static void matrixio_to_int_plus_micro (int data, int *val, int *val2)
{
	*val = data / 1000;
	*val2 = (data % 1000)*1000;
}

static int matrixio_imu_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan, int *val,
				int *val2, long mask)
{
	struct matrixio_bus *data = iio_priv(indio_dev);
	int ret;
	int data;

	if(mask == IIO_CHAN_INFO_RAW) {

		ret = matrixio_hw_buf_read(
	    		data->mio, MATRIXIO_MCU_BASE + (MATRIXIO_SRAM_OFFSET_IMU >> 1) + chan->address,
	    		sizeof(data), &data);

		matrixio_to_int_plus_micro (env_data.altitude, val, val2);
		
		return  IIO_VAL_INT_PLUS_MICRO;
	} 

	return -EINVAL;
}

static const struct iio_info matrixio_imu_info = {
	.read_raw = matrixio_imu_read_raw, 
	.driver_module = THIS_MODULE,
};

static int matrixio_imu_probe(struct platform_device *pdev)
{
	struct matrixio_bus *data;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);

	data->mio = dev_get_drvdata(pdev->dev.parent);

	platform_set_drvdata(pdev, indio_dev);

	mutex_init(&data->lock);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &matrixio_imu_info;
	indio_dev->channels = matrixio_imu_channels;
	indio_dev->num_channels = ARRAY_SIZE(matrixio_imu_channels);
	indio_dev->name = MATRIXIO_UV_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	data->mio = dev_get_drvdata(pdev->dev.parent);

	return iio_device_register(indio_dev);
}

static int matrixio_imu_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&pdev->dev);

	iio_device_unregister(indio_dev);

	return 0;
}

static struct platform_driver matrixio_imu_driver = {
    .driver =
	{
	    .name = "matrixio-imu",
	},
    .probe = matrixio_imu_probe,
    .remove = matrixio_imu_remove,
};

module_platform_driver(matrixio_imu_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO IIO ENV module");