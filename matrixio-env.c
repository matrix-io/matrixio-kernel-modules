/*
 * matrixio_env.c - Support for Vishay VEML6070 UV A light sensor
 *
 * Copyright 2017 Andres Calderon <andres.calderon@admobilize.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for (MATRIX CREATOR Enviromental Sensors)
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include "matrixio-core.h"

#define MATRIXIO_UV_DRV_NAME "matrixio_env"

#define MATRIXIO_SRAM_OFFSET_ENV 0x0

struct matrixio_bus {
	struct matrixio *mio;
	struct mutex lock;
};

struct matrixio_env_data {
	int UV;
	int altitude;
	int pressure;
	int temperature_mpl;
	int humidity;
	int temperature_hts;
};

static const struct iio_chan_spec matrixio_env_channels[] = {
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_UV,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_UVINDEX, 
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{	
		.type = IIO_TEMP,
		.modified = 1,
		.channel2 = IIO_MOD_TEMP_OBJECT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_PRESSURE, 
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
		{
		.type = IIO_HUMIDITYRELATIVE, 
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_DISTANCE, 
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static int matrixio_env_to_uv_index(unsigned val)
{
	/*
	 * conversion of raw UV intensity values to UV index depends on
	 * integration time (IT) and value of the resistor connected to
	 * the RSET pin (Table for RSET:270 KOhm IT: 1
	 */
	unsigned uvi[11] = {187,  373,  560,   /* low */
			    746,  933,  1120,  /* moderate */
			    1308, 1494,	/* high */
			    1681, 1868, 2054}; /* very high */
	int i;

	for (i = 0; i < ARRAY_SIZE(uvi); i++)
		if (val <= uvi[i] * 4) /* 4T */
			return i;

	return 11; /* extreme */
}

static void matrixio_to_int_plus_micro (int data, int *val, int *val2)
{
	*val = data / 1000;
	*val2 = (data % 1000)*1000;
}

static int matrixio_env_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan, int *val,
				int *val2, long mask)
{
	struct matrixio_bus *data = iio_priv(indio_dev);
	int ret;
	struct matrixio_env_data env_data;
	
	mutex_lock(&indio_dev->mlock);
	ret = matrixio_hw_buf_read(
	    data->mio, MATRIXIO_MCU_BASE + (MATRIXIO_SRAM_OFFSET_ENV >> 1),
	    sizeof(env_data), &env_data);
	mutex_unlock(&indio_dev->mlock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch(chan->type){
			case IIO_INTENSITY:
				*val = env_data.UV;
				return IIO_VAL_INT;
			case IIO_TEMP:
				matrixio_to_int_plus_micro (env_data.temperature_hts, val, val2);
				return IIO_VAL_INT_PLUS_MICRO;
			case IIO_PRESSURE:
				matrixio_to_int_plus_micro (env_data.pressure, val, val2);
				return IIO_VAL_INT_PLUS_MICRO;
			case IIO_HUMIDITYRELATIVE:
				matrixio_to_int_plus_micro (env_data.humidity, val, val2);
				return IIO_VAL_INT_PLUS_MICRO;
			case IIO_DISTANCE:
				matrixio_to_int_plus_micro (env_data.altitude, val, val2);
				return  IIO_VAL_INT_PLUS_MICRO;
			default:
				return -EINVAL;
		}
	case IIO_CHAN_INFO_PROCESSED:
		*val = matrixio_env_to_uv_index(env_data.UV);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info matrixio_env_info = {
	.read_raw = matrixio_env_read_raw, 
	.driver_module = THIS_MODULE,
};

static int matrixio_env_probe(struct platform_device *pdev)
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
	indio_dev->info = &matrixio_env_info;
	indio_dev->channels = matrixio_env_channels;
	indio_dev->num_channels = ARRAY_SIZE(matrixio_env_channels);
	indio_dev->name = MATRIXIO_UV_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	data->mio = dev_get_drvdata(pdev->dev.parent);

	return iio_device_register(indio_dev);
}

static int matrixio_env_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&pdev->dev);

	iio_device_unregister(indio_dev);

	return 0;
}

static struct platform_driver matrixio_env_driver = {
    .driver =
	{
	    .name = "matrixio-env",
	},
    .probe = matrixio_env_probe,
    .remove = matrixio_env_remove,
};

module_platform_driver(matrixio_env_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO IIO ENV module");
