#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "matrixio-core.h"

struct matrixio_gpio {
	struct gpio_chip chip;
	struct matrixio *mio;
};

static int matrixio_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	printk(KERN_INFO "get direction");
	/* This device is output only */
	return 0;
}

static int matrixio_gpio_direction_input(struct gpio_chip *chip,
					 unsigned offset)
{
	printk(KERN_INFO "direction input");
	/* This device is output only */
	return -EINVAL;
}

static int matrixio_gpio_direction_output(struct gpio_chip *chip,
					  unsigned offset, int value)
{
	printk(KERN_INFO "direction output");
	return 0;
}

static int matrixio_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	printk(KERN_INFO "get");
	return 0;
}

static void matrixio_gpio_set(struct gpio_chip *chip, unsigned offset,
			      int value)
{
	printk(KERN_INFO "set");
}

static const struct gpio_chip matrixio_gpio_chip = {
    .label = "matrixio-gpio",
    .owner = THIS_MODULE,
    .get_direction = matrixio_gpio_get_direction,
    .direction_input = matrixio_gpio_direction_input,
    .direction_output = matrixio_gpio_direction_output,
    .get = matrixio_gpio_get,
    .set = matrixio_gpio_set,
    .base = -1,
    .ngpio = 16,
    .can_sleep = true,
};

static int matrixio_gpio_probe(struct platform_device *pdev)
{
	struct matrixio_gpio *gpio;
	int ret;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	platform_set_drvdata(pdev, gpio);

	printk(KERN_INFO "probe : %s", pdev->name);

	gpio->mio = dev_get_drvdata(pdev->dev.parent);
	gpio->chip = matrixio_gpio_chip;
	gpio->chip.parent = gpio->mio->dev;

	ret = gpiochip_add_data(&gpio->chip, gpio);

	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		return ret;
	}

	return 0;
}

static int matrixio_gpio_remove(struct platform_device *pdev) { return 0; }

static struct platform_driver matrixio_gpio_driver = {
    .driver =
	{
	    .name = "matrixio-gpio",
	},
    .probe = matrixio_gpio_probe,
    .remove = matrixio_gpio_remove,
};

module_platform_driver(matrixio_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO GPIO module");
