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
	/* This device is output only */
	return 0;
}

static int matrixio_gpio_direction_input(struct gpio_chip *chip,
					 unsigned offset)
{
	/* This device is output only */
	return -EINVAL;
}

static int matrixio_gpio_direction_output(struct gpio_chip *chip,
					  unsigned offset, int value)
{

	return 0;
}

static int matrixio_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static void matrixio_gpio_set(struct gpio_chip *chip, unsigned offset,
			      int value)
{
}

static const struct gpio_chip template_chip = {
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
	printk(KERN_INFO "gpio probe");

	printk(KERN_INFO ": %s", pdev->name);

	return 0;
}

static int matrixio_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

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
