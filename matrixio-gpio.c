#include <asm/uaccess.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "matrixio-core.h"

#define MATRIXIO_GPIO_BASE 0x2800

struct matrixio_gpio {
	struct gpio_chip chip;
	struct matrixio *mio;
	struct mutex lock;
};

static int matrixio_gpio_get_direction(struct gpio_chip *gc, unsigned offset)
{
	struct matrixio_gpio *chip = gpiochip_get_data(gc);
	int gpio_direction;

	mutex_lock(&chip->lock);
	regmap_read(chip->mio->regmap, MATRIXIO_GPIO_BASE, &gpio_direction);
	mutex_unlock(&chip->lock);

	return (gpio_direction & BIT(offset));
}

static int matrixio_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct matrixio_gpio *chip = gpiochip_get_data(gc);
	int gpio_direction;

	mutex_lock(&chip->lock);
	regmap_read(chip->mio->regmap, MATRIXIO_GPIO_BASE, &gpio_direction);
	gpio_direction |= BIT(offset);
	regmap_write(chip->mio->regmap, MATRIXIO_GPIO_BASE, gpio_direction);
	mutex_unlock(&chip->lock);

	return 0;
}

static int matrixio_gpio_direction_output(struct gpio_chip *gc, unsigned offset,
					  int value)
{
	struct matrixio_gpio *chip = gpiochip_get_data(gc);
	int gpio_direction;
	int gpio_value;

	mutex_lock(&chip->lock);
	regmap_read(chip->mio->regmap, MATRIXIO_GPIO_BASE, &gpio_direction);
	regmap_read(chip->mio->regmap, MATRIXIO_GPIO_BASE + 1, &gpio_value);

	if (value)
		gpio_value |= BIT(offset);
	else
		gpio_value &= ~BIT(offset);

	regmap_write(chip->mio->regmap, MATRIXIO_GPIO_BASE, gpio_direction);
	regmap_write(chip->mio->regmap, MATRIXIO_GPIO_BASE + 1, gpio_value);
	mutex_unlock(&chip->lock);

	return 0;
}

static int matrixio_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct matrixio_gpio *chip = gpiochip_get_data(gc);
	int gpio_value;

	mutex_lock(&chip->lock);
	regmap_read(chip->mio->regmap, MATRIXIO_GPIO_BASE + 1, &gpio_value);
	mutex_unlock(&chip->lock);

	return (gpio_value & BIT(offset));
}

static void matrixio_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct matrixio_gpio *chip = gpiochip_get_data(gc);
	int gpio_value;

	mutex_lock(&chip->lock);
	regmap_read(chip->mio->regmap, MATRIXIO_GPIO_BASE + 1, &gpio_value);

	if (value)
		gpio_value |= BIT(offset);
	else
		gpio_value &= ~BIT(offset);

	regmap_write(chip->mio->regmap, MATRIXIO_GPIO_BASE + 1, gpio_value);
	mutex_unlock(&chip->lock);
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

	gpio->mio = dev_get_drvdata(pdev->dev.parent);
	gpio->chip = matrixio_gpio_chip;
	gpio->chip.parent = gpio->mio->dev;

	mutex_init(&gpio->lock);

	ret = gpiochip_add_data(&gpio->chip, gpio);

	if (ret) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		return ret;
	}

	return 0;
}

static int matrixio_gpio_remove(struct platform_device *pdev)
{

	struct matrixio_gpio *gpio;
	gpio = dev_get_drvdata(&pdev->dev);
	mutex_destroy(&gpio->lock);
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
