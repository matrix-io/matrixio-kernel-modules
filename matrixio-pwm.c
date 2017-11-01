#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#include "matrixio-core.h"

#define NUM_PWM 16

struct matrixio_pwm_chip {
	struct matrixio *mio;
	struct pwm_chip chip;
};

static inline struct matrixio_pwm_chip *to_matrixio(struct pwm_chip *chip)
{
	return container_of(chip, struct matrixio_pwm_chip, chip);
}

static int matrixio_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	return 0;
}

static void matrixio_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm) {}

static int matrixio_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	return 0;
}

static void matrixio_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
}

static int matrixio_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			       int duty_ns, int period_ns)
{
	return 0;
}

static const struct pwm_ops matrixio_pwm_ops = {
    .request = matrixio_pwm_request,
    .free = matrixio_pwm_free,
    .config = matrixio_pwm_config,
    .enable = matrixio_pwm_enable,
    .disable = matrixio_pwm_disable,
    .owner = THIS_MODULE,
};

static int matrixio_pwm_probe(struct platform_device *pdev)
{

	struct matrixio_pwm_chip *matrixio_pwm;

	matrixio_pwm =
	    devm_kzalloc(&pdev->dev, sizeof(*matrixio_pwm), GFP_KERNEL);
	if (!matrixio_pwm)
		return -ENOMEM;

	matrixio_pwm->chip.dev = &pdev->dev;
	matrixio_pwm->chip.ops = &matrixio_pwm_ops;
	matrixio_pwm->chip.npwm = NUM_PWM;
	matrixio_pwm->chip.base = -1;

	platform_set_drvdata(pdev, matrixio_pwm);

	return pwmchip_add(&matrixio_pwm->chip);
}

static int matrixio_pwm_remove(struct platform_device *pdev)
{
	struct matrixio_pwm_chip *matrixio_pwm = platform_get_drvdata(pdev);

	return pwmchip_remove(&matrixio_pwm->chip);
}

static struct platform_driver matrixio_pwm_driver = {
    .driver =
	{
	    .name = "matrixio-pwm",
	},
    .probe = matrixio_pwm_probe,
    .remove = matrixio_pwm_remove,
};

module_platform_driver(matrixio_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO PWM module");
