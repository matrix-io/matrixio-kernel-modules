#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#include "matrixio-core.h"

struct regmap_data {
	struct matrixio *mio;
	struct class *cl;
	dev_t devt;
	struct cdev cdev;
	struct device *device;
	int major;
};

int matrixio_regmap_open(struct inode *inode, struct file *filp)
{

	struct regmap_data *el;
	el = container_of(inode->i_cdev, struct regmap_data, cdev);

	filp->private_data = el; /* For use elsewhere */

	return 0;
}

#define WR_VALUE 1200
#define RD_VALUE 1201

static long matrixio_regmap_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	struct regmap_data *el = file->private_data;
	int32_t *user_buffer;
	static int32_t data[12000];

	switch (cmd) {
	case WR_VALUE:
		user_buffer = (int32_t *)arg;
		if (copy_from_user(data, user_buffer, sizeof(int32_t) * 2))
			return -EFAULT;

		if (copy_from_user(&data[2], user_buffer + 2, data[1]))
			return -EFAULT;

		return matrixio_write(el->mio, data[0], data[1],
				      (void *)&data[2]);

	case RD_VALUE:
		user_buffer = (int32_t *)arg;
		if (copy_from_user(data, user_buffer, sizeof(int32_t) * 2))
			return -EFAULT;

		matrixio_read(el->mio, data[0], data[1], (void *)&data[2]);
		if (copy_to_user(user_buffer + 2, &data[2], data[1]))
			return -EFAULT;
		return 0;
	}
	return -EINVAL;
}

struct file_operations matrixio_regmap_file_operations = {
    .owner = THIS_MODULE,
    .open = matrixio_regmap_open,
    .unlocked_ioctl = matrixio_regmap_ioctl};

static int matrixio_regmap_uevent(struct device *d, struct kobj_uevent_env *env)
{
	add_uevent_var(env, "DEVMODE=%#o", 0666);
	return 0;
}

static int matrixio_regmap_probe(struct platform_device *pdev)
{
	struct regmap_data *el;

	el = devm_kzalloc(&pdev->dev, sizeof(struct regmap_data), GFP_KERNEL);

	if (el == NULL)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, el);

	el->mio = dev_get_drvdata(pdev->dev.parent);

	alloc_chrdev_region(&el->devt, 0, 1, "matrixio_regmap");
	el->cl = class_create(THIS_MODULE, "matrixio_regmap");

	el->cl->dev_uevent = matrixio_regmap_uevent;

	el->device =
	    device_create(el->cl, NULL, el->devt, NULL, "matrixio_regmap");

	if (IS_ERR(el->device)) {
		dev_err(&pdev->dev, "Unable to create device "
				    "for matrix; errno = %ld\n",
			PTR_ERR(el->device));
	}
	cdev_init(&el->cdev, &matrixio_regmap_file_operations);
	cdev_add(&el->cdev, el->devt, 1);

	return 0;
}

static int matrixio_regmap_remove(struct platform_device *pdev)
{
	struct regmap_data *el = dev_get_drvdata(&pdev->dev);

	unregister_chrdev(el->major, "matrixio_regmap");

	return 0;
}

static struct platform_driver matrixio_regmap_driver = {
    .driver =
	{
	    .name = "matrixio-regmap",
	},
    .probe = matrixio_regmap_probe,
    .remove = matrixio_regmap_remove,
};

module_platform_driver(matrixio_regmap_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO regmap module");
