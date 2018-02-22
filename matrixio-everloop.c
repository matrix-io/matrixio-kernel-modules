#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "matrixio-core.h"

struct everloop_data {
	struct matrixio *mio;
	struct class *cl;
	dev_t devt;
	struct cdev cdev;
	struct device *device;
	int major;
};

ssize_t matrixio_everloop_write(struct file *pfile, const char __user *buffer,
				size_t length, loff_t *offset)
{
	struct everloop_data *el = pfile->private_data;
	int i;
	uint16_t value;
	for (i = 0; i < length; i = i + 2) {
		value = buffer[i + 1] | buffer[i] << 8;
		regmap_write(el->mio->regmap, MATRIXIO_EVERLOOP_BASE + (i >> 1),
			     value);
	}
	return length;
}

int matrixio_everloop_open(struct inode *inode, struct file *filp)
{

	struct everloop_data *el;
	el = container_of(inode->i_cdev, struct everloop_data, cdev);

	filp->private_data = el; /* For use elsewhere */

	return 0;
}

struct file_operations matrixio_everloop_file_operations = {
    .owner = THIS_MODULE,
    .open = matrixio_everloop_open,
    .write = matrixio_everloop_write};

static int matrixio_everloop_uevent(struct device *d,
				    struct kobj_uevent_env *env)
{
	add_uevent_var(env, "DEVMODE=%#o", 0666);
	return 0;
}

static int matrixio_everloop_probe(struct platform_device *pdev)
{
	struct everloop_data *el;

	el = devm_kzalloc(&pdev->dev, sizeof(struct everloop_data), GFP_KERNEL);

	if (el == NULL)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, el);

	el->mio = dev_get_drvdata(pdev->dev.parent);

	alloc_chrdev_region(&el->devt, 0, 1, "matrixio_everloop");

	el->cl = class_create(THIS_MODULE, "matrixio_everloop");

	el->cl->dev_uevent = matrixio_everloop_uevent;

	el->device =
	    device_create(el->cl, NULL, el->devt, NULL, "matrixio_everloop");

	if (IS_ERR(el->device)) {
		dev_err(&pdev->dev, "Unable to create device "
				    "for matrix; errno = %ld\n",
			PTR_ERR(el->device));
	}

	cdev_init(&el->cdev, &matrixio_everloop_file_operations);

	cdev_add(&el->cdev, el->devt, 1);

	//	el->cl->dev_uevent = matrixio_everloop_uevent;

	return 0;
}

static int matrixio_everloop_remove(struct platform_device *pdev)
{
	struct everloop_data *el = dev_get_drvdata(&pdev->dev);

	unregister_chrdev(el->major, "matrixio_everloop");

	return 0;
}

static struct platform_driver matrixio_everloop_driver = {
    .driver =
	{
	    .name = "matrixio-everloop",
	},
    .probe = matrixio_everloop_probe,
    .remove = matrixio_everloop_remove,
};

module_platform_driver(matrixio_everloop_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO Everloop module");
