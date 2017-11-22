#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "matrixio-core.h"

static struct matrixio *matrixio;
static struct class *cl;

ssize_t matrixio_everloop_read(struct file *pfile, char __user *buffer,
			       size_t length, loff_t *offset)
{
	return 0;
}

ssize_t matrixio_everloop_write(struct file *pfile, const char __user *buffer,
				size_t length, loff_t *offset)
{
	int i;
	uint16_t value;
	for (i = 0; i < length; i = i + 2) {
		value = buffer[i + 1] | buffer[i] << 8;
		regmap_write(matrixio->regmap,
			     MATRIXIO_EVERLOOP_BASE + (i >> 1), value);
	}
	return length;
}

int matrixio_everloop_open(struct inode *pinode, struct file *pfile)
{
	return 0;
}

int matrixio_everloop_close(struct inode *pinode, struct file *pfile)
{
	return 0;
}

struct file_operations matrixio_everloop_file_operations = {
    .owner = THIS_MODULE,
    .read = matrixio_everloop_read,
    .write = matrixio_everloop_write,
    .open = matrixio_everloop_open,
    .release = matrixio_everloop_close};

static struct cdev matrixio_everloop_cdev;

static int matrixio_everloop_probe(struct platform_device *pdev)
{
	dev_t devt;
	int major;

	matrixio = dev_get_drvdata(pdev->dev.parent);

	alloc_chrdev_region(&devt, 0, 1, "everloop");
	cl = class_create(THIS_MODULE, "matrixio_everloop");

	device_create(cl, NULL, devt, NULL, "everloop");

	cdev_init(&matrixio_everloop_cdev, &matrixio_everloop_file_operations);
	cdev_add(&matrixio_everloop_cdev, devt, 1);

	return 0;
}

static int matrixio_everloop_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	printk(KERN_INFO " 1: %s", pdev->name);

	unregister_chrdev(301, "matrixio-everloop");
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
