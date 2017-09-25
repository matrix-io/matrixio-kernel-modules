#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#include "matrixio-core.h"

ssize_t matrixio_everloop_read(struct file *pfile, char __user *buffer,
				size_t length, loff_t *offset)
{
	return 0;
}

ssize_t matrixio_everloop_write(struct file *pfile, const char __user *buffer,
				 size_t length, loff_t *offset)
{
	int i;
	for(i=0; i<length; i++)
		printk(KERN_INFO "0x%20x", buffer[i]);

	printk(KERN_INFO "l=%d o=%d", length, *offset);
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

static int matrixio_everloop_probe(struct platform_device *pdev)
{
	register_chrdev(301, "matrixio-everloop",
			&matrixio_everloop_file_operations);
	return 0;
}

static int  matrixio_everloop_remove(struct platform_device *pdev)
{
	unregister_chrdev(301, "matrixio-everloop");
	return 0;
}


static struct platform_driver matrixio_everloop_driver = {
	.driver = {
		.name = "matrixio-everloop",
	},
	.probe = matrixio_everloop_probe,
	.remove = matrixio_everloop_remove,
};

module_platform_driver(matrixio_everloop_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO Everloop module");

