#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>
#include "matrixio-core.h"

static struct matrixio *matrixio;
static struct uart_port port;
static int irq;

static const char driver_name[] = "ttyMATRIX";
static const char tty_dev_name[] = "ttyMATRIX";

static void matrixio_uart_putc(struct uart_port *port, unsigned char c)
{
	printk(KERN_INFO "MATRIXIO UART PUT Character\n");

	// outb(c, FPGA_BASE);
}

static irqreturn_t uart_rxint(int irq, void *id)
{
	// tty_insert_flip_char(port.state->port.tty, inb(FPGA_BASE),
	// TTY_NORMAL);
	// tty_flip_buffer_push(port.state->port.tty);
	//
	//
	//
	unsigned int val;

	regmap_read(matrixio->regmap, MATRIXIO_UART_BASE+1,&val);
	
	tty_insert_flip_char(&port.state->port, val,TTY_NORMAL);
	tty_flip_buffer_push(&port.state->port);

	printk(KERN_INFO "MATRIXIO UART RX int\n");
	return IRQ_HANDLED;
}

static unsigned int matrixio_uart_tx_empty(struct uart_port *port)
{
	printk(KERN_INFO "MATRIXIO UART Empty\n");
	return 1;
}

static void matrixio_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	printk(KERN_INFO "MATRIXIO UART SEt mctrl\n");
}

static unsigned int matrixio_uart_get_mctrl(struct uart_port *port)
{
	printk(KERN_INFO "MATRIXIO UART Get mctrl\n");
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void matrixio_uart_stop_tx(struct uart_port *port) {}

static void matrixio_uart_start_tx(struct uart_port *port)
{
	printk(KERN_INFO "MATRIXIO UART start TX\n");

	while (1) {
		//regmap_write(matrixio->regmap, MATRIXIO_UART_BASE, port->state->xmit.buf[port->state->xmit.tail]);
		printk("enviado: %c\n",
		       port->state->xmit.buf[port->state->xmit.tail]);
		
		port->state->xmit.tail =
		    (port->state->xmit.tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		
		if (uart_circ_empty(&port->state->xmit))
			break;
	}
}
static void matrixio_uart_stop_rx(struct uart_port *port) {}

static void matrixio_uart_enable_ms(struct uart_port *port) {}

static void matrixio_uart_break_ctl(struct uart_port *port, int break_state) {}

static int matrixio_uart_startup(struct uart_port *port)
{

	printk(KERN_INFO "MATRIXIO UART port open\n");

	/*
		int res;
		printk("fpga_startup\n");
		res = request_irq(port->irq, vuart_rxint, IRQF_TRIGGER_FALLING,
				  "ttyFPGA", (void *)port); // IRQF_DISABLED |
		if (res) {
			printk("No se pudo registrar el IRQ!\n");
			return res;
		}
		// Envío los registros de inicialización para la velocidad de
		// transmisión y recepción, también el formato de datos...
		outb(0x80, FPGA_BASE + LCR);
		// acá lo coloco a 9600 baudios porque es la velocidad de
	   transmisión
		// (esto es temporal).
		outb(0x00, FPGA_BASE + DIVMSB);
		outb(0x0c, FPGA_BASE + DIVLSB);
		// registro para habilitar las interrupciones...
		outb(0x03, FPGA_BASE + LCR);
		outb(0xCF, FPGA_BASE + FCR);
		outb(0x0B, FPGA_BASE + MCR);
		outb(0x01, FPGA_BASE + IER);
	*/
	return 0;
}
static void matrixio_uart_shutdown(struct uart_port *port)
{
	printk(KERN_INFO "MATRIXIO UART Close port\n");

	// free_irq(port->irq, (void *)port);
}

static void matrixio_uart_set_termios(struct uart_port *port,
				      struct ktermios *termios,
				      struct ktermios *old)
{
	printk(KERN_INFO "MATRIXIO UART Termios\n");
}

static const char *matrixio_uart_type(struct uart_port *port)
{

	printk(KERN_INFO "MATRIXIO UART Type\n");
	return "matrixio-uart";
}

static int matrixio_uart_request_port(struct uart_port *port)
{

	printk(KERN_INFO "MATRIXIO UART Request\n");
	return 0;
}

static void matrixio_uart_config_port(struct uart_port *port, int flags)
{
	printk(KERN_INFO "MATRIXIO UART Config port\n");

	// if (flags & UART_CONFIG_TYPE)
	//		port->type = PORT_16550A;
}
static void matrixio_uart_release_port(struct uart_port *port)
{
	printk(KERN_INFO "MATRIXIO UART Realse\n");
}

static int matrixio_uart_verify_port(struct uart_port *port,
				     struct serial_struct *ser)
{
	printk(KERN_INFO "MATRIXIO UART verify port\n");
	return 0;
}

static struct uart_ops matrixio_uart_ops = {	
	.tx_empty =     matrixio_uart_tx_empty,
	.set_mctrl =    matrixio_uart_set_mctrl,
	.get_mctrl =    matrixio_uart_get_mctrl,
	.stop_tx =      matrixio_uart_stop_tx,
	.start_tx =     matrixio_uart_start_tx,
	.stop_rx =      matrixio_uart_stop_rx,
	.enable_ms =    matrixio_uart_enable_ms,
	.break_ctl =    matrixio_uart_break_ctl,
	.startup =      matrixio_uart_startup,
	.shutdown =     matrixio_uart_shutdown,
	.set_termios =  matrixio_uart_set_termios,
	.type =         matrixio_uart_type,
	.release_port = matrixio_uart_release_port,
	.request_port = matrixio_uart_request_port,
	.config_port =  matrixio_uart_config_port,
	.verify_port =  matrixio_uart_verify_port,
};

static struct uart_driver matrixio_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = driver_name,
	.dev_name = tty_dev_name,
	.major = 204,
	.minor = 209,
	.nr = 1,
};

static int matrixio_uart_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	matrixio = dev_get_drvdata(pdev->dev.parent);
 
	//np = matrixio->spi->dev.of_node;


	if (np)
		dev_dbg(dev, "get of data\n");

	ret = uart_register_driver(&matrixio_uart_driver);

	if (ret != 0) {
		dev_err(matrixio->dev, "Failed to register MATRIXIO UART: %d\n",
			ret);
		return ret;
	}

	port.irq = 0;
	port.fifosize = 16;
	port.line = 0;
	port.ops = &matrixio_uart_ops;
	port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
	port.dev = &pdev->dev;
	port.type = PORT_MAX3100;
	port.line = 0;

	ret = uart_add_one_port(&matrixio_uart_driver, &port);

	if (ret != 0) {
		dev_err(matrixio->dev, "Failed to add port: %d\n", ret);
		return ret;
	}
	irq = irq_of_parse_and_map(np, 0);

    	ret = request_irq(irq, uart_rxint, 0, driver_name, matrixio);

        if (unlikely(ret)) {
		        dev_err(&pdev->dev, "can't request irq %d\n", irq);
			    }
	printk(KERN_INFO "MATRIX Creator TTY has been loaded (IRQ=%d,%d)",irq, ret);

	return ret;
}

static int matrixio_uart_remove(struct platform_device *pdev)
{
	printk(KERN_INFO " 1: %s", pdev->name);
	free_irq(irq, matrixio);
	uart_remove_one_port(&matrixio_uart_driver, &port);
	uart_unregister_driver(&matrixio_uart_driver);
	return 0;
}


static const struct of_device_id matrixio_uart_dt_ids[] = {
	{ 
		.compatible = "matrixio-uart", 
		.data = (void *) 0 
	}, 
	{}
};

MODULE_DEVICE_TABLE(of, matrixio_uart_dt_ids);

static struct platform_driver matrixio_uart_platform_driver = {
    .driver =
	{
	    .name = "matrixio-uart",
	    .of_match_table = of_match_ptr(matrixio_uart_dt_ids),
	},
    .probe = matrixio_uart_probe,
    .remove = matrixio_uart_remove,

};

module_platform_driver(matrixio_uart_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO TTY module");
