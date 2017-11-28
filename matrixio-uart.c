#include "matrixio-core.h"
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/freezer.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>

static struct matrixio *matrixio;
static struct uart_port port;
static int irq;
static struct workqueue_struct *workqueue;
static struct work_struct work;
static int force_end_work;

#define MATRIXIO_UART_BUSY  0x0010

struct matrixio_uart_status
{
  uint8_t fifo_length : 6;
  uint8_t fifo_full : 1;
  uint8_t fifo_empty :1;
  uint8_t uart_ucr : 4;
  uint8_t uart_tx_busy:4;
};

static const char driver_name[] = "ttyMATRIX";
static const char tty_dev_name[] = "ttyMATRIX";

static irqreturn_t uart_rxint(int irq, void *id)
{
	if (!freezing(current))
    queue_work(workqueue, &work);
	return IRQ_HANDLED;
}

static void matrixio_uart_work(struct work_struct *w)
{
  unsigned int status;
  unsigned int val;
  struct matrixio_uart_status uart_status;
  //spin_lock(&port.lock);

  

  for( ;;) {
    regmap_read(matrixio->regmap, MATRIXIO_UART_BASE, (unsigned int *)&uart_status);
  printk(KERN_INFO "Fifo Number = %d", uart_status.fifo_length);
  printk(KERN_INFO "FiFo Empty = %d",uart_status.fifo_empty);
  
  if(uart_status.fifo_empty){
    printk(KERN_INFO "IRQ wasted");
    break ;
  }

    regmap_read(matrixio->regmap, MATRIXIO_UART_BASE + 1, &val);
    //spin_unlock(&port.lock);

    tty_insert_flip_char(&port.state->port, val, TTY_NORMAL);
    tty_flip_buffer_push(&port.state->port);

  }
  
}

static unsigned int matrixio_uart_tx_empty(struct uart_port *port) { return 1; }

static void matrixio_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int matrixio_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void matrixio_uart_stop_tx(struct uart_port *port) {}

static void matrixio_uart_start_tx(struct uart_port *port)
{

	while (1) {
    regmap_write(matrixio->regmap, MATRIXIO_UART_BASE + 1,
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

static int matrixio_uart_startup(struct uart_port *port) { return 0; }
static void matrixio_uart_shutdown(struct uart_port *port) {
  flush_workqueue(workqueue);
}

static void matrixio_uart_set_termios(struct uart_port *port,
				      struct ktermios *termios,
				      struct ktermios *old)
{
}

static const char *matrixio_uart_type(struct uart_port *port)
{
	return "matrixio-uart";
}

static int matrixio_uart_request_port(struct uart_port *port) { return 0; }

static void matrixio_uart_config_port(struct uart_port *port, int flags) {}

static void matrixio_uart_release_port(struct uart_port *port) {}

static int matrixio_uart_verify_port(struct uart_port *port,
				     struct serial_struct *ser)
{
	return 0;
}

static struct uart_ops matrixio_uart_ops = {
    .tx_empty = matrixio_uart_tx_empty,
    .set_mctrl = matrixio_uart_set_mctrl,
    .get_mctrl = matrixio_uart_get_mctrl,
    .stop_tx = matrixio_uart_stop_tx,
    .start_tx = matrixio_uart_start_tx,
    .stop_rx = matrixio_uart_stop_rx,
    .enable_ms = matrixio_uart_enable_ms,
    .break_ctl = matrixio_uart_break_ctl,
    .startup = matrixio_uart_startup,
    .shutdown = matrixio_uart_shutdown,
    .set_termios = matrixio_uart_set_termios,
    .type = matrixio_uart_type,
    .release_port = matrixio_uart_release_port,
    .request_port = matrixio_uart_request_port,
    .config_port = matrixio_uart_config_port,
    .verify_port = matrixio_uart_verify_port,
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
  char workqueue_name[12];

	matrixio = dev_get_drvdata(pdev->dev.parent);

	if (np)
		dev_dbg(dev, "get of data\n");

	ret = uart_register_driver(&matrixio_uart_driver);

	if (ret != 0) {
		dev_err(matrixio->dev, "Failed to register MATRIXIO UART: %d\n",
			ret);
		return ret;
	}

  irq = irq_of_parse_and_map(np, 0);

	spin_lock_init(&port.lock);
	port.irq = irq;
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

  sprintf(workqueue_name, "matrixio_uart");

  workqueue = create_freezable_workqueue(workqueue_name);

  if (!workqueue) {
    dev_err(&pdev->dev, "cannot create workqueue");
    return -EBUSY;
  }

  force_end_work = 0;

  INIT_WORK(&work, matrixio_uart_work);

  ret = request_irq(irq, uart_rxint, 0, driver_name, matrixio);

	if (ret) {
		dev_err(&pdev->dev, "can't request irq %d\n", irq);
    destroy_workqueue(workqueue);
    return -EBUSY;
	}

	printk(KERN_INFO "MATRIX Creator TTY has been loaded (IRQ=%d,%d)", irq,
	       ret);

	return ret;
}

static int matrixio_uart_remove(struct platform_device *pdev)
{
	free_irq(irq, matrixio);
	uart_remove_one_port(&matrixio_uart_driver, &port);
	uart_unregister_driver(&matrixio_uart_driver);
	return 0;
}

static const struct of_device_id matrixio_uart_dt_ids[] = {
    {.compatible = "matrixio-uart", .data = (void *)0}, {}};

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
