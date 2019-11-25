/*
 * uartlite.c: Serial driver for Xilinx uartlite serial controller
 *
 * Copyright (C) 2006 Peter Korsgaard <jacmet@sunsite.dk>
 * Copyright (C) 2007 Secret Lab Technologies Ltd.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#define UHZHY_NAME		"ttyUH"
#define UHZHY_MAJOR		205
#define UHZHY_MINOR		187
#define UHZHY_NR_UARTS		16

/* ---------------------------------------------------------------------
 * Register definitions
 *
 * For register details see datasheet:
 * http://www.xilinx.com/support/documentation/ip_documentation/opb_uartlite.pdf
 */
/*
#define UHZHY_RX		0x00
#define UHZHY_TX		0x04
#define UHZHY_STATUS		0x08
#define UHZHY_CONTROL		0x0c

//#define UHZHY_REGION		16

#define UHZHY_STATUS_RXVALID	0x01
#define UHZHY_STATUS_RXFULL	0x02
#define UHZHY_STATUS_TXEMPTY	0x04
#define UHZHY_STATUS_TXFULL	0x08
#define UHZHY_STATUS_IE		0x10
#define UHZHY_STATUS_OVERRUN	0x20
#define UHZHY_STATUS_FRAME	0x40
#define UHZHY_STATUS_PARITY	0x80

#define UHZHY_CONTROL_RST_TX	0x01
#define UHZHY_CONTROL_RST_RX	0x02
#define UHZHY_CONTROL_IE	0x10
*/
#define UHZHY_CLK_RATIO			0x00	/* 时钟分频值 clk_ratio = clk÷(16 x buad_rate) */
#define UHZHY_PARITY			0x04	/* 奇偶校验 2'b00: 不校验;2'b11: 奇校验; 2'b10：偶校验  */
#define UHZHY_STOP_BIT			0x08	/* 停止位宽度 0: 1bit; 1：2bit */
#define UHZHY_THR_INT_RX		0x0c 	/* 接收门限,当接收FIFO内数据超过这个门限时，发出中断;当发送完成后，也会发出中断  */
#define UHZHY_OVER_TIME			0x10	/* 超时时间，单位：us.当FIFO接收数据不满足门限时，超时后发出中断 */
#define UHZHY_RX_FIFO_STA		0x14	/* 接收FIFO的状态指示 bit0: full; bit1: empty */
#define UHZHY_RX_FIFO_CNT		0x18	/* 接收FIFO内数据计数  */
#define UHZHY_TX_FIFO_STA		0x1c	/* 发送FIFO的状态指示 bit0: full; bit1: empty */
#define UHZHY_TX_FIFO_CNT		0x20	/* 发送FIFO内数据计数 */
#define UHZHY_RX_ENA			0x24	/* 接收使能 */
#define UHZHY_TX_ENA			0x28	/* 发送使能  */
#define UHZHY_RX_FIFO_DAT		0x2c	/* 接收FIFO接口  */
#define UHZHY_TX_FIFO_DAT		0x30	/* 发送FIFO接口  */
#define UHZHY_INT_CTRL			0x34	/* 中断使能 0：使能 1：关闭中断  */
#define UHZHY_FIFO_CLR			0x38	/* 置1，清空fifo */

#define UHZHY_REGION			16

#define UHZHY_STATUS_RXFULL		0x01
#define UHZHY_STATUS_RXEMPTY	0x02
#define UHZHY_STATUS_TXFULL		0x01
#define UHZHY_STATUS_TXEMPTY	0x02

#define UHZHY_ENABLE_RX			0x01
#define UHZHY_DISABLE_RX		0x00
#define UHZHY_ENABLE_TX			0x01
#define UHZHY_DISABLE_TX		0x00

#define UHZHY_CHECK_NO			0x00	/* 不校验 */
#define UHZHY_CHECK_EVEN		0x02	/* 偶校验 */
#define UHZHY_CHECK_OLD			0x03	/* 奇校验 */

#define UHZHY_STOP_1BIT			0x00	/* 1位停止位 */
#define UHZHY_STOP_2BIT			0x01	/* 2位停止位 */

#define	UHZHY_THR_NUM			16		/* 接收门限,当接收FIFO内数据超过这个门限时，发出中断 */

#define	INT_ENABLE				0x01
#define	INT_DISABLE				0x00

#define FIFO_CLEAR				0x01

struct uarthzhy_reg_ops {
	u32 (*in)(void __iomem *addr);
	void (*out)(u32 val, void __iomem *addr);
};

static u32 uarthzhy_inbe32(void __iomem *addr)
{
	return ioread32be(addr);
}

static void uarthzhy_outbe32(u32 val, void __iomem *addr)
{
	iowrite32be(val, addr);
}

static struct uarthzhy_reg_ops uarthzhy_be = {
	.in = uarthzhy_inbe32,
	.out = uarthzhy_outbe32,
};

static u32 uarthzhy_inle32(void __iomem *addr)
{
	return ioread32(addr);
}

static void uarthzhy_outle32(u32 val, void __iomem *addr)
{
	iowrite32(val, addr);
}

static struct uarthzhy_reg_ops uarthzhy_le = {
	.in = uarthzhy_inle32,
	.out = uarthzhy_outle32,
};

static inline u32 uart_in32(u32 offset, struct uart_port *port)
{
	struct uarthzhy_reg_ops *reg_ops = port->private_data;

	return reg_ops->in(port->membase + offset);
}

static inline void uart_out32(u32 val, u32 offset, struct uart_port *port)
{
	struct uarthzhy_reg_ops *reg_ops = port->private_data;

	reg_ops->out(val, port->membase + offset);
}

static struct uart_port uhzhy_ports[UHZHY_NR_UARTS];

/* ---------------------------------------------------------------------
 * Core UART driver operations
 */

static int uhzhy_receive(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	unsigned char ch = 0;
	int stat;
	int cnt = 0;
//	int countTest = 0;
	char flag = TTY_NORMAL;

	stat = uart_in32(UHZHY_RX_FIFO_STA, port);
//	countTest++;
//	printk("==[%s:%s]%d stat=0x%x==\n", __FILE__, __func__,__LINE__, stat);
	if (stat & UHZHY_STATUS_RXEMPTY)
	{
//		printk("==[%s:%s]%d stat=0x%x cnt=0x%x countTest=%d=\n", __FILE__, __func__,__LINE__, stat, cnt, countTest);
		return 0;
	}

	cnt = uart_in32(UHZHY_RX_FIFO_CNT, port);
//		printk("==[%s:%s]%d cnt=0x%x==\n", __FILE__, __func__,__LINE__);
	if(cnt != 0)
	{
		port->icount.rx++;
		ch = uart_in32(UHZHY_RX_FIFO_DAT, port);
//		printk("==[%s:%s]%d ch[%d]=0x%x==\n", __FILE__, __func__,__LINE__, port->icount.rx, ch);
		tty_insert_flip_char(tport, ch, flag);
	}
	else
	{
		return 0;
	}

	return 1;
}

static int uhzhy_transmit(struct uart_port *port)
{
	struct circ_buf *xmit  = &port->state->xmit;
	int stat;
	int cnt;

	stat = uart_in32(UHZHY_TX_FIFO_STA, port);

//	printk("==[%s:%s]%d stat=0x%x==\n", __FILE__, __func__,__LINE__, stat);

	if (stat & UHZHY_STATUS_TXFULL)
		return 0;

//	printk("==[%s:%s]%d x_char=0x%x==\n", __FILE__, __func__,__LINE__, port->x_char);
	if (port->x_char) {
		uart_out32(port->x_char, UHZHY_TX_FIFO_DAT, port);
//		printk("########### [%s:%s]%d x_char=0x%x==\n", __FILE__, __func__,__LINE__, port->x_char);
		port->x_char = 0;
		port->icount.tx++;
		return 1;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return 0;

//	printk("==[%s:%s]%d ==\n", __FILE__, __func__,__LINE__);
//	while (!uart_circ_empty(xmit))
	cnt=uart_in32(UHZHY_TX_FIFO_CNT,port);
	if (!uart_circ_empty(xmit) || (cnt != 0))
	{
//		printk("==[%s:%s]%d buf[%d]=0x%x==\n", __FILE__, __func__,__LINE__, xmit->tail, xmit->buf[xmit->tail]);
		uart_out32(xmit->buf[xmit->tail], UHZHY_TX_FIFO_DAT, port);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
		port->icount.tx++;
	}

//	printk("==[%s:%s]%d ==\n", __FILE__, __func__,__LINE__);
	/* wake up */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	return 1;
}

static irqreturn_t uhzhy_isr(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned long flags;
	int stat, busy = 0;

//	printk("==[%s:%s]%d===\n", __FILE__, __func__,__LINE__);

	do
	{
		spin_lock_irqsave(&port->lock, flags);
		busy = uhzhy_receive(port);
		busy |= uhzhy_transmit(port);
		spin_unlock_irqrestore(&port->lock, flags);
	}while (busy);

	tty_flip_buffer_push(&port->state->port);

	return IRQ_HANDLED;
}

static unsigned int uhzhy_tx_empty(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&port->lock, flags);
	ret = uart_in32(UHZHY_TX_FIFO_STA, port);
	spin_unlock_irqrestore(&port->lock, flags);

	return ret & UHZHY_STATUS_TXEMPTY ? TIOCSER_TEMT : 0;
}

static unsigned int uhzhy_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void uhzhy_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* N/A */
}

static void uhzhy_stop_tx(struct uart_port *port)
{
	printk("==[%s:%s]%d===\n", __FILE__, __func__,__LINE__);
	/* N/A */
}

static void uhzhy_start_tx(struct uart_port *port)
{
//	printk("==[%s:%s]%d===\n", __FILE__, __func__,__LINE__);
	uhzhy_transmit(port);
}

static void uhzhy_stop_rx(struct uart_port *port)
{
	/* don't forward any more data (like !CREAD) */
	printk("==[%s:%s]%d===\n", __FILE__, __func__,__LINE__);
	/* don't forward any more data (like !CREAD) */
	port->ignore_status_mask = UHZHY_STATUS_RXFULL;
//	port->ignore_status_mask = UHZHY_STATUS_RXVALID | UHZHY_STATUS_PARITY
//		| UHZHY_STATUS_FRAME | UHZHY_STATUS_OVERRUN;
}

static void uhzhy_break_ctl(struct uart_port *port, int ctl)
{
	/* N/A */
}

static int uhzhy_startup(struct uart_port *port)
{
	int ret;

//	printk("==[%s:%s]%d===\n", __FILE__, __func__,__LINE__);
	ret = request_irq(port->irq, uhzhy_isr, IRQF_SHARED | IRQF_TRIGGER_RISING,
			  "uarthzhy", port);
	if (ret)
		return ret;

//	//enable receive and transmit
//	uart_out32(UHZHY_ENABLE_RX, UHZHY_RX_ENA, port);
//	uart_out32(UHZHY_ENABLE_TX, UHZHY_TX_ENA, port);
//	uart_out32(INT_ENABLE, UHZHY_INT_CTRL, port);

	//set receive threshlod
	uart_out32(UHZHY_THR_NUM, UHZHY_THR_INT_RX, port);

	//clear fifo
	uart_out32(FIFO_CLEAR, UHZHY_FIFO_CLR, port);
	uart_out32(0, UHZHY_FIFO_CLR, port);

//	printk("==[%s:%s]%d interrupt = 0x%x===\n", __FILE__, __func__,__LINE__, uart_in32(UHZHY_INT_CTRL, port));

	return 0;
}

static void uhzhy_shutdown(struct uart_port *port)
{
	printk("==[%s:%s]%d===\n", __FILE__, __func__,__LINE__);

	//disable receive and transmit
	uart_out32(UHZHY_DISABLE_RX, UHZHY_RX_ENA, port);
	uart_out32(UHZHY_DISABLE_TX, UHZHY_TX_ENA, port);
	uart_out32(INT_DISABLE, UHZHY_INT_CTRL, port);
	free_irq(port->irq, port);
}

static void uhzhy_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud;
	unsigned int parity=0;
	unsigned int stop_bit_set;

	spin_lock_irqsave(&port->lock, flags);

	/* enable receive and transmit */
	uart_out32(UHZHY_DISABLE_RX, UHZHY_RX_ENA, port);
	uart_out32(UHZHY_DISABLE_TX, UHZHY_TX_ENA, port);

	port->read_status_mask = UHZHY_STATUS_RXFULL | UHZHY_STATUS_TXFULL;

//	if (termios->c_iflag & INPCK)
//		port->read_status_mask |=
//			UHZHY_STATUS_PARITY | UHZHY_STATUS_FRAME;

//	port->ignore_status_mask = 0;
//	if (termios->c_iflag & IGNPAR)
//		port->ignore_status_mask |= UHZHY_STATUS_PARITY
//			| UHZHY_STATUS_FRAME | UHZHY_STATUS_OVERRUN;

	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UHZHY_STATUS_RXFULL;
//			UHZHY_STATUS_RXVALID | UHZHY_STATUS_PARITY
//			| UHZHY_STATUS_FRAME | UHZHY_STATUS_OVERRUN;

	/* update timeout */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 16);
	uart_update_timeout(port, termios->c_cflag, baud);

	/* Set stop bit */
	if (termios->c_cflag & CSTOPB)
	{
		

		stop_bit_set = UHZHY_STOP_2BIT;
		/* set parity */
		
	}
	else
	{
		
		stop_bit_set = UHZHY_STOP_1BIT;
	}
	
	if (termios->c_cflag & PARENB)
		{

			
			if (termios->c_cflag & PARODD)
				parity = UHZHY_CHECK_OLD;
			else
				parity = UHZHY_CHECK_EVEN;
		}
		else
		{
			
			parity = UHZHY_CHECK_NO;
		}
	//set baudrate
	uart_out32( port->uartclk / (16 * baud), UHZHY_CLK_RATIO, port);
	printk("==WAZ DEBUG: uartclk = %d, clk_ratio = %d, baud = %d\n", port->uartclk, port->uartclk / (16 * baud), baud);

	uart_out32(parity, UHZHY_PARITY, port);
	

	uart_out32(stop_bit_set, UHZHY_STOP_BIT, port);

	/* set timeout */
//	uart_out32( termios->c_cc[VTIME], UHZHY_OVER_TIME, port);
	//set_timeout = (unsigned int)((1.0/baud) * 11.0 * 1000000 * 2); //(1/baud) * 11 为uart发送一个字节的时间； 乘1000000，为转化为us； 乘2，为将延时时间设置为发送一个字节的时间的2倍
	uart_out32(2000, UHZHY_OVER_TIME, port);
//	printk("==[%s:%s]%d VTIME=%d VMIN=%d==\n", __FILE__, __func__,__LINE__, termios->c_cc[VTIME], termios->c_cc[VMIN]);

	/* Enable interrupt */
	uart_out32(INT_ENABLE, UHZHY_INT_CTRL, port);

	/* enable receive and transmit */
	uart_out32(UHZHY_ENABLE_RX, UHZHY_RX_ENA, port);
	uart_out32(UHZHY_ENABLE_TX, UHZHY_TX_ENA, port);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *uhzhy_type(struct uart_port *port)
{
	return port->type == PORT_UARTHZHY ? "uarthzhy" : NULL;
}

static void uhzhy_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, UHZHY_REGION);
	iounmap(port->membase);
	port->membase = NULL;
}

static int uhzhy_request_port(struct uart_port *port)
{
	int ret;

	pr_debug("uhzhy console: port=%p; port->mapbase=%llx\n",
		 port, (unsigned long long) port->mapbase);

	if (!request_mem_region(port->mapbase, UHZHY_REGION, "uarthzhy")) {
		dev_err(port->dev, "Memory region busy\n");
		return -EBUSY;
	}

	port->membase = ioremap(port->mapbase, UHZHY_REGION);
	if (!port->membase) {
		dev_err(port->dev, "Unable to map registers\n");
		release_mem_region(port->mapbase, UHZHY_REGION);
		return -EBUSY;
	}

	printk("==[%s:%s]%d  port->membase=0x%x \n", __FILE__, __func__,__LINE__, (unsigned int)port->membase);

//	uart_out32(UHZHY_ENABLE_TX, UHZHY_TX_ENA, port);

	port->private_data = &uarthzhy_be;

	uart_out32(UHZHY_DISABLE_TX, UHZHY_TX_ENA, port);
	uart_out32(UHZHY_DISABLE_RX, UHZHY_RX_ENA, port);
//	ret = uart_in32(UHZHY_CONTROL, port);
//	uart_out32(UHZHY_CONTROL_RST_TX, UHZHY_CONTROL, port);
//	ret = uart_in32(UHZHY_STATUS, port);
	ret = uart_in32(UHZHY_TX_FIFO_STA, port);
	/* Endianess detection */
	if ((ret & UHZHY_STATUS_TXEMPTY) != UHZHY_STATUS_TXEMPTY)
	{
		printk("==[%s:%s]%d  ==== \n", __FILE__, __func__,__LINE__);
		port->private_data = &uarthzhy_le;
	}

	return 0;
}

static void uhzhy_config_port(struct uart_port *port, int flags)
{
	if (!uhzhy_request_port(port))
		port->type = PORT_UARTHZHY;
}

static int uhzhy_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	return -EINVAL;
}

#ifdef CONFIG_CONSOLE_POLL
static int uhzhy_get_poll_char(struct uart_port *port)
{
	if (!(uart_in32(UHZHY_STATUS, port) & UHZHY_STATUS_RXVALID))
		return NO_POLL_CHAR;

	return uart_in32(UHZHY_RX, port);
}

static void uhzhy_put_poll_char(struct uart_port *port, unsigned char ch)
{
	while (uart_in32(UHZHY_STATUS, port) & UHZHY_STATUS_TXFULL)
		cpu_relax();

	/* write char to device */
	uart_out32(ch, UHZHY_TX, port);
}
#endif

static struct uart_ops uhzhy_ops = {
	.tx_empty	= uhzhy_tx_empty,
	.set_mctrl	= uhzhy_set_mctrl,
	.get_mctrl	= uhzhy_get_mctrl,
	.stop_tx	= uhzhy_stop_tx,
	.start_tx	= uhzhy_start_tx,
	.stop_rx	= uhzhy_stop_rx,
	.break_ctl	= uhzhy_break_ctl,
	.startup	= uhzhy_startup,
	.shutdown	= uhzhy_shutdown,
	.set_termios	= uhzhy_set_termios,
	.type		= uhzhy_type,
	.release_port	= uhzhy_release_port,
	.request_port	= uhzhy_request_port,
	.config_port	= uhzhy_config_port,
	.verify_port	= uhzhy_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= uhzhy_get_poll_char,
	.poll_put_char	= uhzhy_put_poll_char,
#endif
};

/* ---------------------------------------------------------------------
 * Console driver operations
 */

#ifdef CONFIG_SERIAL_UARTHZHY_CONSOLE
static void uhzhy_console_wait_tx(struct uart_port *port)
{
	u8 val;
	unsigned long timeout;

	/*
	 * Spin waiting for TX fifo to have space available.
	 * When using the Microblaze Debug Module this can take up to 1s
	 */
	timeout = jiffies + msecs_to_jiffies(1000);
	while (1) {
		val = uart_in32(UHZHY_TX_FIFO_STA, port);
		if ((val & UHZHY_STATUS_TXFULL) == 0)
			break;
		if (time_after(jiffies, timeout)) {
			dev_warn(port->dev,
				 "timeout waiting for TX buffer empty\n");
			break;
		}
		cpu_relax();
	}
}

static void uhzhy_console_putchar(struct uart_port *port, int ch)
{
	uhzhy_console_wait_tx(port);
	uart_out32(ch, UHZHY_TX_FIFO_DAT, port);
}

static void uhzhy_console_write(struct console *co, const char *s,
				unsigned int count)
{
	struct uart_port *port = &uhzhy_ports[co->index];
	unsigned long flags;
	unsigned int ier;
	int locked = 1;

	if (oops_in_progress) {
		locked = spin_trylock_irqsave(&port->lock, flags);
	} else
		spin_lock_irqsave(&port->lock, flags);

	/* save and disable interrupt */
	ier = uart_in32(UHZHY_INT_CTRL, port);
	uart_out32(INT_DISABLE, UHZHY_INT_CTRL, port);

	uart_console_write(port, s, count, uhzhy_console_putchar);

	uhzhy_console_wait_tx(port);

	/* restore interrupt state */
	uart_out32(ier, UHZHY_INT_CTRL, port);

	if (locked)
		spin_unlock_irqrestore(&port->lock, flags);
}

static int uhzhy_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= UHZHY_NR_UARTS)
		return -EINVAL;

	port = &uhzhy_ports[co->index];

	/* Has the device been initialized yet? */
	if (!port->mapbase) {
		pr_debug("console on ttyUH%i not present\n", co->index);
		return -ENODEV;
	}

	/* not initialized yet? */
	if (!port->membase) {
		if (uhzhy_request_port(port))
			return -ENODEV;
	}

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver uhzhy_uart_driver;

static struct console uhzhy_console = {
	.name	= UHZHY_NAME,
	.write	= uhzhy_console_write,
	.device	= uart_console_device,
	.setup	= uhzhy_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1, /* Specified on the cmdline (e.g. console=ttyUL0 ) */
	.data	= &uhzhy_uart_driver,
};

static int __init uhzhy_console_init(void)
{
	register_console(&uhzhy_console);
	return 0;
}

console_initcall(uhzhy_console_init);

static void early_uarthzhy_putc(struct uart_port *port, int c)
{
	/*
	 * Limit how many times we'll spin waiting for TX FIFO status.
	 * This will prevent lockups if the base address is incorrectly
	 * set, or any other issue on the UARTLITE.
	 * This limit is pretty arbitrary, unless we are at about 10 baud
	 * we'll never timeout on a working UART.
	 */

	unsigned retries = 1000000;
	/* read status bit - 0x8 offset */
	while (--retries && (readl(port->membase + 8) & (1 << 3)))
		;

	/* Only attempt the iowrite if we didn't timeout */
	/* write to TX_FIFO - 0x4 offset */
	if (retries)
		writel(c & 0xff, port->membase + 4);
}

static void early_uarthzhy_write(struct console *console,
				 const char *s, unsigned n)
{
	struct earlycon_device *device = console->data;
	uart_console_write(&device->port, s, n, early_uarthzhy_putc);
}

static int __init early_uarthzhy_setup(struct earlycon_device *device,
				       const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = early_uarthzhy_write;
	return 0;
}
EARLYCON_DECLARE(uarthzhy, early_uarthzhy_setup);
OF_EARLYCON_DECLARE(uarthzhy_b, "xlnx,opb-uarthzhy-1.00.b", early_uarthzhy_setup);
OF_EARLYCON_DECLARE(uarthzhy_a, "xlnx,xps-uarthzhy-1.00.a", early_uarthzhy_setup);

#endif /* CONFIG_SERIAL_UARTLITE_CONSOLE */

static struct uart_driver uhzhy_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "uarthzhy",
	.dev_name	= UHZHY_NAME,
	.major		= UHZHY_MAJOR,
	.minor		= UHZHY_MINOR,
	.nr		= UHZHY_NR_UARTS,
#ifdef CONFIG_SERIAL_UARTHZHY_CONSOLE
	.cons		= &uhzhy_console,
#endif
};

/* ---------------------------------------------------------------------
 * Port assignment functions (mapping devices to uart_port structures)
 */

/** uhzhy_assign: register a uarthzhy device with the driver
 *
 * @dev: pointer to device structure
 * @id: requested id number.  Pass -1 for automatic port assignment
 * @base: base address of uarthzhy registers
 * @irq: irq number for uarthzhy
 *
 * Returns: 0 on success, <0 otherwise
 */
static int uhzhy_assign(struct device *dev,  struct device_node *np, int id, u32 base, int irq)
{
	struct uart_port *port;
	int rc;
	u32 clk, spd;

	printk("==[%s:%s]%d  =\n", __FILE__, __func__,__LINE__);
	/* if id = -1; then scan for a free id and use that */
	if (id < 0) {
		for (id = 0; id < UHZHY_NR_UARTS; id++)
			if (uhzhy_ports[id].mapbase == 0)
				break;
	}
	if (id < 0 || id >= UHZHY_NR_UARTS) {
		dev_err(dev, "%s%i too large\n", UHZHY_NAME, id);
		return -EINVAL;
	}

	if ((uhzhy_ports[id].mapbase) && (uhzhy_ports[id].mapbase != base)) {
		dev_err(dev, "cannot assign to %s%i; it is already in use\n",
			UHZHY_NAME, id);
		return -EBUSY;
	}

	port = &uhzhy_ports[id];

	if (of_property_read_u32(np, "clock-frequency", &clk)) {

		/* Get clk rate through clk driver if present */
		printk("==ERROR: clk or clock-frequency not defined\n");
		clk = 100 * 1000 * 1000;
	}

	port->uartclk = clk;

	printk("==[%s:%s]%d clk=%d base=0x%x =\n", __FILE__, __func__,__LINE__, clk, base);

	spin_lock_init(&port->lock);
	port->fifosize = 16;
	port->regshift = 2;
	port->iotype = UPIO_MEM;
	port->iobase = 1; /* mark port in use */
	port->mapbase = base;
	port->membase = NULL;
	port->ops = &uhzhy_ops;
	port->irq = irq;
	port->flags = UPF_BOOT_AUTOCONF;
	port->dev = dev;
	port->type = PORT_UNKNOWN;
	port->line = id;

	dev_set_drvdata(dev, port);

	/* Register the port */
	rc = uart_add_one_port(&uhzhy_uart_driver, port);
	if (rc) {
		dev_err(dev, "uart_add_one_port() failed; err=%i\n", rc);
		port->mapbase = 0;
		dev_set_drvdata(dev, NULL);
		return rc;
	}

	return 0;
}

/** uhzhy_release: register a uarthzhy device with the driver
 *
 * @dev: pointer to device structure
 */
static int uhzhy_release(struct device *dev)
{
	struct uart_port *port = dev_get_drvdata(dev);
	int rc = 0;

	if (port) {
		rc = uart_remove_one_port(&uhzhy_uart_driver, port);
		dev_set_drvdata(dev, NULL);
		port->mapbase = 0;
	}

	return rc;
}

/* ---------------------------------------------------------------------
 * Platform bus binding
 */

#if defined(CONFIG_OF)
/* Match table for of_platform binding */
static const struct of_device_id uhzhy_of_match[] = {
	{ .compatible = "xlnx,opb-uarthzhy-1.00.b", },
	{ .compatible = "xlnx,xps-uarthzhy-1.00.a", },
	{}
};
MODULE_DEVICE_TABLE(of, uhzhy_of_match);
#endif /* CONFIG_OF */

static int uhzhy_probe(struct platform_device *pdev)
{
	struct resource *res;
	int irq;
	int id = pdev->id;
#ifdef CONFIG_OF
	const __be32 *prop;

	prop = of_get_property(pdev->dev.of_node, "port-number", NULL);
	if (prop)
		id = be32_to_cpup(prop);
#endif

	printk("==[%s:%s]%d id=%d =\n", __FILE__, __func__,__LINE__, id);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	printk("==[%s:%s]%d base=0x%x =\n", __FILE__, __func__,__LINE__, res->start);
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENXIO;

	printk("==[%s:%s]%d irq=%d =\n", __FILE__, __func__,__LINE__, irq);
	return uhzhy_assign(&pdev->dev, pdev->dev.of_node, id, res->start, irq);
}

static int uhzhy_remove(struct platform_device *pdev)
{
	return uhzhy_release(&pdev->dev);
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:uarthzhy");

static struct platform_driver uhzhy_platform_driver = {
	.probe = uhzhy_probe,
	.remove = uhzhy_remove,
	.driver = {
		.name  = "uarthzhy",
		.of_match_table = of_match_ptr(uhzhy_of_match),
	},
};

/* ---------------------------------------------------------------------
 * Module setup/teardown
 */

static int __init uhzhy_init(void)
{
	int ret;

	pr_debug("uarthzhy: calling uart_register_driver()\n");
	ret = uart_register_driver(&uhzhy_uart_driver);
	if (ret)
		goto err_uart;

	pr_debug("uarthzhy: calling platform_driver_register()\n");
	ret = platform_driver_register(&uhzhy_platform_driver);
	if (ret)
		goto err_plat;

	return 0;

err_plat:
	uart_unregister_driver(&uhzhy_uart_driver);
err_uart:
	pr_err("registering uarthzhy driver failed: err=%i", ret);
	return ret;
}

static void __exit uhzhy_exit(void)
{
	platform_driver_unregister(&uhzhy_platform_driver);
	uart_unregister_driver(&uhzhy_uart_driver);
}

module_init(uhzhy_init);
module_exit(uhzhy_exit);

MODULE_AUTHOR("Peter Korsgaard <jacmet@sunsite.dk>");
MODULE_DESCRIPTION("Xilinx uarthzhy serial driver");
MODULE_LICENSE("GPL");
