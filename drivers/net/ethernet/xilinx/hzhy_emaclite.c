/*
 * Xilinx EmacLite Linux driver for the Xilinx Ethernet MAC Lite device.
 *
 * This is a new flat driver which is based on the original emac_lite
 * driver from John Williams <john.williams@xilinx.com>.
 *
 * 2007 - 2013 (c) Xilinx, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define DRIVER_NAME "hzhy_emaclite"

/* Register offsets for the hzhy EmacLite Core */
/* Emac configuration register */
#define HZHY_EMAC_BASE_OFFSET	        0x10000
#define HZHY_MODER_OFFSET		0x0	/* Communication Mode Register */
#define HZHY_MINFLR_OFFSET		0x4	/* Min Data Frame Length Register */
#define HZHY_MAXFLR_OFFSET		0x8	/* Max Data Frame Length Register */
#define HZHY_MACADDR0_OFFSET	0xC	/* MAC Address Low Register */
#define HZHY_MACADDR1_OFFSET	0x10	/* MAC Address Heigh Register */
#define HZHY_HAFR0_OFFSET		0x14	/* Hash0 Filter Register */
#define HZHY_HAFR1_OFFSET		0x18	/* Hash1 Filter Register */
#define HZHY_RXCNTR_OFFSET		0x1C	/* Receive Frame Counter Register */
#define HZHY_TXCNTR_OFFSET		0x20	/* Transmit Frame Counter Register */
#define HZHY_MACRST_OFFSET		0x24	/* MAC Reset Register */
#define HZHY_PHYRST_OFFSET		0x28	/* MAC Reset Register */
#define EMAC_RST_MASK			0x00000001
#define EMAC_MACHIGHT_MASK		0x0000FFFF


/*MAC  CHECK*/
#define FILTER_MNT_CTR                       	0x2c
#define FILTER_MNT_GAP				0x30
#define FILTER_MNT_THR1				0x34
#define FILTER_MNT_THR2				0x38
#define FILTER_MNT_STAT				0x3c
#define FILTER_MNT_CNT				0x40
#define FILTER_MNT_FTHR1			0x44
#define FILTER_MNT_FTHR2			0x48
#define FILTER_MNT_FCNT				0x4c

/*MAC  CHECK*/
#define MNT_CTR		0
#define	MNT_GAP_TIME	5000000
#define	MNT_THR1_DOOR1	100000
#define	MNT_THR2_DOOR2	200000
#define	MNT_FTHR1_DOOR1	100
#define	MNT_FTHR2_DOOR2	200

//used by MODER register
#define EMAC_REV_ENABLE			1
#define EMAC_REV_DISABLE		0
#define EMAC_REV_EANBLE_SHIFT	0
#define EMAC_SEND_CRC_EN		1
#define EMAC_SEND_CRC_DIS		0
#define EMAC_SEND_CRC_SHIFT		1
#define EMAC_REV_DST_MAC_ONLY	0 //check destination mac address检验数据帧的目的地址
#define EMAC_REV_DST_MAC_ALL	1 //receive all data, ignore destination mac address接收所有数据帧不管其目的地址
#define EMAC_REV_DST_MAC_SHIFT	2
#define EMAC_REV_BROARDCAS_EN	1 /* 接收包含该地址的所有数据帧 */
#define EMAC_REV_BROARDCAS_DIS	0 /* 禁止接收包含该地址的所有帧，除非MODER[2]为1 */
#define EMAC_REV_BROARDCAST_SHIFT		3 /* broardcast addresis 广播地址设置*/
#define EMAC_LENGTHEN_EN  		1 /* 加长段数据帧直到其达到MIN_FL */
#define EMAC_LENGTHEN_DIS	  	0 /* 不加长短数据帧 */
#define EMAC_LENGTHEN_SHIFT   	4  /* 加长使能 */

//官方ip
#define XEL_MDIOADDR_OFFSET	0x07E4		/* MDIO Address Register */
#define XEL_MDIOWR_OFFSET	0x07E8		/* MDIO Write Data Register */
#define XEL_MDIORD_OFFSET	0x07EC		/* MDIO Read Data Register */
#define XEL_MDIOCTRL_OFFSET	0x07F0		/* MDIO Control Register */

/* MDIO register */
#if 0
#define HZHY_MDIO_BASE_OFFSET	0x100
#define HZHY_MDCMD_OFFSET	0x100		/* Mdio Command Register */
#define HZHY_MDADDR_OFFSET	0x104		/* Bit[12:8] Register Address, Bit[4:0] Phy Address */
#define HZHY_MDTXD_OFFSET 	0x108		/* Transmit Register */
#define HZHY_MDRXD_OFFSET  	0x10C		/* Receive Register */
#define HZHY_MDSTA_OFFSET	0x110		/* Bit[0], Bit[1], Bit[2] */
#define MDIO_CMDSCAN_MASK		0x00000001
#define MDIO_CMDREAD_MASK		0x00000002
#define MDIO_CMDWRITE_MASK		0x00000004
#define MDIO_PHYADDR_MASK		0x0000001F
#define MDIO_REGADDR_MASK		0x00001F00
#define MDIO_STALINK_MASK		0x00000001
#define MDIO_STABUSY_MASK		0x00000002
#define MDIO_STAINVALID_MASK		0x00000004
#else
/* MDIO Address Register Bit Masks */
#define XEL_MDIOADDR_REGADR_MASK  0x0000001F	/* Register Address */
#define XEL_MDIOADDR_PHYADR_MASK  0x000003E0	/* PHY Address */
#define XEL_MDIOADDR_PHYADR_SHIFT 5
#define XEL_MDIOADDR_OP_MASK	  0x00000400	/* RD/WR Operation */

/* MDIO Write Data Register Bit Masks */
#define XEL_MDIOWR_WRDATA_MASK	  0x0000FFFF	/* Data to be Written */

/* MDIO Read Data Register Bit Masks */
#define XEL_MDIORD_RDDATA_MASK	  0x0000FFFF	/* Data to be Read */

/* MDIO Control Register Bit Masks */
#define XEL_MDIOCTRL_MDIOSTS_MASK 0x00000001	/* MDIO Status Mask */
#define XEL_MDIOCTRL_MDIOEN_MASK  0x00000008	/* MDIO Enable */
#endif

/* Receive register */
#define HZHY_RECV_BASE_OFFSET	0x10200
#define HZHY_RXBADDR_OFFSET	0x200		/* The Address of Receive Buffer */
#define HZHY_RXBLEN_OFFSET	0x204		/* The Length of Each Receive Buffer */
#define HZHY_RXBSUM_OFFSET	0x206		/* The Number of Receive Buffer */
#define HZHY_RXBCTRL_OFFSET	0x207		/* The Enable Control of Receive Buffer */
#define HZHY_RXBSTAT0_OFFSET 	0x208
#define HZHY_RXBSTAT15_OFFSET	0x244

#define HZHY_RECVBUF_LENGTH		0x800	/* 2K Bytes */
#define HZHY_RECVBUF_NUMBER		8
#define HZHY_RECVBUF_TOTOLSIZE	(HZHY_RECVBUF_LENGTH * HZHY_RECVBUF_NUMBER)

/* Transmit register */
#define HZHY_SEND_BASE_OFFSET	0x10300
#define HZHY_TXBADDR_OFFSET	0x300		/* The Address of Transmit Buffer */
#define HZHY_TXBLEN_OFFSET	0x304		/* The Length of Each Transmit Buffer */
#define HZHY_TXBSUM_OFFSET	0x306		/* The Number of Transmit Buffer */
#define HZHY_TXBCTRL_OFFSET	0x307		/* The Enable Control of Transmit Buffer */
#define HZHY_TXBSTAT0_OFFSET 	0x308
#define HZHY_TXBSTAT15_OFFSET	0x344
#define RXTX_ENABLE_MASK		0x01
#define RXTX_FRAMELEN_MASK		0x0000FFFF
#define	RXTX_INVALID_MASK		0x00010000

#define HZHY_SENDBUF_LENGTH		0x800	/* 2K Bytes */
#define HZHY_SENDBUF_NUMBER		8
#define HZHY_SENDBUF_TOTOLSIZE	(HZHY_SENDBUF_LENGTH * HZHY_SENDBUF_NUMBER)


#define XEL_HEADER_OFFSET	12		/* Offset to length field */
#define XEL_HEADER_SHIFT	16		/* Shift value for length */

/* General Ethernet Definitions */
#define XEL_ARP_PACKET_SIZE		28	/* Max ARP packet size */
#define XEL_HEADER_IP_LENGTH_OFFSET	16	/* IP Length Offset */



#define TX_TIMEOUT		(60*HZ)		/* Tx timeout is 60 seconds. */
#define ALIGNMENT		4

/* BUFFER_ALIGN(adr) calculates the number of bytes to the next alignment. */
#define BUFFER_ALIGN(adr) ((ALIGNMENT - ((ulong)adr)) % ALIGNMENT)


/*
 * emac configuration register group
 */
typedef struct emac_config{
	u32 moder;
	u32 min_flr;
	u32 max_flr;
	u32 mac_addr0;
	u32 mac_addr1;
	u32 hafr0;
	u32 hafr1;
	u32 rx_cntr;
	u32 tx_cntr;
	u32 mac_rst;
	u32 phy_rst;
};

/*
 * mdio control register group
 */
typedef struct mdio_control{
	u32 cmd;
	u32 addr;
	u32 txd;
	u32 rxd;
	u32 sta;
};

/*
 * transfer control register group
 */
typedef struct transfer_control{
	u32 addr;
	u16 len;
	u8  sum;
	u8  ctrl;
	u32 stat[16];
};

/**
 * struct net_local - Our private per device data
 * @ndev:		instance of the network device
 * @tx_ping_pong:	indicates whether Tx Pong buffer is configured in HW
 * @rx_ping_pong:	indicates whether Rx Pong buffer is configured in HW
 * @next_tx_buf_to_use:	next Tx buffer to write to
 * @next_rx_buf_to_use:	next Rx buffer to read from
 * @base_addr:		base address of the Emaclite device
 * @reset_lock:		lock used for synchronization
 * @deferred_skb:	holds an skb (for transmission at a later time) when the
 *			Tx buffer is not free
 * @phy_dev:		pointer to the PHY device
 * @phy_node:		pointer to the PHY device node
 * @mii_bus:		pointer to the MII bus
 * @last_link:		last link status
 * @has_mdio:		indicates whether MDIO is included in the HW
 */
struct net_local {

	struct net_device *ndev;

	u32 next_tx_buf_to_use;
	u32 next_rx_buf_to_use;
	void __iomem *base_addr;

	unsigned char *recv_virtaddr;	// Receive buffer virtual address
	unsigned char *recv_physaddr;	// Receive buffer physical address
	unsigned char *send_virtaddr;	// Send buffer virtual address
	unsigned char *send_physaddr;	// Send buffer physical address

	spinlock_t reset_lock;
	struct sk_buff *deferred_skb;

	struct phy_device *phy_dev;
	struct device_node *phy_node;

	struct mii_bus *mii_bus;

	int last_link;
	bool has_mdio;
};

extern void antimax_dmac_cache_maint(const void *start, size_t size, int direction);
/*************************/
/* EmacLite driver calls */
/*************************/

/**
 * xemaclite_enable_interrupts - Enable the interrupts for the EmacLite device
 * @drvdata:	Pointer to the Emaclite device private data
 *
 * This function enables the Tx and Rx interrupts for the Emaclite device along
 * with the Global Interrupt Enable.
 */
static void hzhyemaclite_enable_interrupts(struct net_local *drvdata)
{
	struct transfer_control *recv_base = (struct transfer_control *)(drvdata->base_addr + HZHY_RECV_BASE_OFFSET);
	struct transfer_control *send_base = (struct transfer_control *)(drvdata->base_addr + HZHY_SEND_BASE_OFFSET);

	/* Enable rx and tx function */
	recv_base->ctrl |= RXTX_ENABLE_MASK;
	send_base->ctrl |= RXTX_ENABLE_MASK;

	return;
}

/**
 * xemaclite_disable_interrupts - Disable the interrupts for the EmacLite device
 * @drvdata:	Pointer to the Emaclite device private data
 *
 * This function disables the Tx and Rx interrupts for the Emaclite device,
 * along with the Global Interrupt Enable.
 */
static void hzhyemaclite_disable_interrupts(struct net_local *drvdata)
{
	struct transfer_control *recv_base = (struct transfer_control *)(drvdata->base_addr + HZHY_RECV_BASE_OFFSET);
	struct transfer_control *send_base = (struct transfer_control *)(drvdata->base_addr + HZHY_SEND_BASE_OFFSET);

	/* Enable rx and tx function */
	recv_base->ctrl &= ~RXTX_ENABLE_MASK;
	send_base->ctrl &= ~RXTX_ENABLE_MASK;

	return;
}

/**
 * xemaclite_aligned_write - Write from 16-bit aligned to 32-bit aligned address
 * @src_ptr:	Void pointer to the 16-bit aligned source address
 * @dest_ptr:	Pointer to the 32-bit aligned destination address
 * @length:	Number bytes to write from source to destination
 *
 * This function writes data from a 16-bit aligned buffer to a 32-bit aligned
 * address in the EmacLite device.
 */

static void hzhyemaclite_aligned_write(void *src_ptr, u32 *dest_ptr,
				    unsigned length)
{
	u32 align_buffer;
	u32 *to_u32_ptr;
	u16 *from_u16_ptr, *to_u16_ptr;

	to_u32_ptr = dest_ptr;
	from_u16_ptr = src_ptr;
	align_buffer = 0;

	for (; length > 3; length -= 4) { //for (; length > 3; length -= 4)
		to_u16_ptr = (u16 *)&align_buffer;
		*to_u16_ptr++ = *from_u16_ptr++;
		*to_u16_ptr++ = *from_u16_ptr++;

		/* This barrier resolves occasional issues seen around
		 * cases where the data is not properly flushed out
		 * from the processor store buffers to the destination
		 * memory locations.
		 */
		wmb();

		/* Output a word */
		*to_u32_ptr++ = align_buffer;
	}
	if (length) {
		u8 *from_u8_ptr, *to_u8_ptr;

		/* Set up to output the remaining data */
		align_buffer = 0;
		to_u8_ptr = (u8 *) &align_buffer;
		from_u8_ptr = (u8 *) from_u16_ptr;

		/* Output the remaining data */
		for (; length > 0; length--)
			*to_u8_ptr++ = *from_u8_ptr++;

		/* This barrier resolves occasional issues seen around
		 * cases where the data is not properly flushed out
		 * from the processor store buffers to the destination
		 * memory locations.
		 */
		wmb();
		*to_u32_ptr = align_buffer;
	}
}

/**
 * xemaclite_aligned_read - Read from 32-bit aligned to 16-bit aligned buffer
 * @src_ptr:	Pointer to the 32-bit aligned source address
 * @dest_ptr:	Pointer to the 16-bit aligned destination address
 * @length:	Number bytes to read from source to destination
 *
 * This function reads data from a 32-bit aligned address in the EmacLite device
 * to a 16-bit aligned buffer.
 */
static void hzhyemaclite_aligned_read(u32 *src_ptr, u8 *dest_ptr,
				   unsigned length)
{
	u16 *to_u16_ptr, *from_u16_ptr;
	u32 *from_u32_ptr;
	u32 align_buffer;

	from_u32_ptr = src_ptr;
	to_u16_ptr = (u16 *) dest_ptr;

	for (; length > 3; length -= 4) { //for (; length > 3; length -= 4)
		/* Copy each word into the temporary buffer */
		align_buffer = *from_u32_ptr++;
		from_u16_ptr = (u16 *)&align_buffer;

		/* Read data from source */
		*to_u16_ptr++ = *from_u16_ptr++;
		*to_u16_ptr++ = *from_u16_ptr++;
	}

	if (length) {
		u8 *to_u8_ptr, *from_u8_ptr;

		/* Set up to read the remaining data */
		to_u8_ptr = (u8 *) to_u16_ptr;
		align_buffer = *from_u32_ptr++;
		from_u8_ptr = (u8 *) &align_buffer;

		/* Read the remaining data */
		for (; length > 0; length--)
			*to_u8_ptr = *from_u8_ptr;
	}
}

/**
 * hzhyemaclite_send_data - Send an Ethernet frame
 * @drvdata:	Pointer to the Emaclite device private data
 * @data:	Pointer to the data to be sent
 * @byte_count:	Total frame size, including header
 *
 * This function checks if the Tx buffer of the Emaclite device is free to send
 * data. If so, it fills the Tx buffer with data for transmission. Otherwise, it
 * returns an error.
 *
 * Return:	0 upon success or -1 if the buffer(s) are full.
 *
 * Note:	The maximum Tx packet size can not be more than Ethernet header
 *		(14 Bytes) + Maximum MTU (1500 bytes). This is excluding FCS.
 */
u32 sendCount = 0;
static int hzhyemaclite_send_data(struct net_local *drvdata, u8 *data,
			       unsigned int byte_count)
{
	u32 reg_data,i;
	struct transfer_control *send_base = (struct transfer_control *)(drvdata->base_addr + HZHY_SEND_BASE_OFFSET);
	void __iomem *addr;
	u32 nextBuf;


	

	//printk("Howard: %s--%d--%s\n", __FILE__, __LINE__, __FUNCTION__);

	/* Determine the expected Tx buffer address */
	//addr = drvdata->send_virtaddr + send_base->len * drvdata->next_tx_buf_to_use;
	//printk("==Howard: send_virtaddr=0x%x, send_base->len=0x%x, next_tx_buf_to_use=%d, addr=0x%x\n", drvdata->send_virtaddr, send_base->len,\
	//										     drvdata->next_tx_buf_to_use, addr);
	/* If the length is too large, truncate it */
	if (byte_count > ETH_FRAME_LEN)
		byte_count = ETH_FRAME_LEN;

	/* Check if the expected buffer is available */
#if 1
	while((send_base->stat[drvdata->next_tx_buf_to_use] & RXTX_INVALID_MASK))
	{
		drvdata->next_tx_buf_to_use = (drvdata->next_tx_buf_to_use+1)%HZHY_SENDBUF_NUMBER;
	}

	addr = drvdata->send_virtaddr + send_base->len * drvdata->next_tx_buf_to_use;
#else
	while((send_base->stat[drvdata->next_tx_buf_to_use] & RXTX_INVALID_MASK))
	{
		//udelay(10);
	}
#endif
   /* printk("**Send %d buf[%d]:\n", sendCount++, drvdata->next_tx_buf_to_use);
    for(i = 0; i < byte_count; i++)
    {
    	printk(" 0x%x", data[i]);
    }
    printk("\n");*/

	/* Write the frame to the buffer */
	hzhyemaclite_aligned_write(data, (u32 __force *) addr, byte_count);

	/* Update the Tx Status Register to indicate that there is a
	 * frame to send. Set the XEL_TSR_XMIT_ACTIVE_MASK flag which
	 * is used by the interrupt handler to check whether a frame
	 * has been transmitted */
	reg_data = byte_count | RXTX_INVALID_MASK;
	send_base->stat[drvdata->next_tx_buf_to_use] = reg_data;
	drvdata->next_tx_buf_to_use = (drvdata->next_tx_buf_to_use + 1) % send_base->sum;


	return 0;
}

/**
 * hzhyemaclite_recv_data - Receive a frame
 * @drvdata:	Pointer to the Emaclite device private data
 * @data:	Address where the data is to be received
 *
 * This function is intended to be called from the interrupt context or
 * with a wrapper which waits for the receive frame to be available.
 *
 * Return:	Total number of bytes received
 */
u32 revCount = 0;
static u16 hzhyemaclite_recv_data(struct net_local *drvdata, u8 *data)
{
	void __iomem *addr;
	struct transfer_control *recv_base = (struct transfer_control *)(drvdata->base_addr + HZHY_RECV_BASE_OFFSET);
	u16 length,i;
	u32 reg_data;
	u8 testBuf[1536];

	//printk("*****Howard: %s--%d--%s\n", __FILE__, __LINE__, __FUNCTION__);

	/* Determine the expected buffer address */
#if 0
	addr = drvdata->recv_virtaddr + recv_base->len * drvdata->next_rx_buf_to_use;

	/* Verify which buffer has valid data */
	if(!(recv_base->stat[drvdata->next_rx_buf_to_use] & RXTX_INVALID_MASK))
	{
		//return 0;	/* No data was available */
		//udelay(10);
	}
#else
	while(!(recv_base->stat[drvdata->next_rx_buf_to_use] & RXTX_INVALID_MASK))
	{
		drvdata->next_rx_buf_to_use = (drvdata->next_rx_buf_to_use + 1) % recv_base->sum;
	}

	addr = drvdata->recv_virtaddr + recv_base->len * drvdata->next_rx_buf_to_use;
#endif
	/* Get the length of the ethernet frame that arrived */
	length = recv_base->stat[drvdata->next_rx_buf_to_use] & RXTX_FRAMELEN_MASK;
	//printk("***Howard: recv_virtaddr=0x%x, length=%d, next_rx_buf_to_use=%d, addr=0x%x\n", drvdata->recv_virtaddr, length,\
													     drvdata->next_rx_buf_to_use, addr);

	/* Read from the EmacLite device */
    hzhyemaclite_aligned_read((u32 __force *) addr, data, length);

  /* printk("Rev %d buf[%d]:\n", revCount++, drvdata->next_rx_buf_to_use);
    for(i = 0; i < length; i++)
    {
    	printk(" 0x%x", data[i]);
    }
    printk("\n");*/

	/* Acknowledge the frame */
	recv_base->stat[drvdata->next_rx_buf_to_use] &= ~RXTX_INVALID_MASK;
	drvdata->next_rx_buf_to_use = (drvdata->next_rx_buf_to_use + 1) % recv_base->sum;

	return length;
}

/**
 * hzhyemaclite_update_address - Update the MAC address in the device
 * @drvdata:	Pointer to the Emaclite device private data
 * @address_ptr:Pointer to the MAC address (MAC address is a 48-bit value)
 *
 * Tx must be idle and Rx should be idle for deterministic results.
 * It is recommended that this function should be called after the
 * initialization and before transmission of any packets from the device.
 * The MAC address can be programmed using any of the two transmit
 * buffers (if configured).
 */
static void hzhyemaclite_update_address(struct net_local *drvdata,
				     u8 *address_ptr)
{
	void __iomem *addr;
	struct emac_config *emac_base = (struct emac_config *)(drvdata->base_addr + HZHY_EMAC_BASE_OFFSET);
	u32 reg_data[2];

	//printk("Howard: %s--%d--%s\n", __FILE__, __LINE__, __FUNCTION__);
	//printk("Howard: drvdata->base_addr=0x%x, emac_base=0x%x\n", drvdata->base_addr, emac_base);
	//printk("Howard: mac address is: %x %x %x %x %x %x\n", address_ptr[0], address_ptr[1], address_ptr[2],
	//						      address_ptr[3], address_ptr[4], address_ptr[5]);
	/* Determine the expected Tx buffer address */
	addr = emac_base->mac_addr0;

	/* Write into u32 temporary address */
	//hzhyemaclite_aligned_write(address_ptr, reg_data, ETH_ALEN);

	/* Update the MAC address in the EmacLite */
	//emac_base->mac_addr0 = address_ptr[0] | address_ptr[1]<<8 | address_ptr[2]<<16 | address_ptr[3]<<24;	//reg_data[0];
	//emac_base->mac_addr1 = (address_ptr[4] | address_ptr[5]<<8) & EMAC_MACHIGHT_MASK;
	emac_base->mac_addr0 = address_ptr[5] | address_ptr[4]<<8 | address_ptr[3]<<16 | address_ptr[2]<<24;	//reg_data[0];
	emac_base->mac_addr1 = (address_ptr[1] | address_ptr[0]<<8) & EMAC_MACHIGHT_MASK;

	//printk("Howard: mac_addr0=0x%x, mac_addr1=0x%x\n", emac_base->mac_addr0, emac_base->mac_addr1);

}

/**
 * hzhyemaclite_set_mac_address - Set the MAC address for this device
 * @dev:	Pointer to the network device instance
 * @addr:	Void pointer to the sockaddr structure
 *
 * This function copies the HW address from the sockaddr strucutre to the
 * net_device structure and updates the address in HW.
 *
 * Return:	Error if the net device is busy or 0 if the addr is set
 *		successfully
 */
static int hzhyemaclite_set_mac_address(struct net_device *dev, void *address)
{
	struct net_local *lp = netdev_priv(dev);
	struct sockaddr *addr = address;

	printk("Howard: %s--%d--%s\n", __FILE__, __LINE__, __FUNCTION__);

	if (netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
	hzhyemaclite_update_address(lp, dev->dev_addr);
	return 0;
}

/**
 * hzhyemaclite_tx_timeout - Callback for Tx Timeout
 * @dev:	Pointer to the network device
 *
 * This function is called when Tx time out occurs for Emaclite device.
 */
static void hzhyemaclite_tx_timeout(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	unsigned long flags;

	printk(&lp->ndev->dev, "Exceeded transmit timeout of %lu ms\n",
		TX_TIMEOUT * 1000UL / HZ);

	dev->stats.tx_errors++;

	/* Reset the device */
	spin_lock_irqsave(&lp->reset_lock, flags);

	/* Shouldn't really be necessary, but shouldn't hurt */
	netif_stop_queue(dev);

	hzhyemaclite_disable_interrupts(lp);
	hzhyemaclite_enable_interrupts(lp);

	if (lp->deferred_skb) {
		dev_kfree_skb(lp->deferred_skb);
		lp->deferred_skb = NULL;
		dev->stats.tx_errors++;
	}

	/* To exclude tx timeout */
	dev->trans_start = jiffies; /* prevent tx timeout */

	/* We're all ready to go. Start the queue */
	netif_wake_queue(dev);
	spin_unlock_irqrestore(&lp->reset_lock, flags);
}

/*MAC CHECK INIT*/
static int emac_set_check(unsigned char *baseAddr)
{
	iowrite32(MNT_CTR,baseAddr + HZHY_EMAC_BASE_OFFSET +FILTER_MNT_CTR);
	iowrite32(MNT_GAP_TIME,baseAddr + HZHY_EMAC_BASE_OFFSET +FILTER_MNT_GAP);
	iowrite32(MNT_THR1_DOOR1,baseAddr + HZHY_EMAC_BASE_OFFSET +FILTER_MNT_THR1);
	iowrite32(MNT_THR2_DOOR2,baseAddr + HZHY_EMAC_BASE_OFFSET +FILTER_MNT_THR2);
	iowrite32(MNT_FTHR1_DOOR1,baseAddr + HZHY_EMAC_BASE_OFFSET +FILTER_MNT_FTHR1);
	iowrite32(MNT_FTHR2_DOOR2,baseAddr + HZHY_EMAC_BASE_OFFSET +FILTER_MNT_FTHR2);
	return 0;
	
}


/**********************/
/* Interrupt Handlers */
/**********************/

/**
 * hzhyemaclite_tx_handler - Interrupt handler for frames sent
 * @dev:	Pointer to the network device
 *
 * This function updates the number of packets transmitted and handles the
 * deferred skb, if there is one.
 */
static void hzhyemaclite_tx_handler(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);

	dev->stats.tx_packets++;
	if (lp->deferred_skb) {
		if (hzhyemaclite_send_data(lp,
					(u8 *) lp->deferred_skb->data,
					lp->deferred_skb->len) != 0)
			return;
		else {
			dev->stats.tx_bytes += lp->deferred_skb->len;
			dev_kfree_skb_irq(lp->deferred_skb);
			lp->deferred_skb = NULL;
			dev->trans_start = jiffies; /* prevent tx timeout */
			netif_wake_queue(dev);
		}
	}
}

/**
 * hzhyemaclite_rx_handler- Interrupt handler for frames received
 * @dev:	Pointer to the network device
 *
 * This function allocates memory for a socket buffer, fills it with data
 * received and hands it over to the TCP/IP stack.
 */
static void hzhyemaclite_rx_handler(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	struct sk_buff *skb;
	unsigned int align;
	u32 len;

	len = ETH_FRAME_LEN + ETH_FCS_LEN;
	skb = netdev_alloc_skb(dev, len + ALIGNMENT);
	if (!skb) {
		/* Couldn't get memory. */
		dev->stats.rx_dropped++;
		dev_err(&lp->ndev->dev, "Could not allocate receive buffer\n");
		return;
	}

	/*
	 * A new skb should have the data halfword aligned, but this code is
	 * here just in case that isn't true. Calculate how many
	 * bytes we should reserve to get the data to start on a word
	 * boundary */
	align = BUFFER_ALIGN(skb->data);
	if (align)
		skb_reserve(skb, align);

	skb_reserve(skb, 2);

	len = hzhyemaclite_recv_data(lp, (u8 *) skb->data);
	if (!len) {
		dev->stats.rx_errors++;
		dev_kfree_skb_irq(skb);
		return;
	}

	skb_put(skb, len);	/* Tell the skb how much data we got */

	skb->protocol = eth_type_trans(skb, dev);
	skb_checksum_none_assert(skb);

	dev->stats.rx_packets++;
	dev->stats.rx_bytes += len;

	if (!skb_defer_rx_timestamp(skb))
		netif_rx(skb);	/* Send the packet upstream */
}

/**
 * hzhyemaclite_interrupt - Interrupt handler for this driver
 * @irq:	Irq of the Emaclite device
 * @dev_id:	Void pointer to the network device instance used as callback
 *		reference
 *
 * This function handles the Tx and Rx interrupts of the EmacLite device.
 */
static irqreturn_t hzhyemaclite_interrupt(int irq, void *dev_id)
{
	bool tx_complete = false;
	struct net_device *dev = dev_id;
	struct net_local *lp = netdev_priv(dev);
	void __iomem *base_addr = lp->base_addr;
	struct transfer_control *recv_base = (struct transfer_control *)(lp->base_addr + HZHY_RECV_BASE_OFFSET);
	struct transfer_control *send_base = (struct transfer_control *)(lp->base_addr + HZHY_SEND_BASE_OFFSET);
	u32 tx_status;
	
	//printk("===Howard: enter isr\n");

	/* Check if there is Rx Data available */
	while(recv_base->stat[lp->next_rx_buf_to_use] & RXTX_INVALID_MASK)
	{
		hzhyemaclite_rx_handler(dev);
	}

	/* Check if the Transmission for the previous send is completed */

	/* If there was a Tx interrupt, call the Tx Handler */
	if (tx_complete != 0)
		hzhyemaclite_tx_handler(dev);

	return IRQ_HANDLED;
}

/**********************/
/* MDIO Bus functions */
/**********************/
#if 0
/**
 * xemaclite_mdio_wait - Wait for the MDIO to be ready to use
 * @lp:		Pointer to the Emaclite device private data
 *
 * This function waits till the device is ready to accept a new MDIO
 * request.
 *
 * Return:	0 for success or ETIMEDOUT for a timeout
 */

static int hzhyemaclite_mdio_wait(struct net_local *lp)
{
    unsigned long end = jiffies + 2;
	struct mdio_control *mdio_base = (struct mdio_control *)(lp->base_addr + HZHY_MDIO_BASE_OFFSET);

	/* wait for the MDIO interface to not be busy or timeout
	   after some time.
	*/
	while (mdio_base->sta & MDIO_STABUSY_MASK) {
		if (time_before_eq(end, jiffies)) {
			WARN_ON(1);
			return -ETIMEDOUT;
		}
		msleep(1);
	}
	return 0;
}

/**
 * hzhyemaclite_mdio_read - Read from a given MII management register
 * @bus:	the mii_bus struct
 * @phy_id:	the phy address
 * @reg:	register number to read from
 *
 * This function waits till the device is ready to accept a new MDIO
 * request and then writes the phy address to the MDIO Address register
 * and reads data from MDIO Read Data register, when its available.
 *
 * Return:	Value read from the MII management register
 */
static int hzhyemaclite_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	struct net_local *lp = bus->priv;
	struct mdio_control *mdio_base = (struct mdio_control *)(lp->base_addr + HZHY_MDIO_BASE_OFFSET);
	u32 ctrl_reg;
	u32 rc;

	/* Waiting for mdio bus get ready */
	if (hzhyemaclite_mdio_wait(lp))
		return -ETIMEDOUT;
	printk("******[%s:%s]%d phy_id=%d reg=%d*********\n",__FILE__,__func__,__LINE__, phy_id,reg);
	/* Write the PHY address, register number and set the OP bit in the
	 * MDIO Address register. Set the Status bit in the MDIO Control
	 * register to start a MDIO read transaction.
	 */
	mdio_base->addr = (phy_id & MDIO_PHYADDR_MASK) | ((reg<<8) & MDIO_REGADDR_MASK);
	mdio_base->cmd |= MDIO_CMDREAD_MASK;
	printk("******[%s:%s]%d mdio_base->addr=0x%x*********\n",__FILE__,__func__,__LINE__,mdio_base->addr);

	/* Waiting for read completed */
	if (hzhyemaclite_mdio_wait(lp))
		return -ETIMEDOUT;

	printk("******[%s:%s]%d*********\n",__FILE__,__func__,__LINE__);
	/* Get the specified regiter value */
	rc = mdio_base->rxd & 0xFFFF;

	printk("Howard: xemaclite_mdio_read(phy_id=%i, reg=%x) == %x\n", phy_id, reg, rc);

	return rc;
}

/**
 * hzhyemaclite_mdio_write - Write to a given MII management register
 * @bus:	the mii_bus struct
 * @phy_id:	the phy address
 * @reg:	register number to write to
 * @val:	value to write to the register number specified by reg
 *
 * This function waits till the device is ready to accept a new MDIO
 * request and then writes the val to the MDIO Write Data register.
 */
static int hzhyemaclite_mdio_write(struct mii_bus *bus, int phy_id, int reg,
				u16 val)
{
	struct net_local *lp = bus->priv;
	struct mdio_control *mdio_base = (struct mdio_control *)(lp->base_addr + HZHY_MDIO_BASE_OFFSET);
	u32 ctrl_reg;

	printk("Howard: xemaclite_mdio_write(phy_id=%i, reg=%x, val=%x)\n", phy_id, reg, val);

	/* Waiting for mdio bus get ready */
	if (hzhyemaclite_mdio_wait(lp))
		return -ETIMEDOUT;

	/* Write the PHY address, register number and clear the OP bit in the
	 * MDIO Address register and then write the value into the MDIO Write
	 * Data register. Finally, set the Status bit in the MDIO Control
	 * register to start a MDIO write transaction.
	 */
	mdio_base->addr = (phy_id & MDIO_PHYADDR_MASK) | ((reg<<8) & MDIO_REGADDR_MASK);
	mdio_base->txd  = val & 0xFFFF;
	mdio_base->cmd |= MDIO_CMDWRITE_MASK;

	/* Waiting for write completed */
	/*if (hzhyemaclite_mdio_wait(lp))
		return -ETIMEDOUT;
	*/
	return 0;
}

/**
 * hzhyemaclite_mdio_setup - Register mii_bus for the Emaclite device
 * @lp:		Pointer to the Emaclite device private data
 * @ofdev:	Pointer to OF device structure
 *
 * This function enables MDIO bus in the Emaclite device and registers a
 * mii_bus.
 *
 * Return:	0 upon success or a negative error upon failure
 */
static int hzhyemaclite_mdio_setup(struct net_local *lp, struct device *dev)
{
	struct mii_bus *bus;
	int rc;
	struct resource res;
	struct device_node *np = of_get_parent(lp->phy_node);
	struct device_node *npp;

	/* Don't register the MDIO bus if the phy_node or its parent node
	 * can't be found.
	 */
	if (!np) {
		dev_err(dev, "Failed to register mdio bus.\n");
		return -ENODEV;
	}
	npp = of_get_parent(np);

	of_address_to_resource(npp, 0, &res);
	if (lp->ndev->mem_start != res.start) {
		struct phy_device *phydev;
		phydev = of_phy_find_device(lp->phy_node);
		if (!phydev)
			dev_info(dev,
				 "MDIO of the phy is not registered yet\n");
		else
			put_device(&phydev->mdio.dev);
		return 0;
	}

	/* Enable the MDIO bus by asserting the enable bit in MDIO Control
	 * register.
	 */


	printk("Howard: mdiobus_alloc\n");

	bus = mdiobus_alloc();
	if (!bus) {
		dev_err(dev, "Failed to allocate mdiobus\n");
		return -ENOMEM;
	}

	snprintf(bus->id, MII_BUS_ID_SIZE, "%.8llx", (unsigned long long)res.start);
	bus->priv = lp;
	bus->name = "Hzhy Emaclite MDIO";
	bus->read = hzhyemaclite_mdio_read;
	bus->write = hzhyemaclite_mdio_write;
	bus->parent = dev;

	lp->mii_bus = bus;
	
	printk("Howard: of_mdiobus_register\n");

	rc = of_mdiobus_register(bus, np);
	if (rc) {
		dev_err(dev, "Failed to register mdio bus.\n");
		goto err_register;
	}

	return 0;

err_register:
	mdiobus_free(bus);
	return rc;
}
#endif

/**********************/
/* MDIO Bus functions */
/**********************/

//chn add  mdio mux
static struct net_local *lp_eth0;

/**
 * xemaclite_mdio_wait - Wait for the MDIO to be ready to use
 * @lp:		Pointer to the Emaclite device private data
 *
 * This function waits till the device is ready to accept a new MDIO
 * request.
 *
 * Return:	0 for success or ETIMEDOUT for a timeout
 */

static int hzhyemaclite_mdio_wait(struct net_local *lp)
{
	unsigned long end = jiffies + 50;//2

	/* wait for the MDIO interface to not be busy or timeout
	   after some time.
	*/
	//chn modify
	//while (__raw_readl(lp->base_addr + XEL_MDIOCTRL_OFFSET) &
	while (__raw_readl(lp_eth0->base_addr + XEL_MDIOCTRL_OFFSET) &
			XEL_MDIOCTRL_MDIOSTS_MASK) {
		if (time_before_eq(end, jiffies)) {
			WARN_ON(1);
			return -ETIMEDOUT;
		}
		msleep(1);
	}
	return 0;
}

/**
 * xemaclite_mdio_read - Read from a given MII management register
 * @bus:	the mii_bus struct
 * @phy_id:	the phy address
 * @reg:	register number to read from
 *
 * This function waits till the device is ready to accept a new MDIO
 * request and then writes the phy address to the MDIO Address register
 * and reads data from MDIO Read Data register, when its available.
 *
 * Return:	Value read from the MII management register
 */
static int hzhyemaclite_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	//struct net_local *lp = bus->priv;
	struct net_local *lp = lp_eth0;//chn modify
	u32 ctrl_reg;
	u32 rc;

	//printk("<1>");
	udelay(1000);
	if (hzhyemaclite_mdio_wait(lp))
	{
		printk("sun add %s 1 timeout %d\n", __FUNCTION__, -ETIMEDOUT);
		return -ETIMEDOUT;
	}
	/* Write the PHY address, register number and set the OP bit in the
	 * MDIO Address register. Set the Status bit in the MDIO Control
	 * register to start a MDIO read transaction.
	 */
	//printk("<2>");
	udelay(1000);
	ctrl_reg = __raw_readl(lp->base_addr + XEL_MDIOCTRL_OFFSET);
	//printk("<3>");
	udelay(1000);
	__raw_writel(XEL_MDIOADDR_OP_MASK |
		     ((phy_id << XEL_MDIOADDR_PHYADR_SHIFT) | reg),
		     lp->base_addr + XEL_MDIOADDR_OFFSET);
	//printk("<4>");
	udelay(1000);
	//printk("==XEL_MDIOADDR_OFFSET=0x%x====\n", lp->base_addr + XEL_MDIOADDR_OFFSET);
	__raw_writel(ctrl_reg | XEL_MDIOCTRL_MDIOSTS_MASK,
		     lp->base_addr + XEL_MDIOCTRL_OFFSET);
	//printk("<5>");
	udelay(1000);
	if (hzhyemaclite_mdio_wait(lp))
	{
		printk("sun add %s 2 timeout %d\n", __FUNCTION__, -ETIMEDOUT);
		return -ETIMEDOUT;
	}
	//printk("<6>\n");
	udelay(1000);
	rc = __raw_readl(lp->base_addr + XEL_MDIORD_OFFSET);

	dev_dbg(&lp->ndev->dev,
		"xemaclite_mdio_read(phy_id=%i, reg=%x) == %x\n",
		phy_id, reg, rc);

	//printk("====xemaclite_mdio_read(phy_id=%i, reg=%x) == %x lp->base_addr=0x%x\n", phy_id, reg, rc, lp->base_addr);

	return rc;
}

/**
 * xemaclite_mdio_write - Write to a given MII management register
 * @bus:	the mii_bus struct
 * @phy_id:	the phy address
 * @reg:	register number to write to
 * @val:	value to write to the register number specified by reg
 *
 * This function waits till the device is ready to accept a new MDIO
 * request and then writes the val to the MDIO Write Data register.
 */
static int hzhyemaclite_mdio_write(struct mii_bus *bus, int phy_id, int reg,
				u16 val)
{
	//struct net_local *lp = bus->priv;
	struct net_local *lp = lp_eth0; //chn modify
	u32 ctrl_reg;

	dev_dbg(&lp->ndev->dev,
		"xemaclite_mdio_write(phy_id=%i, reg=%x, val=%x)\n",
		phy_id, reg, val);
	//printk("====xemaclite_mdio_write(phy_id=%i, reg=%x, val=%x)\n",phy_id, reg, val);

	if (hzhyemaclite_mdio_wait(lp))
		return -ETIMEDOUT;

	/* Write the PHY address, register number and clear the OP bit in the
	 * MDIO Address register and then write the value into the MDIO Write
	 * Data register. Finally, set the Status bit in the MDIO Control
	 * register to start a MDIO write transaction.
	 */
	ctrl_reg = __raw_readl(lp->base_addr + XEL_MDIOCTRL_OFFSET);
	__raw_writel(~XEL_MDIOADDR_OP_MASK &
		     ((phy_id << XEL_MDIOADDR_PHYADR_SHIFT) | reg),
		     lp->base_addr + XEL_MDIOADDR_OFFSET);
	__raw_writel(val, lp->base_addr + XEL_MDIOWR_OFFSET);
	__raw_writel(ctrl_reg | XEL_MDIOCTRL_MDIOSTS_MASK,
		     lp->base_addr + XEL_MDIOCTRL_OFFSET);

	return 0;
}

/**
 * xemaclite_mdio_setup - Register mii_bus for the Emaclite device
 * @lp:		Pointer to the Emaclite device private data
 * @ofdev:	Pointer to OF device structure
 *
 * This function enables MDIO bus in the Emaclite device and registers a
 * mii_bus.
 *
 * Return:	0 upon success or a negative error upon failure
 */
static int hzhyemaclite_mdio_setup(struct net_local *lp, struct device *dev)
{
	struct mii_bus *bus;
	int rc;
	struct resource res;
	struct device_node *np = of_get_parent(lp->phy_node);
	struct device_node *npp;

	printk("sun axi mdio: enter %s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	/* Don't register the MDIO bus if the phy_node or its parent node
	 * can't be found.
	 */
	if (!np) {
		
		dev_err(dev, "Failed to register mdio bus.\n");
		return -ENODEV;
	}
	
	npp = of_get_parent(np);

	of_address_to_resource(npp, 0, &res);

	if (lp->ndev->mem_start != res.start) {
	
		struct phy_device *phydev;
		phydev = of_phy_find_device(lp->phy_node);
		if (!phydev)
			dev_info(dev,
				 "MDIO of the phy is not registered yet\n");
		else
			put_device(&phydev->mdio.dev);
		return 0;
	}

	/* Enable the MDIO bus by asserting the enable bit in MDIO Control
	 * register.
	 */
	
	__raw_writel(XEL_MDIOCTRL_MDIOEN_MASK,
		     lp->base_addr + XEL_MDIOCTRL_OFFSET);
	
//	__raw_writel(96, lp->base_addr + 0x00000500);

	bus = mdiobus_alloc();
	if (!bus) {
		printk("3\n");
		dev_err(dev, "Failed to allocate mdiobus\n");
		return -ENOMEM;
	}

	//chn add  mdio mux
	if(NULL == lp_eth0)
	{
		lp_eth0 = lp;
		printk(KERN_WARNING "lp_eth0 : %p\n", lp_eth0);
	}

	snprintf(bus->id, MII_BUS_ID_SIZE, "%.8llx",
		 (unsigned long long)res.start);
	bus->priv = lp;
	bus->name = "Xilinx Emaclite MDIO";
	bus->read = hzhyemaclite_mdio_read;
	bus->write = hzhyemaclite_mdio_write;
	bus->parent = dev;

	lp->mii_bus = bus;

	rc = of_mdiobus_register(bus, np);
	if (rc) {
		printk("4\n");
		dev_err(dev, "Failed to register mdio bus.\n");
		goto err_register;
	}

	printk("sun axi mdio: leave %s %s %d\n", __FILE__, __FUNCTION__, __LINE__);

	return 0;

err_register:
	mdiobus_free(bus);
	return rc;
}

/**
 * hzhyemaclite_adjust_link - Link state callback for the Emaclite device
 * @ndev: pointer to net_device struct
 *
 * There's nothing in the Emaclite device to be configured when the link
 * state changes. We just print the status.
 */
static int hzhyemaclite_open(struct net_device *dev);
static int hzhyemaclite_close(struct net_device *dev);
static void hzhyemaclite_adjust_link(struct net_device *ndev)
{
#if 0
	struct net_local *lp = netdev_priv(ndev);
	struct phy_device *phy = lp->phy_dev;
	struct mdio_control *mdio_base = (struct mdio_control *)(lp->base_addr + HZHY_MDIO_BASE_OFFSET);
	int link_state;
	
	printk("Howard: %s--%d--%s\n", __FILE__, __LINE__, __FUNCTION__);

	/* hash together the state values to decide if something has changed */
	// Get the current link status
	link_state = mdio_base->sta & MDIO_STALINK_MASK;
	if (lp->last_link != link_state) {
		lp->last_link = link_state;
		phy_print_status(phy);
	}
#else
	struct net_local *lp = netdev_priv(ndev);
	struct phy_device *phy = lp->phy_dev;
	int link_state;
	struct emac_config *emac_base;
	struct transfer_control *recv_base;
	struct transfer_control *send_base;
	int retval;
	u32 bmcr;
//	printk("sun add: enter %s %s %d\n", __FILE__, __FUNCTION__, __LINE__);//1217
	/* hash together the state values to decide if something has changed */
	link_state = phy->speed | (phy->duplex << 1) | phy->link;

	if (lp->last_link != link_state) {
		lp->last_link = link_state;
		phy_print_status(phy);

//		printk("sun add %s %d\n", __FUNCTION__, __LINE__);//1217
		/*
		 *  add by waz again init phy
		 *  是为了解决 多次拔插网线会出现网口灯灭的情况，且量取phy芯片信号时，RXER信号为高，说明接收出错
		 */
		if (phy->link) {
			//add by sun phy led init
			bmcr = phy_read(lp->phy_dev, MII_PHYADDR);
			//printk("sun add if %s %s 0x%04x\n", __FILE__, __FUNCTION__, bmcr);
			bmcr &= (~0x20);
			phy_write(lp->phy_dev, MII_PHYADDR, bmcr);
			//printk("sun add if %s %s 0x%04x\n", __FILE__, __FUNCTION__, bmcr);
			udelay(100);
		}
		else{
			//hzhyemaclite_close(ndev);
			emac_base = (struct emac_config *)(lp->base_addr + HZHY_EMAC_BASE_OFFSET);
			emac_base->moder = 0;
			emac_base->mac_rst = 1;
			emac_base->phy_rst = 1;
			udelay(100); // phy reset need 1us at least

			lp->next_tx_buf_to_use = 0x0;
			lp->next_rx_buf_to_use = 0x0;

			//printk("&&&&&&&&[%s:%s]%d&&&&&&&&&\n",__FILE__,__func__,__LINE__);
			netif_stop_queue(ndev);
			hzhyemaclite_disable_interrupts(lp);
			free_irq(ndev->irq, ndev);
			//printk("&&&&&&&&[%s:%s]%d&&&&&&&&&\n",__FILE__,__func__,__LINE__);

			//printk("****open***\n");
			//hzhyemaclite_open(ndev);
			/* Just to be safe, stop the device first */
			hzhyemaclite_disable_interrupts(lp);
			if (lp->phy_node) {
				//printk("Howard: of_phy_connect\n");

				/* EmacLite doesn't support giga-bit speeds */
				lp->phy_dev->supported &= (PHY_BASIC_FEATURES);
				lp->phy_dev->advertising = lp->phy_dev->supported;

#if  1
				/* Don't advertise 1000BASE-T Full/Half duplex speeds */
				//phy_write(lp->phy_dev, MII_CTRL1000, 0);  zhanghuan gai

				/* Advertise only 10 and 100mbps full/half duplex speeds */
				phy_write(lp->phy_dev, MII_ADVERTISE, ADVERTISE_ALL | ADVERTISE_CSMA);
#if 1
				/* Restart auto negotiation */
				bmcr = phy_read(lp->phy_dev, MII_BMCR);
				//bmcr |= (BMCR_SPEED100 | BMCR_FULLDPLX);
				//bmcr &= ~(1 << 12);
				bmcr |= (1 << 12);
				//bmcr |= (BMCR_ANENABLE | BMCR_ANRESTART);
				phy_write(lp->phy_dev, MII_BMCR, bmcr);
#endif
				//phy_start(lp->phy_dev);
				bmcr = phy_read(lp->phy_dev, MII_PHYADDR);
				//printk("sun add else %s %s 0x%04x\n", __FILE__, __FUNCTION__, bmcr);
				bmcr &= (~0x20);
				phy_write(lp->phy_dev, MII_PHYADDR, bmcr);
				//printk("sun add else %s %s 0x%04x\n", __FILE__, __FUNCTION__, bmcr);
#endif

			}
			//printk("Howard: hzhyemaclite_update_address\n");

			/* Set the MAC address each time opened */
			hzhyemaclite_update_address(lp, ndev->dev_addr);

			//Enable receive
			//emac_base = (struct emac_config *)(lp->base_addr + HZHY_EMAC_BASE_OFFSET);
			emac_base->moder = (EMAC_REV_ENABLE << EMAC_REV_EANBLE_SHIFT) | (EMAC_SEND_CRC_EN << EMAC_SEND_CRC_SHIFT) | (EMAC_LENGTHEN_EN << EMAC_LENGTHEN_SHIFT);
		/////////
			emac_base->min_flr = 64; //min 64 bytes
			emac_base->max_flr = 1536; //max 1536 bytes
			emac_base->hafr0 = 0xffff;
			emac_base->hafr1 = 0xffff;

		////////
			//printk("Howard: Register isr\n");

			/* Grab the IRQ */
			retval = request_irq(ndev->irq, hzhyemaclite_interrupt, 0, ndev->name, ndev);
#if 0		
	if (retval) {
				dev_err(&lp->ndev->dev, "Could not allocate interrupt %d\n",
						ndev->irq);
				if (lp->phy_dev)
					phy_disconnect(lp->phy_dev);
				lp->phy_dev = NULL;

				return retval;
			}
#endif

			/* Enable Interrupts */
			hzhyemaclite_enable_interrupts(lp);

			//printk("Howard: Call netif_start_queue to go.\n");

			/* We're ready to go */
			netif_start_queue(ndev);
		}
	}
#endif
}

/**
 * hzhyemaclite_open - Open the network device
 * @dev:	Pointer to the network device
 *
 * This function sets the MAC address, requests an IRQ and enables interrupts
 * for the Emaclite device and starts the Tx queue.
 * It also connects to the phy device, if MDIO is included in Emaclite device.
 */
static int hzhyemaclite_open(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	struct emac_config *emac_base;
	int retval;
	emac_base = (struct emac_config *)(lp->base_addr + HZHY_EMAC_BASE_OFFSET);
	
	printk("Howard: %s--%d--%s\n", __FILE__, __LINE__, __FUNCTION__);

	/* Just to be safe, stop the device first */
	hzhyemaclite_disable_interrupts(lp);

	//emac_base->mac_rst = 1;

	if (lp->phy_node) {
		u32 bmcr;
		
		printk("Howard: of_phy_connect\n");

		lp->phy_dev = of_phy_connect(lp->ndev, lp->phy_node,
					     hzhyemaclite_adjust_link, 0,
					     PHY_INTERFACE_MODE_RGMII);   // PHY_INTERFACE_MODE_MII  gai
		if (!lp->phy_dev) {
			dev_err(&lp->ndev->dev, "of_phy_connect() failed\n");
			return -ENODEV;
		}

		/* EmacLite doesn't support giga-bit speeds */
		lp->phy_dev->supported &= (PHY_BASIC_FEATURES);
		lp->phy_dev->advertising = lp->phy_dev->supported;
#if 1
		/* Don't advertise 1000BASE-T Full/Half duplex speeds */
		//phy_write(lp->phy_dev, MII_CTRL1000, 0); zhanghuan gai

		/* Advertise only 10 and 100mbps full/half duplex speeds */
		phy_write(lp->phy_dev, MII_ADVERTISE, ADVERTISE_ALL | ADVERTISE_CSMA);
#if 1
		/* Restart auto negotiation */
		bmcr = phy_read(lp->phy_dev, MII_BMCR);
		//bmcr |= (BMCR_SPEED100 | BMCR_FULLDPLX);
		//bmcr &= ~(1 << 12);
		bmcr |= 1 << 12;
		//bmcr |= (BMCR_ANENABLE | BMCR_ANRESTART);
		phy_write(lp->phy_dev, MII_BMCR, bmcr);
#endif

		phy_start(lp->phy_dev);
#endif
	}
	
	printk("Howard: hzhyemaclite_update_address\n");

	/* Set the MAC address each time opened */
	hzhyemaclite_update_address(lp, dev->dev_addr);
	
	//Enable receive
	//emac_base = (struct emac_config *)(lp->base_addr + HZHY_EMAC_BASE_OFFSET);
	emac_base->moder = (EMAC_REV_ENABLE << EMAC_REV_EANBLE_SHIFT) | (EMAC_SEND_CRC_EN << EMAC_SEND_CRC_SHIFT) | (EMAC_LENGTHEN_EN << EMAC_LENGTHEN_SHIFT);
/////////
	emac_base->min_flr = 64; //min 64 bytes
	emac_base->max_flr = 1536; //max 1536 bytes
	emac_base->hafr0 = 0xffff;
	emac_base->hafr1 = 0xffff;

////////
	printk("Howard: Register isr\n");

	/* Grab the IRQ */
	retval = request_irq(dev->irq, hzhyemaclite_interrupt, 0, dev->name, dev);
	if (retval) {
		dev_err(&lp->ndev->dev, "Could not allocate interrupt %d\n",
			dev->irq);
		if (lp->phy_dev)
			phy_disconnect(lp->phy_dev);
		lp->phy_dev = NULL;

		return retval;
	}

	/* Enable Interrupts */
	hzhyemaclite_enable_interrupts(lp);
	
	printk("Howard: Call netif_start_queue to go.\n");

	/* We're ready to go */
	netif_start_queue(dev);

	return 0;
}

/**
 * hzhyemaclite_close - Close the network device
 * @dev:	Pointer to the network device
 *
 * This function stops the Tx queue, disables interrupts and frees the IRQ for
 * the Emaclite device.
 * It also disconnects the phy device associated with the Emaclite device.
 */
static int hzhyemaclite_close(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	struct emac_config *emac_base;
	struct transfer_control *recv_base;
	struct transfer_control *send_base;

	emac_base = (struct emac_config *)(lp->base_addr + HZHY_EMAC_BASE_OFFSET);
	emac_base->moder = 0;
	emac_base->mac_rst = 1;
	emac_base->phy_rst = 1;

	lp->next_tx_buf_to_use = 0x0;
	lp->next_rx_buf_to_use = 0x0;

	netif_stop_queue(dev);
	hzhyemaclite_disable_interrupts(lp);
	free_irq(dev->irq, dev);

	if (lp->phy_dev)
		phy_disconnect(lp->phy_dev);
	lp->phy_dev = NULL;

	return 0;
}

/**
 * hzhyemaclite_send - Transmit a frame
 * @orig_skb:	Pointer to the socket buffer to be transmitted
 * @dev:	Pointer to the network device
 *
 * This function checks if the Tx buffer of the Emaclite device is free to send
 * data. If so, it fills the Tx buffer with data from socket buffer data,
 * updates the stats and frees the socket buffer. The Tx completion is signaled
 * by an interrupt. If the Tx buffer isn't free, then the socket buffer is
 * deferred and the Tx queue is stopped so that the deferred socket buffer can
 * be transmitted when the Emaclite device is free to transmit data.
 *
 * Return:	0, always.
 */
static int hzhyemaclite_send(struct sk_buff *orig_skb, struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	//u32 bmcr; 
	//u32 bmsr;  
	struct sk_buff *new_skb;
	unsigned int len;
	unsigned long flags;

	//printk("Howard: %s--%d--%s\n", __FILE__, __LINE__, __FUNCTION__);

	/*bmcr = phy_read(lp->phy_dev, MII_BMCR);
	printk("sun added :the BMCR[0x00] value is 0x%x\n", bmcr);
	bmsr = phy_read(lp->phy_dev, MII_BMSR);
	printk("sun added :the BMCR[0x00] value is 0x%x\n", bmsr);*/


	len = orig_skb->len;
	new_skb = orig_skb;

	spin_lock_irqsave(&lp->reset_lock, flags);
	if (hzhyemaclite_send_data(lp, (u8 *) new_skb->data, len) != 0) {
		/* If the Emaclite Tx buffer is busy, stop the Tx queue and
		 * defer the skb for transmission during the ISR, after the
		 * current transmission is complete */
		netif_stop_queue(dev);
		lp->deferred_skb = new_skb;

		/* Take the time stamp now, since we can't do this in an ISR. */
		skb_tx_timestamp(new_skb);
		spin_unlock_irqrestore(&lp->reset_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&lp->reset_lock, flags);

	skb_tx_timestamp(new_skb);

	dev->stats.tx_bytes += len;
	dev_consume_skb_any(new_skb);
	


	return 0;
}

/**
 * hzhyemaclite_remove_ndev - Free the network device
 * @ndev:	Pointer to the network device to be freed
 *
 * This function un maps the IO region of the Emaclite device and frees the net
 * device.
 */
static void hzhyemaclite_remove_ndev(struct net_device *ndev)
{
	if (ndev) {
		free_netdev(ndev);
	}
}


/**
 * get_bool - Get a parameter from the OF device
 * @ofdev:	Pointer to OF device structure
 * @s:		Property to be retrieved
 *
 * This function looks for a property in the device node and returns the value
 * of the property if its found or 0 if the property is not found.
 *
 * Return:	Value of the parameter if the parameter is found, or 0 otherwise
 */
static bool get_bool(struct platform_device *ofdev, const char *s)
{
	u32 *p = (u32 *)of_get_property(ofdev->dev.of_node, s, NULL);

	if (p) {
		return (bool)*p;
	} else {
		dev_warn(&ofdev->dev, "Parameter %s not found,"
			"defaulting to false\n", s);
		return false;
	}
}

static struct net_device_ops hzhyemaclite_netdev_ops;

/**
 * hzhyemaclite_of_probe - Probe method for the Emaclite device.
 * @ofdev:	Pointer to OF device structure
 * @match:	Pointer to the structure used for matching a device
 *
 * This function probes for the Emaclite device in the device tree.
 * It initializes the driver data structure and the hardware, sets the MAC
 * address and registers the network device.
 * It also registers a mii_bus for the Emaclite device, if MDIO is included
 * in the device.
 *
 * Return:	0, if the driver is bound to the Emaclite device, or
 *		a negative error if there is failure.
 */
static int hzhyemaclite_of_probe(struct platform_device *ofdev)
{
	struct resource *res;
	struct net_device *ndev = NULL;
	struct net_local *lp = NULL;
	struct device *dev = &ofdev->dev;
	struct transfer_control *recv_base;
	struct transfer_control *send_base;
	const void *mac_address;

	int rc = 0;

	printk("\nHoward: %s--%d--%s\n", __FILE__, __LINE__, __FUNCTION__);

	/* Create an ethernet device instance */
	ndev = alloc_etherdev(sizeof(struct net_local));
	if (!ndev)
		return -ENOMEM;

	dev_set_drvdata(dev, ndev);
	SET_NETDEV_DEV(ndev, &ofdev->dev);

	lp = netdev_priv(ndev);
	lp->ndev = ndev;

	/* Get IRQ for the device */
	res = platform_get_resource(ofdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "no IRQ found\n");
		rc = -ENXIO;
		goto error;
	}
	
	//ndev->irq = res->start;
	ndev->irq = platform_get_irq(ofdev, 0);

	printk("Howard: Have got ndev->irq=%d, res->start=%d\n", ndev->irq, res->start);

	res = platform_get_resource(ofdev, IORESOURCE_MEM, 0);
	lp->base_addr = devm_ioremap_resource(&ofdev->dev, res);
	if (IS_ERR(lp->base_addr)) {
		rc = PTR_ERR(lp->base_addr);
		goto error;
	}

	ndev->mem_start = res->start;
	ndev->mem_end = res->end;

    printk("Howard: Have got io memory, lp->base_addr=0x%x, mem_start=0x%x, mem_end=0x%x\n", lp->base_addr, res->start, res->end);
	
	spin_lock_init(&lp->reset_lock);
	lp->next_tx_buf_to_use = 0x0;
	lp->next_rx_buf_to_use = 0x0;


#if 1
	/* Set up receive buffers */
	lp->recv_virtaddr = dma_alloc_coherent(NULL, HZHY_RECVBUF_TOTOLSIZE, &lp->recv_physaddr, GFP_KERNEL);
	if(lp->recv_virtaddr == NULL)
	{
		printk("lp->recv_virtaddr dma_alloc_coherent failed\n");
		goto error;
	}

	recv_base = (struct transfer_control *)(lp->base_addr + HZHY_RECV_BASE_OFFSET);

	recv_base->addr = lp->recv_physaddr;

	recv_base->len  = HZHY_RECVBUF_LENGTH;

	recv_base->sum  = HZHY_RECVBUF_NUMBER;

	
	printk("Howard: Have got receive buffer, virtual address=0x%x, physical address=0x%x\n", lp->recv_virtaddr, lp->recv_physaddr);

	/* Set up send buffers */

	lp->send_virtaddr = dma_alloc_coherent(NULL, HZHY_SENDBUF_TOTOLSIZE, &lp->send_physaddr, GFP_KERNEL);
	if(lp->send_virtaddr == NULL)
	{
		printk("lp->send_virtaddr dma_alloc_coherent failed\n");
		goto error;
	}

	send_base = (struct transfer_control *)(lp->base_addr + HZHY_SEND_BASE_OFFSET);
	send_base->addr = lp->send_physaddr;
	send_base->len  = HZHY_SENDBUF_LENGTH;
	send_base->sum  = HZHY_SENDBUF_NUMBER;

	printk("Howard: Have got send buffer, virtual address=0x%x, physical address=0x%x\n", lp->send_virtaddr, lp->send_physaddr);
#endif
	/* map PL and PS communication buffer */
	mac_address = of_get_mac_address(ofdev->dev.of_node);
	if (mac_address)
		/* Set the MAC address. */
		memcpy(ndev->dev_addr, mac_address, ETH_ALEN);
	else
		dev_warn(dev, "No MAC address found\n");

	printk("Howard: Get the mac_address %s\n", (char *)mac_address);

	/* Clear the Tx CSR's in case this is a restart */


	/* Set the MAC address in the EmacLite device */
	printk("Howard: hzhyemaclite_update_address\n");
	hzhyemaclite_update_address(lp, ndev->dev_addr);
	printk("Howard: MAC address is now %pM\n", ndev->dev_addr);

	/* Get phy node */
	printk("Howard: Get the phy node\n");
	lp->phy_node = of_parse_phandle(ofdev->dev.of_node, "phy-handle", 0);
	rc = hzhyemaclite_mdio_setup(lp, &ofdev->dev);
	if (rc)
		dev_warn(&ofdev->dev, "error registering MDIO bus\n");

	/* Setup other fields */
	ndev->netdev_ops = &hzhyemaclite_netdev_ops;
	ndev->flags &= ~IFF_MULTICAST;
	ndev->watchdog_timeo = TX_TIMEOUT;

	/* Finally, register the device */
	rc = register_netdev(ndev);
	if (rc) {
		dev_err(dev,
			"Cannot register network device, aborting\n");
		goto error;
	}

	printk("Howard: Hzhy EmacLite at 0x%08X mapped to 0x%08lX, irq=%d\n\n",
			(unsigned int __force)ndev->mem_start,
			(unsigned long __force)lp->base_addr, ndev->irq);

	emac_set_check(lp->base_addr);
	return 0;

error:
	hzhyemaclite_remove_ndev(ndev);
	return rc;
}

/**
 * hzhyemaclite_of_remove - Unbind the driver from the Emaclite device.
 * @of_dev:	Pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 *
 * Return:	0, always.
 */
static int hzhyemaclite_of_remove(struct platform_device *of_dev)
{
	struct net_device *ndev = platform_get_drvdata(of_dev);
	struct net_local *lp = netdev_priv(ndev);

	/* Un-register the mii_bus, if configured */
	if (lp->has_mdio) {
		mdiobus_unregister(lp->mii_bus);
		mdiobus_free(lp->mii_bus);
		lp->mii_bus = NULL;
	}

	unregister_netdev(ndev);

	of_node_put(lp->phy_node);
	lp->phy_node = NULL;

	hzhyemaclite_remove_ndev(ndev);

	return 0;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void
hzhyemaclite_poll_controller(struct net_device *ndev)
{
	disable_irq(ndev->irq);
	hzhyemaclite_interrupt(ndev->irq, ndev);
	enable_irq(ndev->irq);
}
#endif

static struct net_device_ops hzhyemaclite_netdev_ops = {
	.ndo_open		= hzhyemaclite_open,
	.ndo_stop		= hzhyemaclite_close,
	.ndo_start_xmit		= hzhyemaclite_send,
	.ndo_set_mac_address	= hzhyemaclite_set_mac_address,
	.ndo_tx_timeout		= hzhyemaclite_tx_timeout,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = hzhyemaclite_poll_controller,
#endif
};

/* Match table for OF platform binding */
static const struct of_device_id hzhyemaclite_of_match[] = {
	{ .compatible = "hzhy,hzhy-ethernetlite-1.00", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, hzhyemaclite_of_match);

static struct platform_driver hzhyemaclite_of_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = hzhyemaclite_of_match,
	},
	.probe		= hzhyemaclite_of_probe,
	.remove		= hzhyemaclite_of_remove,
};

module_platform_driver(hzhyemaclite_of_driver);

MODULE_AUTHOR("Hzhy, Inc.");
MODULE_DESCRIPTION("Hzhy Ethernet MAC Lite driver");
MODULE_LICENSE("GPL");
