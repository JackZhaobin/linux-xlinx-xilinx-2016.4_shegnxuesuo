/*
 * Driver for the Texas Instruments DP83848 PHY
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/phy.h>

#define TI_DP83848C_PHY_ID		0x20005ca0
#define NS_DP83848C_PHY_ID		0x20005c90
#define TLK10X_PHY_ID			0x2000a210

/* Registers */
#define DP83848_MICR			0x11 /* MII Interrupt Control Register */
#define DP83848_MISR			0x12 /* MII Interrupt Status Register */

/* MICR Register Fields */
#define DP83848_MICR_INT_OE		BIT(0) /* Interrupt Output Enable */
#define DP83848_MICR_INTEN		BIT(1) /* Interrupt Enable */

/* MISR Register Fields */
#define DP83848_MISR_RHF_INT_EN		BIT(0) /* Receive Error Counter */
#define DP83848_MISR_FHF_INT_EN		BIT(1) /* False Carrier Counter */
#define DP83848_MISR_ANC_INT_EN		BIT(2) /* Auto-negotiation complete */
#define DP83848_MISR_DUP_INT_EN		BIT(3) /* Duplex Status */
#define DP83848_MISR_SPD_INT_EN		BIT(4) /* Speed status */
#define DP83848_MISR_LINK_INT_EN	BIT(5) /* Link status */
#define DP83848_MISR_ED_INT_EN		BIT(6) /* Energy detect */
#define DP83848_MISR_LQM_INT_EN		BIT(7) /* Link Quality Monitor */

#define DP83848_INT_EN_MASK		\
	(DP83848_MISR_ANC_INT_EN |	\
	 DP83848_MISR_DUP_INT_EN |	\
	 DP83848_MISR_SPD_INT_EN |	\
	 DP83848_MISR_LINK_INT_EN)

#define DP83822_PHY_ID			0x2000A240

static int dp83848_ack_interrupt(struct phy_device *phydev)
{
	int err = phy_read(phydev, DP83848_MISR);

	return err < 0 ? err : 0;
}

static int dp83848_config_intr(struct phy_device *phydev)
{
	int control, ret;

	control = phy_read(phydev, DP83848_MICR);
	if (control < 0)
		return control;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		control |= DP83848_MICR_INT_OE;
		control |= DP83848_MICR_INTEN;

		ret = phy_write(phydev, DP83848_MISR, DP83848_INT_EN_MASK);
		if (ret < 0)
			return ret;
	} else {
		control &= ~DP83848_MICR_INTEN;
	}

	return phy_write(phydev, DP83848_MICR, control);
}

/* NatSemi DP83822 */
#define DP83822_PHY_BMCR_REG 	0x00
#define DP83822_PHY_BMSR_REG 	0x01
#define DP83822_PHY_ANAR_REG	0x04
#define DP83822_PHY_CR1_REG		0x09
#define DP83822_PHY_CR2_REG		0x0A
#define DP83822_PHY_STAT_REG 	0x10
#define DP83822_PHY_PHYSCR_REG	0x11
#define DP83822_PHY_RCSR_REG	0x17
#define DP83822_PHY_PHYCR_REG	0x19

#define DP83822_DUPLEX		(1 << 2)
#define DP83822_SPEED		(1 << 1)
#define DP83822_LINK		(1 << 0)

#define IEEE_ASYMMETRIC_PAUSE_MASK			0x0800
#define IEEE_PAUSE_MASK						0x0400
#define ADVERTISE_100			(ADVERTISE_100FULL | ADVERTISE_100HALF)
#define ADVERTISE_10			(ADVERTISE_10FULL | ADVERTISE_10HALF)

#define IEEE_CTRL_AUTONEGOTIATE_ENABLE		0x1000
#define IEEE_STAT_AUTONEGOTIATE_RESTART		0x0200

#define PHY_83822_BCMR_10M_FULL		0x0100
#define PHY_83822_BCMR_100M_FULL		0x2100

static int DP83822_config_init(struct phy_device *phydev)
{
	int mii_reg;

printk("sun add :@@@@@@[%s:%s]%d interface=%d autoneg = %d########\n",__FILE__,__func__,__LINE__, phydev->interface, phydev->autoneg);

	if(phydev->interface == PHY_INTERFACE_MODE_RGMII_ID)
	{
		/*printk("*********************before************************\n");
		mii_reg = phy_read(phydev, DP83822_PHY_RCSR_REG);
		printk("sun added :the RCSR[0x17] value is 0x%x\n", mii_reg);	

		mii_reg = phy_read(phydev, DP83822_PHY_BMSR_REG); //PHYBMSR
		printk("sun added :the BMSR[0x01] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_STAT_REG); //PHYBMSR
		printk("sun added :the PHYSTAT[0x10] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_PHYCR_REG); //PHYBMSR
		printk("sun added :the PHYCR[0x19] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_BMCR_REG);
		printk("sun added :the BMCR[0x00] value is 0x%x\n", mii_reg);
		printk("*********************************************\n");*/

		mii_reg = phy_read(phydev, DP83822_PHY_RCSR_REG);
		mii_reg &=(~0x80) ; //[bit7] 0 = 25-MHz clock reference
		mii_reg |=(1<<12);//[bit12]1 
		phy_write(phydev, DP83822_PHY_RCSR_REG, mii_reg);
		mii_reg = phy_read(phydev, DP83822_PHY_RCSR_REG);
		printk("sun added :the RCSR[0x17] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_PHYSCR_REG);
		mii_reg |= 0x8000; //[bit15] 1 = Disable internal clocks circuitry
		phy_write(phydev, DP83822_PHY_PHYSCR_REG, mii_reg);
		mii_reg = phy_read(phydev, DP83822_PHY_PHYSCR_REG);
		printk("sun added :the PHYSCR[0x11] value is 0x%x\n", mii_reg);
#if 1
		mii_reg = phy_read(phydev, DP83822_PHY_BMCR_REG); //PHYCR
		//mii_reg |= (1 << 8) | (1 << 13); //close autoaneg force 100M full duplex
		//mii_reg &= ~(1 << 12);
		mii_reg |= (1<<9) | (1 << 12);
	
		phy_write(phydev, DP83822_PHY_BMCR_REG, mii_reg);
		mii_reg = phy_read(phydev, DP83822_PHY_BMCR_REG);
		printk("sun added :the BMCR[0x00] value is 0x%x\n", mii_reg);
#if 0
		mii_reg = phy_read(phydev, DP83822_PHY_CR1_REG); //PHYCR
		mii_reg |= (3 << 5);
		phy_write(phydev, DP83822_PHY_CR1_REG, mii_reg);
		mii_reg = phy_read(phydev, DP83822_PHY_CR1_REG);
		printk("sun added :the CR1[0x09] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_CR2_REG); //CR2
		mii_reg |= (1 << 4);							 //enhanced led link
		phy_write(phydev, DP83822_PHY_CR2_REG, mii_reg);
		mii_reg = phy_read(phydev, DP83822_PHY_CR2_REG);
		printk("sun added :the CR2[0x0A] value is 0x%x\n", mii_reg);
#endif
#endif
		/*mii_reg = phy_read(phydev, DP83822_PHY_BMSR_REG); //PHYBMSR
		printk("sun added :the BMSR[0x01] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_STAT_REG); //PHYBMSR
		printk("sun added :the PHYSTAT[0x10] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_PHYCR_REG); //PHYBMSR
		printk("sun added :the PHYCR[0x19] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_BMCR_REG);
		printk("sun added :the BMCR[0x00] value is 0x%x\n", mii_reg);*/

		return genphy_config_init(phydev);
	}
	else
	{

#if 1

		/*printk("*********************before************************\n");
		mii_reg = phy_read(phydev, DP83822_PHY_RCSR_REG);
		printk("sun added :the RCSR[0x17] value is 0x%x\n", mii_reg);	

		mii_reg = phy_read(phydev, DP83822_PHY_BMSR_REG); //PHYBMSR
		printk("sun added :the BMSR[0x01] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_STAT_REG); //PHYBMSR
		printk("sun added :the PHYSTAT[0x10] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_PHYCR_REG); //PHYBMSR
		printk("sun added :the PHYCR[0x19] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_BMCR_REG);
		printk("sun added :the BMCR[0x00] value is 0x%x\n", mii_reg);
		printk("*********************************************\n");*/
		mii_reg = phy_read(phydev, MII_BMCR);
		mii_reg |= 0x2100; //[bit13] 1 = 100 Mbps; [bit8] 1 = Full-Duple
		//mii_reg &= (~0x2100); //[bit13] 0 = 10 Mbps; [bit8] 0 = half-Duple		

		//mii_reg &= (~0x1000); //[bit12] 0 = Disable Auto-Negotiation
		//mii_reg |= (1<<13); //[bit13] 1 = 100 Mbps; [bit8] 1 = Full-Duple
		mii_reg |= 0x1000; //[bit12] 1 = Enable Auto-Negotiation
		//mii_reg |= (1<<9);
		
		phy_write(phydev, MII_BMCR, mii_reg);
		printk("sun added :the BMCR[0x00] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_RCSR_REG);
		//mii_reg &=(~0x200); //[bit9] 0 = Mode determined by bit[5]
		//mii_reg |=0x200;//[bit9] 1=enable rgmii

		//mii_reg |= 0x80; //[bit7] 1 = 50-MHz clock reference, CMOS-level oscillator
		mii_reg &=(~0x80); //[bit7] 0 = 25-MHz clock reference

		//mii_reg |= 0x20; //[bit5] 1 = Enable RMII mode of operation
		//mii_reg |= 0x100; //[bit8] 1 = Transmit path internal clock shift is enabled
		//mii_reg &= (~0x40); //[bit6] 0 = FIFO Normal operation
		phy_write(phydev, DP83822_PHY_RCSR_REG, mii_reg);
		mii_reg = phy_read(phydev, DP83822_PHY_RCSR_REG);
		printk("sun added :the RCSR[0x17] value is 0x%x\n", mii_reg);
		
		mii_reg = phy_read(phydev, DP83822_PHY_CR2_REG);
		printk("sun added :the CR2[0x0A] value is 0x%x\n", mii_reg);
		
		//disable auto-negotiation
		//phydev->autoneg = AUTONEG_DISABLE;
		
		return genphy_config_init(phydev);
		//disable auto-negotiation
		//phydev->autoneg = AUTONEG_DISABLE;
#endif 
	}

	return 0;
}

static int DP83822_config_aneg(struct phy_device *phydev)
{
	int mii_reg;

//printk("sun add: &&&&&&&&[%s:%s]%d&&&&&&&&&\n",__FILE__,__func__,__LINE__);
	if(phydev->interface == PHY_INTERFACE_MODE_RGMII_ID)
{
#if 1
		mii_reg = phy_read(phydev, DP83822_PHY_PHYCR_REG);
		mii_reg |= 0x8000; //[bit15] 1 = Enable Auto-Negotiation Auto-MDIX capability
		phy_write(phydev, DP83822_PHY_PHYCR_REG, mii_reg);
		mii_reg = phy_read(phydev, DP83822_PHY_PHYCR_REG);
//		printk("sun add: the PHYCR[0x19] value is 0x%x\n", mii_reg);
#endif
		mii_reg = phy_read(phydev, DP83822_PHY_STAT_REG);
//		printk("sun add: the PHYSTAT[0x10] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_BMCR_REG); //PHYCR
//		printk("sun added :the BMCR[0x00] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_ANAR_REG);
		mii_reg |= IEEE_ASYMMETRIC_PAUSE_MASK;
		mii_reg |= IEEE_PAUSE_MASK;
		mii_reg |= ADVERTISE_100;
		mii_reg |= ADVERTISE_10;
		phy_write(phydev, DP83822_PHY_ANAR_REG, mii_reg);
#if 1
		//Enable Auto-Negotiation
		mii_reg = phy_read(phydev, DP83822_PHY_BMCR_REG);
		mii_reg |= IEEE_CTRL_AUTONEGOTIATE_ENABLE;
		mii_reg |= IEEE_STAT_AUTONEGOTIATE_RESTART;
		phy_write(phydev, DP83822_PHY_BMCR_REG, mii_reg);
		
		/*mii_reg = phy_read(phydev, DP83822_PHY_BMSR_REG); //PHYBMSR
		printk("sun added :the BMSR[0x01] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_STAT_REG); //PHYBMSR
		printk("sun added :the PHYSTAT[0x10] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_PHYCR_REG); //PHYBMSR
		printk("sun added :the PHYCR[0x19] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_BMCR_REG);
		printk("sun added :the BMCR[0x00] value is 0x%x\n", mii_reg);*/

#endif
		return genphy_config_aneg(phydev);
	}
	else
	{
		/*printk("*********************after************************\n");
		mii_reg = phy_read(phydev, DP83822_PHY_RCSR_REG);
		printk("sun added :the RCSR[0x17] value is 0x%x\n", mii_reg);	

		mii_reg = phy_read(phydev, DP83822_PHY_BMSR_REG); //PHYBMSR
		printk("sun added :the BMSR[0x01] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_STAT_REG); //PHYBMSR
		printk("sun added :the PHYSTAT[0x10] value is 0x%x\n", mii_reg);

		mii_reg = phy_read(phydev, DP83822_PHY_PHYCR_REG); //PHYBMSR
		printk("sun added :the PHYCR[0x19] value is 0x%x\n", mii_reg);
		
		mii_reg = phy_read(phydev, DP83822_PHY_BMCR_REG);
		printk("sun added :the BMCR[0x00] value is 0x%x\n", mii_reg);
		printk("*********************************************\n");*/
		

		//Enable Auto-Negotiation
		mii_reg = phy_read(phydev, DP83822_PHY_BMCR_REG);
		mii_reg |= IEEE_CTRL_AUTONEGOTIATE_ENABLE;
		mii_reg |= IEEE_STAT_AUTONEGOTIATE_RESTART;
		phy_write(phydev, DP83822_PHY_BMCR_REG, mii_reg);
		return genphy_config_aneg(phydev);
#if 0
		phydev->speed = SPEED_100;
		phydev->duplex = DUPLEX_FULL;
		return genphy_config_aneg(phydev);
#endif
	}
	return 0;
}

static int DP83822_read_status(struct phy_device *phydev)
{
	int mii_reg,ret;

	if(phydev->interface == PHY_INTERFACE_MODE_RGMII_ID)
	{
		return genphy_read_status(phydev);
	}
	else
	{
		/*ret = phy_read(phydev, DP83822_PHY_STAT_REG);

		if (ret & DP83822_SPEED)
		{
			phydev->speed = SPEED_10;
		}
		else
		{
			phydev->speed = SPEED_100;
		}

		if (ret & DP83822_DUPLEX)
		{
			phydev->duplex = DUPLEX_FULL;
		}
		else
		{
			phydev->duplex = DUPLEX_HALF;
		}*/

		return genphy_read_status(phydev);
	}

	return 0;
}

static struct mdio_device_id __maybe_unused dp83848_tbl[] = {
	{ TI_DP83848C_PHY_ID, 0xfffffff0 },
	{ NS_DP83848C_PHY_ID, 0xfffffff0 },
	{ TLK10X_PHY_ID, 0xfffffff0 },
	{ DP83822_PHY_ID, 0xfffffff0 },
	{ }
};
MODULE_DEVICE_TABLE(mdio, dp83848_tbl);

#define DP83848_PHY_DRIVER(_id, _name)				\
	{							\
		.phy_id		= _id,				\
		.phy_id_mask	= 0xfffffff0,			\
		.name		= _name,			\
		.features	= PHY_BASIC_FEATURES,		\
		.flags		= PHY_HAS_INTERRUPT,		\
								\
		.soft_reset	= genphy_soft_reset,		\
		.config_init	= genphy_config_init,		\
		.suspend	= genphy_suspend,		\
		.resume		= genphy_resume,		\
		.config_aneg	= genphy_config_aneg,		\
		.read_status	= genphy_read_status,		\
								\
		/* IRQ related */				\
		.ack_interrupt	= dp83848_ack_interrupt,	\
		.config_intr	= dp83848_config_intr,		\
	}

#define DP83822_PHY_DRIVER(_id, _name)				\
	{							\
		.phy_id		= _id,				\
		.phy_id_mask	= 0xfffffff0,			\
		.name		= _name,			\
		.features	= PHY_BASIC_FEATURES,		\
		.flags		= PHY_HAS_INTERRUPT,		\
								\
		.soft_reset	= genphy_soft_reset,		\
		/*.config_init	= genphy_config_init,*/		\
		.config_init	= DP83822_config_init,		\
		.suspend	= genphy_suspend,		\
		.resume		= genphy_resume,		\
		/*.config_aneg	= genphy_config_aneg,	*/	\
		.config_aneg	= DP83822_config_aneg,		\
		/*.read_status	= genphy_read_status,	*/	\
		.read_status	= DP83822_read_status,		\
								\
		/* IRQ related */				\
		.ack_interrupt	= dp83848_ack_interrupt,	\
		.config_intr	= dp83848_config_intr,		\
	}

static struct phy_driver dp83848_driver[] = {
	DP83848_PHY_DRIVER(TI_DP83848C_PHY_ID, "TI DP83848C 10/100 Mbps PHY"),
	DP83848_PHY_DRIVER(NS_DP83848C_PHY_ID, "NS DP83848C 10/100 Mbps PHY"),
	DP83848_PHY_DRIVER(TLK10X_PHY_ID, "TI TLK10X 10/100 Mbps PHY"),
	DP83822_PHY_DRIVER(DP83822_PHY_ID, "TI DP83822 10/100 Mbps PHY"),
};
module_phy_driver(dp83848_driver);

MODULE_DESCRIPTION("Texas Instruments DP83848 PHY driver");
MODULE_AUTHOR("Andrew F. Davis <afd@ti.com");
MODULE_LICENSE("GPL");
