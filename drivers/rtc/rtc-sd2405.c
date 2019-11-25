/*
 * Driver for Epson's RTC module RX-8025 SA/NB
 *
 * Copyright (C) 2009 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Copyright (C) 2005 by Digi International Inc.
 * All rights reserved.
 *
 * Modified by fengjh at rising.com.cn
 * <lm-sensors@lm-sensors.org>
 * 2006.11
 *
 * Code cleanup by Sergei Poselenov, <sposelenov@emcraft.com>
 * Converted to new style by Wolfgang Grandegger <wg@grandegger.com>
 * Alarm and periodic interrupt added by Dmitry Rakhchev <rda@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#include <linux/bcd.h>
#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rtc.h>

/* Register definitions */
#define SD2405_REG_SEC			(0x00)
#define		SD2405_MASK_SEC		(0x7f)
#define SD2405_REG_MIN			(0x01)
#define		SD2405_MASK_MIN		(0x7f)
#define SD2405_REG_HOUR			(0x02)
#define		SD2405_MASK_HOUR_24	(0x2f)
#define		SD2405_MASK_HOUR_12	(0x1f)
#define		SD2405_BIT_1224		BIT(7)
#define		SD2405_BIT_AMPM		BIT(5)
#define SD2405_REG_WDAY			(0x03)
#define		SD2405_MASK_WDAY	(0x07)
#define SD2405_REG_MDAY			(0x04)
#define		SD2405_MASK_MDAY	(0x2f)
#define SD2405_REG_MONTH		(0x05)
#define		SD2405_MASK_MONTH	(0x1f)
#define SD2405_REG_YEAR			(0x06)
#define		SD2405_MASK_YEAR	(0xff)

#define SD2405_REG_ALMSEC		(0x07)
#define SD2405_REG_ALMMIN		(0x08)
#define SD2405_REG_ALMHOUR		(0x09)
#define SD2405_REG_ALMWDAY		(0x0a)
#define SD2405_REG_ALMMDAY		(0x0b)
#define SD2405_REG_ALMMONTH		(0x0c)
#define SD2405_REG_ALMYEAR		(0x0d)
#define SD2405_REG_ALMEN		(0x0e)
#define		SD2405_BIT_ALEAY	BIT(6)
#define		SD2405_BIT_ALEAMO	BIT(5)
#define		SD2405_BIT_ALEAD	BIT(4)
#define		SD2405_BIT_ALEAW	BIT(3)
#define		SD2405_BIT_ALEAH	BIT(2)
#define		SD2405_BIT_ALEAMN	BIT(1)
#define		SD2405_BIT_ALEAS	BIT(0)

#define SD2405_REG_CTRL1		(0x0f)
#define		SD2405_BIT_WRTC3	BIT(7)
#define		SD2405_BIT_INTAF	BIT(5)
#define		SD2405_BIT_INTDF	BIT(4)
#define		SD2405_BIT_WRTC2	BIT(2)
#define		SD2405_BIT_RTCF		BIT(0)
#define SD2405_REG_CTRL2		(0x10)
#define		SD2405_BIT_WRTC1	BIT(7)
#define		SD2405_BIT_IM		BIT(6)
#define		SD2405_MASK_INTS	(0x30)
/* interrupt pin output */
#define		SD2405_INTS_NONE	(0x00 << 4)
#define		SD2405_INTS_ALM		(0x01 << 4)
#define		SD2405_INTS_FREQ	(0x02 << 4)
#define		SD2405_INTS_CT		(0x03 << 4)
#define		SD2405_BIT_FOBAT	BIT(3)
#define		SD2405_BIT_INTDE	BIT(2)
#define		SD2405_BIT_INTAE	BIT(1)
#define		SD2405_BIT_INTFE	BIT(0)
#define SD2405_REG_CTRL3		(0x11)
#define		SD2405_BIT_ARST		BIT(7)
#define		SD2405_MASK_TDS		(0x30)
/* counte down timer clock frequency */
#define		SD2405_TDS_4096		(0x00 << 4)	/* 4096 HZ */
#define		SD2405_TDS_64		(0x01 << 4)	/* 64 HZ */
#define		SD2405_TDS_1		(0x02 << 4)	/* 1 HZ */
#define		SD2405_TDS_1_60		(0x03 << 4)	/* 1/60 HZ */

#define		SD2405_MASK_FS		(0x0f)
#define		SD2405_BITS_FS(f)	(((f) << 0) & SD2405_MASK_FS)
#define		SD2405_BITS_FS_1S	(0x0f << 0)

#define SD2405_REG_TIMEADJ		(0x12)
#define SD2405_REG_CDTIMER		(0x13)

#define SD2405_REG_RAMSTART		(0x14)
#define SD2405_REG_RAMEND		(0x1f)

#define SD2405_MAX_YEAR			(99)

#define SD2405_MASK_I2C_MODE		(0xE0)

#define WRITE_PROTECT

static const struct i2c_device_id sd2405_id[] = {
	{ "sd2405", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sd2405_id);

struct sd2405_data {
	struct i2c_client *client;
	struct rtc_device *rtc;
#define SD2405_HOUR_MODE_24 1
	int hour_mode;
};

static s32 sd2405_read_reg(const struct i2c_client *client, u8 address)
{
	return i2c_smbus_read_byte_data(client, address);
}

static int sd2405_read_regs(const struct i2c_client *client,
				u8 address, u8 length, u8 *values)
{
	int ret = i2c_smbus_read_i2c_block_data(client, address, length,
						values);
	if (ret != length)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static s32 sd2405_write_reg(const struct i2c_client *client, u8 address,
				u8 value)
{
#ifdef SD2405_DEBUG
	int i;
	u8 regs[32];
#endif
	s32 ret;
#ifdef WRITE_PROTECT
	s32 ctrl1, ctrl2;
	u8 ctrl;

	/* enable write */
	ret = i2c_smbus_write_byte_data(client, SD2405_REG_CTRL2, 
			SD2405_BIT_WRTC1);
	if (ret < 0)
		goto disable;

	ret = i2c_smbus_write_byte_data(client, SD2405_REG_CTRL1, 
			SD2405_BIT_WRTC2 | SD2405_BIT_WRTC3);
	if (ret < 0)
		goto disable;
#endif

	ret = i2c_smbus_write_byte_data(client, address, value);

#ifdef WRITE_PROTECT
disable:
	/* disable write */
	ctrl1 = i2c_smbus_read_byte_data(client, SD2405_REG_CTRL1);
	ctrl2 = i2c_smbus_read_byte_data(client, SD2405_REG_CTRL2);
	ctrl = ctrl1 & ~(SD2405_BIT_WRTC2 | SD2405_BIT_WRTC3);
	i2c_smbus_write_byte_data(client, SD2405_REG_CTRL1, ctrl);
	ctrl = ctrl2 & ~SD2405_BIT_WRTC1;
	i2c_smbus_write_byte_data(client, SD2405_REG_CTRL2, ctrl);
#endif
#ifdef SD2405_DEBUG
	sd2405_read_regs(client, 0x00, 32, regs);
	for (i = 0; i < 32; i++)
		printk(KERN_ALERT "a:0x%02x  d:0x%02x\n", i, regs[i]);
#endif
	return ret;
}

static s32 sd2405_write_regs(const struct i2c_client *client,
				 u8 address, u8 length, const u8 *values)
{
#ifdef SD2405_DEBUG
	int i;
	u8 regs[32];
#endif
	s32 ret;
#ifdef WRITE_PROTECT
	s32 ctrl1, ctrl2;
	u8 ctrl;
	/* enable write */
	ret = i2c_smbus_write_byte_data(client, SD2405_REG_CTRL2, 
			SD2405_BIT_WRTC1);
	if (ret < 0)
		goto disable;

	ret = i2c_smbus_write_byte_data(client, SD2405_REG_CTRL1, 
			SD2405_BIT_WRTC2 | SD2405_BIT_WRTC3);
	if (ret < 0)
		goto disable;
#endif

	ret = i2c_smbus_write_i2c_block_data(client, address,
					length, values);
#ifdef WRITE_PROTECT				
disable:
	/* disable write */
	ctrl1 = i2c_smbus_read_byte_data(client, SD2405_REG_CTRL1);
	ctrl2 = i2c_smbus_read_byte_data(client, SD2405_REG_CTRL2);
	ctrl = ctrl1 & ~(SD2405_BIT_WRTC2 | SD2405_BIT_WRTC3);
	i2c_smbus_write_byte_data(client, SD2405_REG_CTRL1, ctrl);
	ctrl = ctrl2 & ~SD2405_BIT_WRTC1;
	i2c_smbus_write_byte_data(client, SD2405_REG_CTRL2, ctrl);
#endif
#ifdef SD2405_DEBUG
	sd2405_read_regs(client, 0x00, 32, regs);
	for (i = 0; i < 32; i++)
		printk(KERN_ALERT "a:0x%02x  d:0x%02x\n", i, regs[i]);
#endif
	return ret;
}

static irqreturn_t sd2405_handle_irq(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct sd2405_data *sd2405 = i2c_get_clientdata(client);
	struct mutex *lock = &sd2405->rtc->ops_lock;
	int ctrl1, ctrl2, ctrl3;

	mutex_lock(lock);
	ctrl1 = sd2405_read_reg(client, SD2405_REG_CTRL1);
	ctrl2 = sd2405_read_reg(client, SD2405_REG_CTRL2);
	ctrl3 = sd2405_read_reg(client, SD2405_REG_CTRL3);
	if (ctrl1 < 0 || ctrl2 < 0 || ctrl3 < 0)
		goto out;

	if ((ctrl2 & SD2405_MASK_INTS) == SD2405_INTS_FREQ) {
		/* periodic */
		if ((ctrl3 & SD2405_MASK_FS) == 0x0a)
			/* 1s */
			rtc_update_irq(sd2405->rtc, 1, RTC_UF | RTC_IRQF);
		else
			rtc_update_irq(sd2405->rtc, 1, RTC_PF | RTC_IRQF);
	}

	if ((ctrl2 & SD2405_MASK_INTS) == SD2405_INTS_ALM) {
		/* alarm */
		if (!(ctrl3 & SD2405_BIT_ARST)) {	/* manually clear flag */
			ctrl1 &= ~SD2405_BIT_INTAF;
			if (sd2405_write_reg(client, SD2405_REG_CTRL1, ctrl1))
				goto out;
		}
		rtc_update_irq(sd2405->rtc, 1, RTC_AF | RTC_IRQF);
	}

	if ((ctrl2 & SD2405_MASK_INTS) == SD2405_INTS_CT) {
		/* count down timer, just clear flag, do nothing */
		if (!(ctrl3 & SD2405_BIT_ARST)) {	/* manually clear flag */
			ctrl1 &= ~SD2405_BIT_INTDF;
			if (sd2405_write_reg(client, SD2405_REG_CTRL1, ctrl1))
				goto out;
		}
	}

out:
	mutex_unlock(lock);

	return IRQ_HANDLED;
}

static int sd2405_get_time(struct device *dev, struct rtc_time *dt)
{
	struct sd2405_data *sd2405 = dev_get_drvdata(dev);
	u8 date[7];
	int err;

	err = sd2405_read_regs(sd2405->client, SD2405_REG_SEC, 7, date);
	if (err)
		return err;

	dev_dbg(dev, "%s: read 0x%02x 0x%02x "
		"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", __func__,
		date[0], date[1], date[2], date[3], date[4],
		date[5], date[6]);

	dt->tm_sec = bcd2bin(date[SD2405_REG_SEC] & SD2405_MASK_SEC);
	dt->tm_min = bcd2bin(date[SD2405_REG_MIN] & SD2405_MASK_MIN);
	if (sd2405->hour_mode == SD2405_HOUR_MODE_24)
		dt->tm_hour = bcd2bin(date[SD2405_REG_HOUR] & 0x3f);
	else
		dt->tm_hour = bcd2bin(date[SD2405_REG_HOUR] & 0x1f) % 12
			+ (date[SD2405_REG_HOUR] & 0x20 ? 12 : 0);

	dt->tm_mday = bcd2bin(date[SD2405_REG_MDAY] & 0x3f);
	/* tm_mon is number of month 0-11 */
	dt->tm_mon = bcd2bin(date[SD2405_REG_MONTH] & 0x1f) - 1;
	/* tm_year is number since 1900 */
	dt->tm_year = bcd2bin(date[SD2405_REG_YEAR]) + 100;

	dev_dbg(dev, "%s: date %ds %dm %dh %dmd %dm %dy\n", __func__,
		dt->tm_sec, dt->tm_min, dt->tm_hour,
		dt->tm_mday, dt->tm_mon, dt->tm_year);

	return rtc_valid_tm(dt);
}

static int sd2405_set_time(struct device *dev, struct rtc_time *dt)
{
	struct sd2405_data *sd2405 = dev_get_drvdata(dev);
	u8 date[7];
	int ret;

	if ((dt->tm_year < 100) || (dt->tm_year > 199))
		return -EINVAL;

	/*
	 * Here the read-only bits are written as "0".	I'm not sure if that
	 * is sound.
	 */
	date[SD2405_REG_SEC] = bin2bcd(dt->tm_sec);
	date[SD2405_REG_MIN] = bin2bcd(dt->tm_min);
	if (sd2405->hour_mode == SD2405_HOUR_MODE_24)
		date[SD2405_REG_HOUR] = bin2bcd(dt->tm_hour);
	else
		date[SD2405_REG_HOUR] = (dt->tm_hour >= 12 ? 0x20 : 0)
			| bin2bcd((dt->tm_hour + 11) % 12 + 1);

	date[SD2405_REG_WDAY] = bin2bcd(dt->tm_wday);
	date[SD2405_REG_MDAY] = bin2bcd(dt->tm_mday);
	date[SD2405_REG_MONTH] = bin2bcd(dt->tm_mon + 1);
	date[SD2405_REG_YEAR] = bin2bcd(dt->tm_year - 100);

	dev_dbg(dev,
		"%s: write 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		__func__,
		date[0], date[1], date[2], date[3], date[4], date[5], date[6]);

	ret = sd2405_write_regs(sd2405->client, SD2405_REG_SEC, 7, date);

	return ret;
}

static int sd2405_init_client(struct i2c_client *client)
{
	struct sd2405_data *sd2405 = i2c_get_clientdata(client);
	u8 ctrl[3], ctrl1, ctrl_w[5];
	int need_clear = 0;
	int err;

	ctrl[0] = ctrl[1] = ctrl[2] = 0xff;
	err = sd2405_read_regs(sd2405->client, SD2405_REG_CTRL1, 3, ctrl);
	if (err)
		goto out;

	/* lost all power last time, reconfig to default 1s period */
	if (ctrl[0] & SD2405_BIT_RTCF) {
		ctrl_w[0] = SD2405_BIT_WRTC3 | SD2405_BIT_WRTC2;
		ctrl_w[1] = SD2405_BIT_WRTC1 | SD2405_BIT_IM |\
			    SD2405_INTS_FREQ | SD2405_BIT_INTFE;
		/* set frequency select 1s and auto reset all int flag*/
		ctrl_w[2] = SD2405_BIT_ARST | SD2405_BITS_FS_1S;
		ctrl_w[3] = 0;
		ctrl_w[4] = 0;
		
		err = sd2405_write_regs(sd2405->client, SD2405_REG_CTRL1, 5, ctrl_w);
		if (err < 0)
			goto out;
	}

	/* alarm interrupt flag */
	if (ctrl[0] & SD2405_BIT_INTAF) {
		dev_warn(&client->dev, "Alarm was detected\n");
		need_clear = 1;
	}

	/* count down timer interrupt flag */
	if (ctrl[0] & SD2405_BIT_INTDF)
		need_clear = 1;

	/* need clear and auto reset flag not set */
	if (need_clear && ~(ctrl[2] | SD2405_BIT_ARST)) {
		ctrl1 = ctrl[0];
		ctrl1 &= ~(SD2405_BIT_INTAF | SD2405_BIT_INTDF |
			   SD2405_BIT_RTCF);

		err = sd2405_write_reg(client, SD2405_REG_CTRL1, ctrl1);
	}
out:
	return err;
}

/* Alarm support */
static int sd2405_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct sd2405_data *sd2405 = dev_get_drvdata(dev);
	struct i2c_client *client = sd2405->client;
	u8 ald[8];
	int ctrl1, ctrl2, err, i;

	if (client->irq <= 0)
		return -EINVAL;

	err = sd2405_read_regs(client, SD2405_REG_ALMSEC, 8, ald);
	if (err)
		return err;

	ctrl1 = sd2405_read_reg(client, SD2405_REG_CTRL1);
	if (ctrl1 < 0)
		return ctrl1;

	ctrl2 = sd2405_read_reg(client, SD2405_REG_CTRL2);
	if (ctrl2 < 0)
		return ctrl2;

	dev_dbg(dev, "%s: read alarm 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ctrl2 %02x\n",
			__func__, ald[0], ald[1], ald[2], ald[3], ald[4], 
			ald[5], ald[6], ald[7], ctrl2);

	/* Hardware alarms precision is 1 second! */
	t->time.tm_sec = bcd2bin(ald[0] & SD2405_MASK_SEC);
	t->time.tm_min = bcd2bin(ald[1] & SD2405_MASK_MIN);
	if (sd2405->hour_mode == SD2405_HOUR_MODE_24)
		t->time.tm_hour = bcd2bin(ald[2] & SD2405_MASK_HOUR_24);
	else
		t->time.tm_hour = bcd2bin(ald[2] & SD2405_MASK_HOUR_12) % 12
			+ (ald[2] & 0x20 ? 12 : 0);
	
	/* alarm week day if bit indicated, 0 - 6 <> bit0 - bit6
	 * we just get the first alarm weekday 
	 * */
	t->time.tm_wday = 0;
	for (i = 0; i < 7; i++) {
		if ((ald[3] >> i) & 0x01) {
			t->time.tm_wday = i;
			break;
		}
	}
	t->time.tm_wday++;
	t->time.tm_mday = bcd2bin(ald[4] & SD2405_MASK_MDAY);
	t->time.tm_mon = bcd2bin(ald[5] & SD2405_MASK_MONTH);
	t->time.tm_year = bcd2bin(ald[6] & SD2405_MASK_YEAR);

	dev_dbg(dev, "%s: date: %ds %dm %dh %dmd %dm %dy\n",
		__func__,
		t->time.tm_sec, t->time.tm_min, t->time.tm_hour,
		t->time.tm_mday, t->time.tm_mon, t->time.tm_year);
	t->enabled = (ctrl2 & SD2405_BIT_INTAE);
	t->pending = (ctrl1 & SD2405_BIT_INTAF) && t->enabled;

	return err;
}

static int sd2405_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sd2405_data *sd2405 = dev_get_drvdata(dev);
	u8 ald[8];
	int ctrl1, ctrl2, err;

	if (client->irq <= 0)
		return -EINVAL;

	/*
	 * Hardware alarm precision is 1 second!
	 */
	ald[0] = bin2bcd(t->time.tm_sec);
	ald[1] = bin2bcd(t->time.tm_min);
	if (sd2405->hour_mode == SD2405_HOUR_MODE_24)
		ald[2] = bin2bcd(t->time.tm_hour);
	else
		ald[2] = (t->time.tm_hour >= 12 ? 0x20 : 0)
			| bin2bcd((t->time.tm_hour + 11) % 12 + 1);

	ald[3] = 0x01 << t->time.tm_wday;
	ald[4] = bin2bcd(t->time.tm_mday);
	ald[5] = bin2bcd(t->time.tm_mon);
	ald[6] = bin2bcd(t->time.tm_year);
	/* enable all alarm except alarm week */
	ald[7] = SD2405_BIT_ALEAY | SD2405_BIT_ALEAMO | SD2405_BIT_ALEAD |
		 SD2405_BIT_ALEAH | SD2405_BIT_ALEAMN | SD2405_BIT_ALEAS;

	dev_dbg(dev, "%s: write 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \
			0x%02x 0x%02x\n", __func__, ald[0], ald[1], ald[2], 
			ald[3], ald[4], ald[5], ald[6], ald[7]);

	ctrl1 = sd2405_read_reg(sd2405->client, SD2405_REG_CTRL1);
	if (ctrl1 < 0)
		return ctrl1;

	if (ctrl1 & SD2405_BIT_INTAF) {
		ctrl1 &= ~SD2405_BIT_INTAF;
		ctrl1 |= SD2405_BIT_WRTC3 | SD2405_BIT_WRTC2;
		err = sd2405_write_reg(sd2405->client, SD2405_REG_CTRL1,ctrl1);
		if (err)
			return err;
	}
	err = sd2405_write_regs(sd2405->client, SD2405_REG_ALMSEC, 8, ald);

	if (err)
		return err;

	if (t->enabled) {
		ctrl2 = sd2405_read_reg(sd2405->client, SD2405_REG_CTRL2);
		if (ctrl2 < 0)
			return ctrl2;
		ctrl2 |= SD2405_BIT_INTAE | SD2405_BIT_WRTC1;
		err = sd2405_write_reg(sd2405->client, SD2405_REG_CTRL2,
					   ctrl2);
		if (err)
			return err;
	}

	return 0;
}

static int sd2405_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct sd2405_data *sd2405 = dev_get_drvdata(dev);
	s32 ctrl_reg;
	u8 ctrl2;
	int err;

	ctrl_reg = sd2405_read_reg(sd2405->client, SD2405_REG_CTRL2);
	if (ctrl_reg < 0)
		return ctrl_reg;
	
	ctrl2 = ctrl_reg;
	if (enabled)
		ctrl2 |= SD2405_BIT_INTAE;
	else
		ctrl2 &= ~SD2405_BIT_INTAE;

	ctrl2 |= SD2405_BIT_WRTC1;
	if (ctrl2 != ctrl_reg) {
		err = sd2405_write_reg(sd2405->client, SD2405_REG_CTRL2,
					   ctrl2);
		if (err)
			return err;
	}
	return 0;
}

static struct rtc_class_ops sd2405_rtc_ops = {
	.read_time = sd2405_get_time,
	.set_time = sd2405_set_time,
	.read_alarm = sd2405_read_alarm,
	.set_alarm = sd2405_set_alarm,
	.alarm_irq_enable = sd2405_alarm_irq_enable,
};

/*
 * Clock precision adjustment support
 *
 * According to the SD2405 SA/NB application manual the frequency and
 * temperature characteristics can be approximated using the following
 * equation:
 *
 *	 df = a * (ut - t)**2
 *
 *	 df: Frequency deviation in any temperature
 *	 a : Coefficient = (-35 +-5) * 10**-9
 *	 ut: Ultimate temperature in degree = +25 +-5 degree
 *	 t : Any temperature in degree
 *
 * Note that the clock adjustment in ppb must be entered (which is
 * the negative value of the deviation).
 */
static int sd2405_get_clock_adjust(struct device *dev, int *adj)
{
/*
	struct i2c_client *client = to_i2c_client(dev);
	int digoff;

	digoff = sd2405_read_reg(client, SD2405_REG_DIGOFF);
	if (digoff < 0)
		return digoff;

	*adj = digoff >= 64 ? digoff - 128 : digoff;
	if (*adj > 0)
		(*adj)--;
	*adj *= -SD2405_ADJ_RESOLUTION;
*/
	*adj = 0;
	return 0;
}

static int sd2405_set_clock_adjust(struct device *dev, int adj)
{
/*
	struct i2c_client *client = to_i2c_client(dev);
	u8 digoff;
	int err;

	adj /= -SD2405_ADJ_RESOLUTION;
	if (adj > SD2405_ADJ_DATA_MAX)
		adj = SD2405_ADJ_DATA_MAX;
	else if (adj < SD2405_ADJ_DATA_MIN)
		adj = SD2405_ADJ_DATA_MIN;
	else if (adj > 0)
		adj++;
	else if (adj < 0)
		adj += 128;
	digoff = adj;

	err = sd2405_write_reg(client, SD2405_REG_DIGOFF, digoff);
	if (err)
		return err;

	dev_dbg(dev, "%s: write 0x%02x\n", __func__, digoff);
*/
	return 0;
}

static ssize_t sd2405_sysfs_show_clock_adjust(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	int err, adj;

	err = sd2405_get_clock_adjust(dev, &adj);
	if (err)
		return err;

	return sprintf(buf, "%d\n", adj);
}

static ssize_t sd2405_sysfs_store_clock_adjust(struct device *dev,
						   struct device_attribute *attr,
						   const char *buf, size_t count)
{
	int adj, err;

	if (sscanf(buf, "%i", &adj) != 1)
		return -EINVAL;

	err = sd2405_set_clock_adjust(dev, adj);

	return err ? err : count;
}

static DEVICE_ATTR(clock_adjust_ppb, S_IRUGO | S_IWUSR,
		   sd2405_sysfs_show_clock_adjust,
		   sd2405_sysfs_store_clock_adjust);

static int sd2405_sysfs_register(struct device *dev)
{
	return device_create_file(dev, &dev_attr_clock_adjust_ppb);
}

static void sd2405_sysfs_unregister(struct device *dev)
{
	device_remove_file(dev, &dev_attr_clock_adjust_ppb);
}

static int sd2405_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sd2405_data *sd2405;
	int err = 0;
#ifdef SD2405_DEBUG
	int i;
	u8 regs[32];
#endif
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA
					 | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&adapter->dev,
			"doesn't support required functionality\n");
		return -EIO;
	}

	sd2405 = devm_kzalloc(&client->dev, sizeof(*sd2405), GFP_KERNEL);
	if (!sd2405)
		return -ENOMEM;

	sd2405->client = client;
	i2c_set_clientdata(client, sd2405);

	err = sd2405_init_client(client);
	if (err)
		return err;

	sd2405->rtc = devm_rtc_device_register(&client->dev, client->name,
					  &sd2405_rtc_ops, THIS_MODULE);
	if (IS_ERR(sd2405->rtc)) {
		dev_err(&client->dev, "unable to register the class device\n");
		return PTR_ERR(sd2405->rtc);
	}

	if (client->irq > 0) {
		dev_info(&client->dev, "IRQ %d supplied\n", client->irq);
		err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						sd2405_handle_irq,
						IRQF_ONESHOT,
						"sd2405", client);
		if (err) {
			dev_err(&client->dev, 
			  "unable to request IRQ, alarms disabled.<%d>\n", err);
			client->irq = 0;
		}
	}

	sd2405->rtc->max_user_freq = 32768;

	sd2405->rtc->uie_unsupported = 0;

	err = sd2405_sysfs_register(&client->dev);

#ifdef SD2405_DEBUG
	sd2405_read_regs(client, 0x00, 32, regs);
	for (i = 0; i < 32; i++)
		printk(KERN_ALERT "a:0x%02x  d:0x%02x\n", i, regs[i]);
#endif

	return err;
}

static int sd2405_remove(struct i2c_client *client)
{
	sd2405_sysfs_unregister(&client->dev);
	return 0;
}

static struct of_device_id sd2405_of_match[] = {
	{.compatible = "sd2405"},
	{},
};

static struct i2c_driver sd2405_driver = {
	.driver = {
		.name = "rtc-sd2405",
		.of_match_table = of_match_ptr(sd2405_of_match),
	},
	.probe		= sd2405_probe,
	.remove		= sd2405_remove,
	.id_table	= sd2405_id,
};

module_i2c_driver(sd2405_driver);

MODULE_AUTHOR("liuxn <liuxn_xa@hzhytech.com>");
MODULE_DESCRIPTION("SD-2405 AL RTC driver");
MODULE_LICENSE("GPL");
