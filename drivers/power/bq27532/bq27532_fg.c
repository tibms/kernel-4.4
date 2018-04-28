/*
 * bqGauge battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 */
#define pr_fmt(fmt)	"bq27532- %s: " fmt, __func__
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <asm/unaligned.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/debugfs.h>
#include <linux/alarmtimer.h>
#include "bqfs_cmd_type.h"
#include "bq27532_gmfs_coslight.h"

#define	bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_err
#define bq_log	pr_err

#define	INVALID_REG_ADDR	0xFF
#define BQFS_UPDATE_KEY		0x8F91


#define	FG_FLAGS_FC				BIT(9)
#define	FG_FLAGS_CHG				BIT(8)
#define	FG_FLAGS_BAT_DET			BIT(3)
#define	FG_FLAGS_SOC1				BIT(2)
#define	FG_FLAGS_DSG				BIT(0)


enum bq_fg_reg_idx {
	BQ_FG_REG_CTRL = 0,
	BQ_FG_REG_TEMP,		/* Battery Temperature */
	BQ_FG_REG_VOLT,		/* Battery Voltage */
	BQ_FG_REG_AI,		/* Average Current */
	BQ_FG_REG_FLAGS,	/* Flags */
	BQ_FG_REG_TTE,		/* Time to Empty */
	BQ_FG_REG_TTF,		/* Time to Full */
	BQ_FG_REG_FCC,		/* Full Charge Capacity */
	BQ_FG_REG_RM,		/* Remaining Capacity */
	BQ_FG_REG_CC,		/* Cycle Count */
	BQ_FG_REG_SOC,		/* Relative State of Charge */
	BQ_FG_REG_SOH,		/* State of Health */
	BQ_FG_REG_DC,		/* Design Capacity */

	NUM_REGS,
};

enum bq_fg_subcmd {
	FG_SUBCMD_CTRL_STATUS	= 0x0000,
	FG_SUBCMD_DEV_TYPE	= 0x0001,
	FG_SUBCMD_FW_VER	= 0x0002,
	FG_SUBCMD_CHEM_ID	= 0x0008,
	FG_SUBCMD_BAT_INSERT	= 0x000D,
	FG_SUBCMD_BAT_REMOVE	= 0x000E,
	FG_SUBCMD_DF_VER	= 0x001F,
	FG_SUBCMD_SEAL		= 0x0020,
	FG_SUBCMD_IT_ENABLE	= 0x0021,
	FG_SUBCMD_DEV_RESET	= 0x0041,
	FG_SUBCMD_ENTER_ROM	= 0x0F00,
};


enum {
	SEAL_STATE_FA,
	SEAL_STATE_UNSEALED,
	SEAL_STATE_SEALED,
};


enum bq_fg_device {
	BQ27532,
};

enum {
	UPDATE_REASON_FG_RESET = 1,
	UPDATE_REASON_NEW_VERSION,
	UPDATE_REASON_FORCED,
	UPDATE_REASON_IN_ROM_MODE,
};

struct fg_batt_profile {
	const bqfs_cmd_t *bqfs_image;
	u32 array_size;
	u16 version;
};

static const struct fg_batt_profile bqfs_image[] = {
	{ bqfs_coslight, ARRAY_SIZE(bqfs_coslight), 1 }, /*100*/
};

static const unsigned char *device2str[] = {
	"bq27532",
};

const unsigned char *update_reason_str[] = {
	"Reset",
	"New Version",
	"Force",
	"In Rom Mode",
};

static u8 bq27532_regs[NUM_REGS] = {
	0x00,	/* CONTROL */
	0x06,	/* TEMP */
	0x08,	/* VOLT */
	0x14,	/* AVG CURRENT */
	0x0A,	/* FLAGS */
	0xFF,	/* Time to empty */
	0xFF,	/* Time to full */
	0x12,	/* Full charge capacity */
	0x10,	/* Remaining Capacity */
	0x1E,	/* CycleCount */
	0x20,	/* State of Charge */
	0x1C,	/* State of Health */
	0xFF,	/* Design Capacity */
};

struct bq_fg_chip {
	struct device *dev;
	struct i2c_client *client;


	struct mutex i2c_rw_lock;
	struct mutex data_lock;
	struct mutex update_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	int force_update;
	int fw_ver;
	int df_ver;

	u8 chip;
	u8 regs[NUM_REGS];

	/* status tracking */

	bool batt_present;
	bool batt_fc;
	bool batt_ot;
	bool batt_ut;
	bool batt_soc1;
	bool batt_socf;
	bool batt_dsg;
	bool allow_chg;
	bool itpor;

	int seal_state; /* 0 - Full Access, 1 - Unsealed, 2 - Sealed */
	int batt_tte;
	int batt_soc;
	int batt_fcc;	/* Full charge capacity */
	int batt_rm;	/* Remaining capacity */
	int batt_dc;	/* Design Capacity */
	int batt_volt;
	int batt_temp;
	int batt_curr;

	int batt_cyclecnt;	/* cycle count */

	struct work_struct update_work;

	/* debug */
	int skip_reads;
	int skip_writes;

	int fake_soc;
	int fake_temp;

	struct dentry *debug_root;

	struct power_supply *fg_psy;
	struct power_supply_desc fg_psy_d;

	u32 connected_rid;
	int  batt_id;
};

static int __fg_read_byte(struct i2c_client *client, u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		bq_err("i2c read byte fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u8)ret;

	return 0;
}

static int __fg_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		bq_err("i2c write byte fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}


static int __fg_read_word(struct i2c_client *client, u8 reg, u16 *val)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		bq_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16)ret;

	return 0;
}


static int __fg_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		bq_err("i2c write word fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}

static int __fg_read_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{

	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, len, buf);

	return ret;
}

static int __fg_write_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, len, buf);

	return ret;
}

static int fg_read_byte(struct bq_fg_chip *bq, u8 reg, u8 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_byte(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_write_byte(struct bq_fg_chip *bq, u8 reg, u8 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_byte(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_read_word(struct bq_fg_chip *bq, u8 reg, u16 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_word(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_write_word(struct bq_fg_chip *bq, u8 reg, u16 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_word(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_read_block(struct bq_fg_chip *bq, u8 reg, u8 *buf, u8 len)
{
	int ret;

	if (bq->skip_reads)
		return 0;
	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_block(bq->client, reg, buf, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;

}

static int fg_write_block(struct bq_fg_chip *bq, u8 reg, u8 *data, u8 len)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_block(bq->client, reg, data, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static u8 checksum(u8 *data, u8 len)
{
	u8 i;
	u16 sum = 0;

	for (i = 0; i < len; i++)
		sum += data[i];

	sum &= 0xFF;

	return (0xFF - sum);
}

#if 0
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{
	int i;
	int idx = 0;
	int num;
	u8 strbuf[128];

	bq_err("%s buf: ", msg);
	for (i = 0; i < len; i++) {
		num = sprintf(&strbuf[idx], "%02X ", buf[i]);
		idx += num;
	}
	bq_err("%s\n", strbuf);
}
#else
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{}
#endif


#define TIMEOUT_INIT_COMPLETED	100
static int fg_check_init_completed(struct bq_fg_chip *bq)
{
	int ret;
	int i = 0;
	u16 status;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_CTRL], FG_SUBCMD_CTRL_STATUS);
	if (ret < 0) {
		bq_err("Failed to write control status cmd, ret = %d\n", ret);
		return ret;
	}

	msleep(5);

	while (i++ < TIMEOUT_INIT_COMPLETED) {
		ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CTRL], &status);
		if (ret >= 0 && (status & 0x0080))
			return 0;
		msleep(100);
	}
	bq_err("wait for FG INITCOMP timeout\n");
	return ret;
}

static int fg_get_seal_state(struct bq_fg_chip *bq)
{
	int ret;
	u16 status;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_CTRL], FG_SUBCMD_CTRL_STATUS);
	if (ret < 0) {
		bq_err("Failed to write control status cmd, ret = %d\n", ret);
		return ret;
	}

	msleep(5);

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CTRL], &status);
	if (ret < 0) {
		bq_err("Failed to read control status, ret = %d\n", ret);
		return ret;
	}

	if (status & 0x2000)
		bq->seal_state = SEAL_STATE_SEALED;
	else if (status & 0x4000)
		bq->seal_state = SEAL_STATE_FA;
	else
		bq->seal_state = SEAL_STATE_UNSEALED;

	return 0;
}

static int fg_unseal_send_key(struct bq_fg_chip *bq, u32 key)
{
	int ret;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_CTRL], key & 0xFFFF);

	if (ret < 0) {
		bq_err("unable to write unseal key step 1, ret = %d\n", ret);
		return ret;
	}

	msleep(5);

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_CTRL], (key >> 16) & 0xFFFF);
	if (ret < 0) {
		bq_err("unable to write unseal key step 2, ret = %d\n", ret);
		return ret;
	}

	msleep(100);

	return 0;
}

#define FG_DEFAULT_UNSEAL_KEY	0x80008000
static int fg_unseal(struct bq_fg_chip *bq)
{
	int ret;
	int retry = 0;

	ret = fg_unseal_send_key(bq, FG_DEFAULT_UNSEAL_KEY);
	if (!ret) {
		while (retry++ < 100) {
			ret = fg_get_seal_state(bq);
			if (bq->seal_state == SEAL_STATE_UNSEALED ||
			    bq->seal_state == SEAL_STATE_FA) {
				bq_log("FG is unsealed");
				return 0;
			}
		}
	}

	bq_err("unseal failed\n");

	return -1;
}


#define FG_DEFAULT_UNSEAL_FA_KEY	0x36724614
static int fg_unseal_full_access(struct bq_fg_chip *bq)
{
	int ret;
	int retry = 0;

	ret = fg_unseal_send_key(bq, FG_DEFAULT_UNSEAL_FA_KEY);
	if (!ret) {
		while (retry++ < 100) {
			fg_get_seal_state(bq);
			if (bq->seal_state == SEAL_STATE_FA) {
				bq_log("FG is in full access.");
				return 0;
			}
			msleep(200);
		}
	}

	bq_err("unseal full access failed\n");

	return -1;
}

static int fg_seal(struct bq_fg_chip *bq)
{
	int ret;
	int retry = 0;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_CTRL], FG_SUBCMD_SEAL);

	if (ret < 0) {
		bq_err("Failed to send seal command\n");
		return ret;
	}

	while (retry++ < 1000) {
		fg_get_seal_state(bq);
		if (bq->seal_state == SEAL_STATE_SEALED) {
			bq_log("FG is sealed successfully");
			return 0;
		}
		msleep(200);
	}
	bq_err("Seal device retry time out!");
	return -1;
}


static int fg_read_df_version(struct bq_fg_chip *bq, u16 *ver)
{
	int ret;
	u16 df_ver = 0;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_CTRL], FG_SUBCMD_DF_VER);
	if (ret < 0) {
		bq_err("Failed to write control status cmd, ret = %d\n", ret);
		return ret;
	}

	msleep(5);

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CTRL], &df_ver);
	if (!ret)
		*ver = df_ver;
	return ret;
}


#define	DF_ACCESS_BLOCK_DATA_CHKSUM	0x60
#define	DF_ACCESS_BLOCK_DATA_CTRL	0x61
#define	DF_ACCESS_BLOCK_DATA_CLASS	0x3E
#define	DF_ACCESS_DATA_BLOCK		0x3F
#define	DF_ACCESS_BLOCK_DATA		0x40


static int fg_df_read_block(struct bq_fg_chip *bq, u8 classid,
					u8 offset, u8 *buf, u8 len)
{
	int ret, i;
	u8 tmp_buf[40];
	u8 cksum_calc, cksum;

	if (offset % 32 + len > 32)
		return -1;

	bq_info("subclass:%d, offset:%d\n", classid, offset);

	/* access general data flash */
	ret = fg_write_byte(bq, DF_ACCESS_BLOCK_DATA_CTRL, 0);
	if (ret < 0)
		return ret;
	msleep(5);
	/* send class id */
	ret = fg_write_byte(bq, DF_ACCESS_BLOCK_DATA_CLASS, classid);
	if (ret < 0)
		return ret;
	msleep(5);
	/* send data flash block */
	ret = fg_write_byte(bq, DF_ACCESS_DATA_BLOCK, offset / 32);
	if (ret < 0)
		return ret;
	msleep(5);
	/* perform reading */
	ret = fg_read_block(bq, DF_ACCESS_BLOCK_DATA, tmp_buf, 32);
	if (ret < 0)
		return ret;

	fg_print_buf(__func__, tmp_buf, 32);

	msleep(5);
	cksum_calc = checksum(buf, 32);
	ret = fg_read_byte(bq, DF_ACCESS_BLOCK_DATA_CHKSUM, &cksum);

	if (ret < 0 || cksum_calc != cksum)
		return 1;
	/* return expected data */
	for (i = 0; i < len; i++)
		buf[i] = tmp_buf[offset % 32 + i];
	return 0;
}
EXPORT_SYMBOL_GPL(fg_df_read_block);

static int fg_df_write_block(struct bq_fg_chip *bq, u8 classid,
					u8 offset, u8 *buf, u8 len)
{
	int ret, i;
	u8 cksum;
	u8 crc_calc;
	u8 tmp_buf[64];

	bq_info("subclass:%d, offset:%d\n", classid, offset);

	if (offset % 32 + len > 32)
		return -1;

	/* Enable general flash access */
	ret = fg_write_byte(bq, DF_ACCESS_BLOCK_DATA_CTRL, 0);
	if (ret < 0)
		return ret;
	msleep(5);
	/* send class id */
	ret = fg_write_byte(bq, DF_ACCESS_BLOCK_DATA_CLASS, classid);
	if (ret < 0)
		return ret;
	msleep(5);
	/* send blk offset */
	ret = fg_write_byte(bq, DF_ACCESS_DATA_BLOCK, offset / 32);
	if (ret < 0)
		return ret;

	/* read back block data */
	ret = fg_read_block(bq, DF_ACCESS_BLOCK_DATA, tmp_buf, 32);
	if (ret < 0)
		return ret;
	ret = fg_read_byte(bq, DF_ACCESS_BLOCK_DATA_CHKSUM, &cksum);
	if (ret < 0)
		return ret;

	crc_calc = checksum(tmp_buf, 32);
	if (cksum != crc_calc)
		return -2;

	/* update buffer with new data */
	for (i = 0; i < len; i++)
		tmp_buf[offset % 32 + i] = buf[i];

	ret = fg_write_block(bq, DF_ACCESS_BLOCK_DATA, tmp_buf, 32);
	msleep(5);

	fg_print_buf(__func__, tmp_buf, 32);
	/* calculate new crc*/
	crc_calc = checksum(tmp_buf, 32);
	ret = fg_write_byte(bq, DF_ACCESS_BLOCK_DATA_CHKSUM, crc_calc);
	return ret;
}
EXPORT_SYMBOL_GPL(fg_df_write_block);

#define BQFG_I2C_ROM_ADDR    (0x16 >> 1)
#define BQFG_I2C_DEV_ADDR    (0xAA >> 1)
static bool fg_in_rom_mode(struct bq_fg_chip *bq)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	int ret;
	u8 tmp;
	/* in Rom mode, fg works with 16h address*/
	client->addr = BQFG_I2C_ROM_ADDR;
	ret = fg_read_byte(bq, 0x66, &tmp);
	mdelay(2);
	client->addr = BQFG_I2C_DEV_ADDR;/* restore address*/
	if (ret < 0) /* not in rom mode */
		return false;
	return true;
}

static bool fg_enter_rom_mode(struct bq_fg_chip *bq)
{
	int ret;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_CTRL], FG_SUBCMD_ENTER_ROM);
	if (ret < 0)
		return false;
	return fg_in_rom_mode(bq);
}


static int fg_read_fw_version(struct bq_fg_chip *bq)
{

	int ret;
	u16 version;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_CTRL], 0x0002);

	if (ret < 0) {
		bq_err("Failed to send firmware version subcommand:%d\n", ret);
		return ret;
	}

	mdelay(2);

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CTRL], &version);
	if (ret < 0) {
		bq_err("Failed to read firmware version:%d\n", ret);
		return ret;
	}

	return version;
}


static int fg_read_status(struct bq_fg_chip *bq)
{
	int ret;
	u16 flags;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_FLAGS], &flags);
	if (ret < 0)
		return ret;

	mutex_lock(&bq->data_lock);
	bq->batt_present	= !!(flags & FG_FLAGS_BAT_DET);
	bq->batt_fc		= !!(flags & FG_FLAGS_FC);
	bq->batt_soc1		= !!(flags & FG_FLAGS_SOC1);
	bq->batt_dsg		= !!(flags & FG_FLAGS_DSG);
	mutex_unlock(&bq->data_lock);

	return 0;
}


static int fg_read_rsoc(struct bq_fg_chip *bq)
{
	int ret;
	u16 soc = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOC], &soc);
	if (ret < 0) {
		bq_err("could not read RSOC, ret = %d\n", ret);
		return ret;
	}

	return soc;

}

static int fg_read_temperature(struct bq_fg_chip *bq)
{
	int ret;
	u16 temp = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TEMP], &temp);
	if (ret < 0) {
		bq_err("could not read temperature, ret = %d\n", ret);
		return ret;
	}

	return temp - 2730;

}

static int fg_read_volt(struct bq_fg_chip *bq)
{
	int ret;
	u16 volt = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_VOLT], &volt);
	if (ret < 0) {
		bq_err("could not read voltage, ret = %d\n", ret);
		return ret;
	}

	return volt;

}

static int fg_read_current(struct bq_fg_chip *bq, int *curr)
{
	int ret;
	u16 avg_curr = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_AI], &avg_curr);
	if (ret < 0) {
		bq_err("could not read current, ret = %d\n", ret);
		return ret;
	}
	*curr = (int)((s16)avg_curr);

	return ret;
}

static int fg_read_fcc(struct bq_fg_chip *bq)
{
	int ret;
	u16 fcc;

	if (bq->regs[BQ_FG_REG_FCC] == INVALID_REG_ADDR) {
		bq_err("FCC command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_FCC], &fcc);

	if (ret < 0)
		bq_err("could not read FCC, ret=%d\n", ret);

	return fcc;
}

static int fg_read_dc(struct bq_fg_chip *bq)
{

	int ret;
	u16 dc;

	if (bq->regs[BQ_FG_REG_DC] == INVALID_REG_ADDR) {
		bq_err("DesignCapacity command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_DC], &dc);

	if (ret < 0) {
		bq_err("could not read DC, ret=%d\n", ret);
		return ret;
	}

	return dc;
}


static int fg_read_rm(struct bq_fg_chip *bq)
{
	int ret;
	u16 rm;

	if (bq->regs[BQ_FG_REG_RM] == INVALID_REG_ADDR) {
		bq_err("RemainingCapacity command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_RM], &rm);

	if (ret < 0) {
		bq_err("could not read DC, ret=%d\n", ret);
		return ret;
	}

	return rm;

}

static int fg_read_cyclecount(struct bq_fg_chip *bq)
{
	int ret;
	u16 cc;

	if (bq->regs[BQ_FG_REG_CC] == INVALID_REG_ADDR) {
		bq_err("Cycle Count not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CC], &cc);

	if (ret < 0) {
		bq_err("could not read Cycle Count, ret=%d\n", ret);
		return ret;
	}

	return cc;
}

static int fg_read_tte(struct bq_fg_chip *bq)
{
	int ret;
	u16 tte;

	if (bq->regs[BQ_FG_REG_TTE] == INVALID_REG_ADDR) {
		bq_err("Time To Empty not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TTE], &tte);

	if (ret < 0) {
		bq_err("could not read Time To Empty, ret=%d\n", ret);
		return ret;
	}

	if (ret == 0xFFFF)
		return -ENODATA;

	return tte;
}

static int fg_get_batt_status(struct bq_fg_chip *bq)
{

	fg_read_status(bq);

	if (!bq->batt_present)
		return POWER_SUPPLY_STATUS_UNKNOWN;
	else if (bq->batt_fc)
		return POWER_SUPPLY_STATUS_FULL;
	else if (bq->batt_dsg)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else if (bq->batt_curr > 0)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

}


static int fg_get_batt_capacity_level(struct bq_fg_chip *bq)
{
	if (!bq->batt_present)
		return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	else if (bq->batt_fc)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (bq->batt_soc1)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (bq->batt_soc < 3)/*TODO*/
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

}


static int fg_get_batt_health(struct bq_fg_chip *bq)
{
	if (!bq->batt_present)
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	else if (bq->batt_ot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (bq->batt_ut)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;

}

static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
/*	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,*/
	POWER_SUPPLY_PROP_CHARGE_FULL,
/*	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,*/
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_UPDATE_NOW,
};

static int fg_get_property(struct power_supply *psy, enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&bq->update_lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fg_get_batt_status(bq);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = fg_read_volt(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_volt = ret;
		val->intval = bq->batt_volt * 1000;
		mutex_unlock(&bq->data_lock);

		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq->batt_present;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		mutex_lock(&bq->data_lock);
		fg_read_current(bq, &bq->batt_curr);
		val->intval = -bq->batt_curr * 1000;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (bq->fake_soc >= 0) {
			val->intval = bq->fake_soc;
			break;
		}
		ret = fg_read_rsoc(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_soc = ret;
		val->intval = bq->batt_soc;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = fg_get_batt_capacity_level(bq);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		if (bq->fake_temp != -EINVAL) {
			val->intval = bq->fake_temp;
			break;
		}
		ret = fg_read_temperature(bq);
		mutex_lock(&bq->data_lock);
		if (ret > 0)
			bq->batt_temp = ret;
		val->intval = bq->batt_temp;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = fg_read_tte(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_tte = ret;

		val->intval = bq->batt_tte;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = fg_read_fcc(bq);
		mutex_lock(&bq->data_lock);
		if (ret > 0)
			bq->batt_fcc = ret;
		val->intval = bq->batt_fcc * 1000;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = fg_read_dc(bq);
		mutex_lock(&bq->data_lock);
		if (ret > 0)
			bq->batt_dc = ret;
		val->intval = bq->batt_dc * 1000;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = fg_read_cyclecount(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_cyclecnt = ret;
		val->intval = bq->batt_cyclecnt;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = fg_get_batt_health(bq);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = bq->connected_rid ;
		break;
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		val->intval = 0;
		break;

	default:
		mutex_unlock(&bq->update_lock);
		return -EINVAL;
	}
	mutex_unlock(&bq->update_lock);
	return 0;
}
static void fg_dump_registers(struct bq_fg_chip *bq);

static int fg_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
		bq->fake_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		bq->fake_soc = val->intval;
		power_supply_changed(bq->fg_psy);
		break;
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		fg_dump_registers(bq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int fg_prop_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int fg_psy_register(struct bq_fg_chip *bq)
{
	struct power_supply_config fg_psy_cfg = {};

	bq->fg_psy_d.name = "bms";
	bq->fg_psy_d.type = POWER_SUPPLY_TYPE_BMS;
	bq->fg_psy_d.properties = fg_props;
	bq->fg_psy_d.num_properties = ARRAY_SIZE(fg_props);
	bq->fg_psy_d.get_property = fg_get_property;
	bq->fg_psy_d.set_property = fg_set_property;
	bq->fg_psy_d.property_is_writeable = fg_prop_is_writeable;

	fg_psy_cfg.drv_data = bq;
	fg_psy_cfg.num_supplicants = 0;
	bq->fg_psy = devm_power_supply_register(bq->dev,
						&bq->fg_psy_d,
						&fg_psy_cfg);
	if (IS_ERR(bq->fg_psy)) {
		bq_err("Failed to register fg_psy");
		return PTR_ERR(bq->fg_psy);
	}
	return 0;
}


static void fg_psy_unregister(struct bq_fg_chip *bq)
{

	power_supply_unregister(bq->fg_psy);
}

static int fg_check_update_necessary(struct bq_fg_chip *bq)
{
	int ret;
	u16 dm_ver = 0;

	if (fg_in_rom_mode(bq))
		return UPDATE_REASON_IN_ROM_MODE;

	ret = fg_read_df_version(bq, &dm_ver);
	if (!ret && dm_ver < bqfs_image[bq->batt_id].version)
		return UPDATE_REASON_NEW_VERSION;
	else
		return 0;
}

static bool fg_update_bqfs_execute_cmd(struct bq_fg_chip *bq,
					const bqfs_cmd_t *cmd)
{
	int ret;
	int i;
	u8 tmp_buf[CMD_MAX_DATA_SIZE];

	switch (cmd->cmd_type) {
	case CMD_R:
		ret = fg_read_block(bq, cmd->reg,
					(u8 *)&cmd->data.bytes,
					cmd->data_len);
		if (ret < 0)
			return false;
		else
			return true;
		break;
	case CMD_W:
		ret = fg_write_block(bq, cmd->reg,
					(u8 *)&cmd->data.bytes,
					cmd->data_len);
		if (ret < 0)
			return false;
		else
			return true;
		break;
	case CMD_C:
		if (fg_read_block(bq, cmd->reg, tmp_buf, cmd->data_len) < 0)
			return false;
		if (memcmp(tmp_buf, cmd->data.bytes, cmd->data_len)) {
			bq_info("CMD_C failed at line %d\n", cmd->line_num);
			for (i = 0; i < cmd->data_len; i++)
				bq_err("Read: %02X, Cmp:%02X",
					tmp_buf[i], cmd->data.bytes[i]);
			return false;
		}

		return true;
		/*break;*/
	case CMD_X:
		mdelay(cmd->data.delay);
		return true;
		/*break;*/
	default:
		bq_err("Unsupported command at line %d\n", cmd->line_num);
		return false;
	}

}

static void fg_update_bqfs(struct bq_fg_chip *bq)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	const bqfs_cmd_t *image;
	int reason = 0;
	int i;

	if (bq->force_update == ~BQFS_UPDATE_KEY)
		reason = UPDATE_REASON_FORCED;
	else
		reason = fg_check_update_necessary(bq);

	if (!reason) {
		bq_log("Fuel Gauge parameter no need update, ignored\n");
		return;
	}

	if (bq->batt_id >= ARRAY_SIZE(bqfs_image) ||
		bq->batt_id < 0) {
		bq_err("batt_id is out of range");
		return;
	}

	bq_log("Fuel Gauge parameter update, reason:%s, version:%d, batt_id=%d Start...\n",
			update_reason_str[reason - 1], bqfs_image[bq->batt_id].version, bq->batt_id);

	/* check if in rom mode, if not, enter rom mode*/
	if (reason != UPDATE_REASON_IN_ROM_MODE) {
		fg_get_seal_state(bq);
		if (bq->seal_state != SEAL_STATE_FA) {
			if (bq->seal_state == SEAL_STATE_SEALED) {
				if (fg_unseal(bq))
					return;
			}
			mdelay(10);
			if (fg_unseal_full_access(bq))
				return;
		}
		if (!fg_enter_rom_mode(bq)) {
			bq_err("Failed to enter rom mode");
			return;
		}
	}
	/* write i2c stream to device*/
	mutex_lock(&bq->update_lock);
	client->addr = BQFG_I2C_ROM_ADDR;
	image = bqfs_image[bq->batt_id].bqfs_image;
	for (i = 0; i < bqfs_image[bq->batt_id].array_size; i++) {
		if (!fg_update_bqfs_execute_cmd(bq, &image[i])) {
			mutex_unlock(&bq->update_lock);
			bq_err("Failed at command: %d\n", i);
			return;
		}
		mdelay(5);
	}
	mutex_unlock(&bq->update_lock);

	client->addr = BQFG_I2C_DEV_ADDR;

	bq_log("Done!\n");

	return;

}
EXPORT_SYMBOL_GPL(fg_update_bqfs);

static const u8 fg_dump_regs[] = {
	0x00, 0x02, 0x04, 0x06,
	0x08, 0x0A, 0x0C, 0x0E,
	0x10, 0x16, 0x18, 0x1A,
	0x1C, 0x1E, 0x20, 0x28,
	0x2A, 0x2C, 0x2E, 0x30,
	0x66, 0x68, 0x6C, 0x6E,
	0x70,
};

static int show_registers(struct seq_file *m, void *data)
{
	struct bq_fg_chip *bq = m->private;
	int i;
	int ret;
	u16 val = 0;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		msleep(5);
		ret = fg_read_word(bq, fg_dump_regs[i], &val);
		if (!ret)
			seq_printf(m, "Reg[%02X] = 0x%04X\n",
						fg_dump_regs[i], val);
	}
	return 0;
}


static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq_fg_chip *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}

static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct bq_fg_chip *bq)
{
	bq->debug_root = debugfs_create_dir("bq_fg", NULL);
	if (!bq->debug_root)
		bq_err("Failed to create debug dir\n");

	if (bq->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
						bq->debug_root,
						bq, &reg_debugfs_ops);

		debugfs_create_x32("fake_soc",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->fake_soc));

		debugfs_create_x32("fake_temp",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->fake_temp));

		debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_reads));
		debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_writes));
	}
}

static ssize_t fg_attr_show_Ra_table(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	int ret;
	u8 rd_buf[64];
	u8 temp_buf[100];
	int len, i, idx;
	int last_seal_status;

	fg_get_seal_state(bq);
	last_seal_status = bq->seal_state;

	idx = 0;

	mutex_lock(&bq->update_lock);
	if (bq->seal_state == SEAL_STATE_SEALED) {
		ret = fg_unseal(bq);
		if (ret)
			goto exit;
	}

	ret = fg_df_read_block(bq, 91, 0, rd_buf, 20);	/*Ra Table*/
	if (ret)
		goto exit_0;

	len = sprintf(temp_buf, "Ra Status:0x%02X\n", rd_buf[0]);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	len = sprintf(temp_buf, "Ra Flag:0x%02X\n", rd_buf[1]);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	len = sprintf(temp_buf, "Ra Base R:%d\n", rd_buf[2] << 8 | rd_buf[3]);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	len = sprintf(temp_buf, "Ra Gain:0x%02X\n", rd_buf[4]);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	len = sprintf(temp_buf, "Ra Table:\n");
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	for (i = 1; i <= 14; i++) {
		len = sprintf(temp_buf, "%d ", rd_buf[4+i]);
		memcpy(&buf[idx], temp_buf, len);
		idx += len;
	}
exit_0:
	if (last_seal_status == SEAL_STATE_SEALED)
		fg_seal(bq);
exit:
	mutex_unlock(&bq->update_lock);
	return idx;
}

static ssize_t fg_attr_show_Qmax(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	int ret;
	u8 rd_buf[64];
	int len;
	int last_seal_status;

	fg_get_seal_state(bq);
	last_seal_status = bq->seal_state;

	len = 0;

	mutex_lock(&bq->update_lock);

	if (bq->seal_state == SEAL_STATE_SEALED) {
		ret = fg_unseal(bq);
		if (ret)
			goto exit;
	}

	ret = fg_df_read_block(bq, 82, 1, rd_buf, 2);
	if (ret)
		goto exit_0;

	len = sprintf(buf, "Qmax Cell 0 = %d\n", (rd_buf[0] << 8) | rd_buf[1]);

exit_0:
	if (last_seal_status == SEAL_STATE_SEALED)
		fg_seal(bq);
exit:
	mutex_unlock(&bq->update_lock);
	return len;
}

static ssize_t fg_attr_store_update(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	unsigned int key = 0;

	sscanf(buf, "%x", &key);
	if (key == BQFS_UPDATE_KEY) {
		bq->force_update = ~key;
		schedule_work(&bq->update_work);
	}
	return count;
}

static ssize_t fg_attr_show_df_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	int ret;
	u16 ver;

	ret = fg_read_df_version(bq, &ver);
	if (!ret)
		return sprintf(buf, "0x%04X\n", ver);
	else
		return sprintf(buf, "Read DF Version error");
}



static DEVICE_ATTR(RaTable, S_IRUGO, fg_attr_show_Ra_table, NULL);
static DEVICE_ATTR(Qmax, S_IRUGO, fg_attr_show_Qmax, NULL);
static DEVICE_ATTR(update, S_IWUSR, NULL, fg_attr_store_update);
static DEVICE_ATTR(df_version, S_IRUGO, fg_attr_show_df_version, NULL);

static struct attribute *fg_attributes[] = {
	&dev_attr_RaTable.attr,
	&dev_attr_Qmax.attr,
	&dev_attr_update.attr,
	&dev_attr_df_version.attr,
	NULL,
};

static const struct attribute_group fg_attr_group = {
	.attrs = fg_attributes,
};

static void fg_update_bqfs_workfunc(struct work_struct *work)
{
	struct bq_fg_chip *bq = container_of(work,
					struct bq_fg_chip, update_work);

	fg_update_bqfs(bq);
}

static void fg_dump_registers(struct bq_fg_chip *bq)
{
	int i;
	int ret;
	u16 val;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		msleep(5);
		ret = fg_read_word(bq, fg_dump_regs[i], &val);
		if (!ret)
			bq_err("Reg[%02X] = 0x%04X\n", fg_dump_regs[i], val);
	}
}

static irqreturn_t fg_irq_thread(int irq, void *dev_id)
{
	struct bq_fg_chip *bq = dev_id;
	bool last_batt_present;

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		bq_info("IRQ triggered before device resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;

	last_batt_present = bq->batt_present;

	mutex_lock(&bq->update_lock);
	fg_read_status(bq);
	mutex_unlock(&bq->update_lock);

	fg_dump_registers(bq);

	bq_info("seal_state=%d, batt_present=%d",
		bq->seal_state, bq->batt_present);

	if (!last_batt_present && bq->batt_present) {/* battery inserted */
		bq_log("Battery inserted\n");
	} else if (last_batt_present && !bq->batt_present) {/* battery removed */
		bq_log("Battery removed\n");
		bq->batt_soc	= -ENODATA;
		bq->batt_fcc	= -ENODATA;
		bq->batt_rm	= -ENODATA;
		bq->batt_volt	= -ENODATA;
		bq->batt_curr	= -ENODATA;
		bq->batt_temp	= -ENODATA;
		bq->batt_cyclecnt = -ENODATA;
	}

	if (bq->batt_present) {
		mutex_lock(&bq->update_lock);

		bq->batt_soc = fg_read_rsoc(bq);
		bq->batt_volt = fg_read_volt(bq);
		fg_read_current(bq, &bq->batt_curr);
		bq->batt_temp = fg_read_temperature(bq);
		bq->batt_rm = fg_read_rm(bq);

		mutex_unlock(&bq->update_lock);
		bq_err("RSOC:%d, Volt:%d, Current:%d, Temperature:%d\n",
			bq->batt_soc, bq->batt_volt, bq->batt_curr, bq->batt_temp);
	}

	power_supply_changed(bq->fg_psy);
	mutex_unlock(&bq->irq_complete);

	return IRQ_HANDLED;
}


static void determine_initial_status(struct bq_fg_chip *bq)
{
	fg_irq_thread(bq->client->irq, bq);
}

static void convert_rid2battid(struct bq_fg_chip *bq)
{
	/* TODO: here is just an example, modify it accordingly*/
#if 0
	if (bq->connected_rid > 220 && bq->connected_rid < 440)
		bq->batt_id = 2;
	else if (bq->connected_rid > 60 && bq->connected_rid < 140)
		bq->batt_id = 1;
	else if (bq->connected_rid > 10 && bq->connected_rid < 50)
		bq->batt_id = 0;
	else
		bq->batt_id = 1;
#endif
	bq->batt_id = 0;
}


static int fg_parse_batt_id(struct bq_fg_chip *bq)
{
	/*TODO: read battery id pin voltage, calcuate resisotr ID,
	 * store it in bq->connected_rid_*/
	convert_rid2battid(bq);

	return 0;
}

static int bq_fg_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{

	int ret;
	struct bq_fg_chip *bq;
	u8 *regs;

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);

	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;
	bq->chip = id->driver_data;

	bq->batt_soc	= -ENODATA;
	bq->batt_fcc	= -ENODATA;
	bq->batt_rm	= -ENODATA;
	bq->batt_dc	= -ENODATA;
	bq->batt_volt	= -ENODATA;
	bq->batt_temp	= -ENODATA;
	bq->batt_curr	= -ENODATA;
	bq->batt_cyclecnt = -ENODATA;

	bq->fake_soc	= -EINVAL;
	bq->fake_temp	= -EINVAL;

	if (bq->chip == BQ27532) {
		regs = bq27532_regs;
	} else {
		bq_err("unexpected fuel gauge: %d\n", bq->chip);
		regs = bq27532_regs;
	}

	memcpy(bq->regs, regs, NUM_REGS);

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->update_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	INIT_WORK(&bq->update_work, fg_update_bqfs_workfunc);

	fg_parse_batt_id(bq);

	fg_update_bqfs(bq);

	fg_check_init_completed(bq);

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
			fg_irq_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"bq fuel gauge irq", bq);
		if (ret < 0) {
			bq_err("request irq for irq=%d failed, ret = %d\n",
						client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
	}

	device_init_wakeup(bq->dev, 1);

	bq->fw_ver = fg_read_fw_version(bq);

	fg_psy_register(bq);

	create_debugfs_entry(bq);
	ret = sysfs_create_group(&bq->dev->kobj, &fg_attr_group);
	if (ret)
		bq_err("Failed to register sysfs, err:%d\n", ret);

	determine_initial_status(bq);

	bq_err("bq fuel gauge probe successfully, %s FW ver:%d\n",
			device2str[bq->chip], bq->fw_ver);

	return 0;

err_1:
	fg_psy_unregister(bq);
	return ret;
}


static inline bool is_device_suspended(struct bq_fg_chip *bq)
{
	return !bq->resume_completed;
}


static int bq_fg_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);

	return 0;
}

static int bq_fg_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;

}


static int bq_fg_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		fg_irq_thread(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(bq->fg_psy);

	return 0;


}

static int bq_fg_remove(struct i2c_client *client)
{
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	fg_psy_unregister(bq);

	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->update_lock);
	mutex_destroy(&bq->irq_complete);

	debugfs_remove_recursive(bq->debug_root);
	sysfs_remove_group(&bq->dev->kobj, &fg_attr_group);

	return 0;

}

static void bq_fg_shutdown(struct i2c_client *client)
{
	bq_info("bq fuel gauge driver shutdown!\n");
}

static const struct of_device_id bq_fg_match_table[] = {
	{.compatible = "ti,bq27532",},
	{},
};
MODULE_DEVICE_TABLE(of, bq_fg_match_table);

static const struct i2c_device_id bq_fg_id[] = {
	{ "bq27532", BQ27532 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq_fg_id);

static const struct dev_pm_ops bq_fg_pm_ops = {
	.resume		= bq_fg_resume,
	.suspend_noirq = bq_fg_suspend_noirq,
	.suspend	= bq_fg_suspend,
};

static struct i2c_driver bq_fg_driver = {
	.driver = {
		.name	= "bq_fg",
		.owner	= THIS_MODULE,
		.of_match_table = bq_fg_match_table,
		.pm	= &bq_fg_pm_ops,
	},
	.id_table	= bq_fg_id,

	.probe		= bq_fg_probe,
	.remove		= bq_fg_remove,
	.shutdown	= bq_fg_shutdown,

};

module_i2c_driver(bq_fg_driver);

MODULE_DESCRIPTION("TI BQ2742x Gauge Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
