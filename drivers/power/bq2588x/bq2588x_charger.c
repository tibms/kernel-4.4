/*
 * BQ25700 battery charging driver
 *
 * Copyright (C) 2017 Texas Instruments *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


#define pr_fmt(fmt)	"[bq2588x] %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/alarmtimer.h>

#include "bq2588x_reg.h"
#include "bq2588x.h"

#define	bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_err
#define bq_log	pr_err

enum {
	USER		= BIT(0),
	JEITA		= BIT(1),
	BATT_FC		= BIT(2),		/* Batt FULL */
	BATT_PRES	= BIT(3),
};


enum {
	BQ25880 = 0x01,
	BQ25882 = 0x02
};

enum {
	CHARGE_STATE_NOT_CHARGING,
	CHARGE_STATE_PRECHARGE,
	CHARGE_STATE_FASTCHARGE,
	CHARGE_STATE_OTG,
};

enum wakeup_src {
	WAKEUP_SRC_MONITOR = 0,
	WAKEUP_SRC_JEITA,
	WAKEUP_SRC_MAX,
};

enum int_mask {
	INT_MASK_ADC_DONE	= 0x00000080,
	INT_MASK_IINDPM		= 0x00000040,
	INT_MASK_VINDPM		= 0x00000020,
	INT_MASK_TREG		= 0x00000010,
	INT_MASK_WDT		= 0x00000008,

	INT_MASK_PG		= 0x00008000,
	INT_MASK_VBUS		= 0x00001000,
	INT_MASK_TS		= 0x00000400,
	INT_MASK_ICO		= 0x00000200,
	INT_MASK_VSYS		= 0x00000100,

	INT_MASK_VBUS_OVP	= 0x00800000,
	INT_MASK_TSHUT		= 0x00400000,
	INT_MASK_BAT_OVP	= 0x00200000,
	INT_MASK_TMR		= 0x00100000,
	INT_MASK_SYS_SHORT	= 0x00080000,
	INT_MASK_OTG		= 0x00010000,
};


#define WAKEUP_SRC_MASK (~(~0 << WAKEUP_SRC_MAX))
struct bq2588x_wakeup_source {
	struct wakeup_source source;
	unsigned long enabled_bitmap;
	spinlock_t ws_lock;
};


struct bq2588x_otg_regulator {
	struct regulator_desc rdesc;
	struct regulator_dev *rdev;
};


struct bq2588x {
	struct device *dev;
	struct i2c_client *client;

	int part_no;
	int revision;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex profile_change_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;

	struct bq2588x_wakeup_source bq2588x_ws;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool usb_present;

	bool power_good;

	bool batt_full;

	int vbus_volt;
	int vbat_volt;
	int vsys_volt;
	int ibus_curr;
	int ichg_curr;
	int die_temp;
	int ts_temp;

	bool software_jeita_supported;
	bool jeita_active;

	bool batt_hot;
	bool batt_cold;
	bool batt_warm;
	bool batt_cool;

	int batt_hot_degc;
	int batt_warm_degc;
	int batt_cool_degc;
	int batt_cold_degc;
	int hot_temp_hysteresis;
	int cold_temp_hysteresis;

	int batt_cool_ma;
	int batt_warm_ma;
	int batt_cool_mv;
	int batt_warm_mv;

	int batt_temp;

	int jeita_ma;
	int jeita_mv;

	int usb_psy_ma;
	int charge_state;
	int charging_disabled_status;


	bool charge_enabled;	/* Register bit status */
	bool otg_enabled;
	bool vindpm_triggered;
	bool iindpm_triggered;

	int icl_ma;
	int ivl_mv;

	int chg_ma;
	int chg_mv;

	int fault_status;

	int dev_id;

	enum power_supply_type supply_type;

	struct delayed_work monitor_work;

	struct bq2588x_otg_regulator otg_vreg;

	int skip_writes;
	int skip_reads;

	struct bq2588x_platform_data *platform_data;

	struct delayed_work discharge_jeita_work; /*normal no charge mode*/
	struct delayed_work charge_jeita_work; /*charge mode jeita work*/

	struct alarm jeita_alarm;

	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply *batt_psy;
	struct power_supply_desc batt_psy_d;
};

enum {
	ADC_IBUS,
	ADC_ICHG,
	ADC_VBUS,
	ADC_VBAT,
	ADC_VSYS,
	ADC_TS,
	ADC_TDIE,
	ADC_MAX_NUM
};


static int __bq2588x_read_reg(struct bq2588x *bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		pm_relax(bq->dev);
		return ret;
	}

	*data = (u8)ret;


	return 0;
}

static int __bq2588x_write_reg(struct bq2588x *bq, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		bq_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				val, reg, ret);
		pm_relax(bq->dev);
		return ret;
	}


	return 0;
}

static int bq2588x_read_reg(struct bq2588x *bq, u8 reg, u8 *data)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2588x_read_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2588x_read_word(struct bq2588x *bq, u8 reg, u16 *data)
{
	s32 ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = i2c_smbus_read_word_data(bq->client, reg);
	mutex_unlock(&bq->i2c_rw_lock);
	if (ret >= 0)
		*data = (u16)ret;

	return ret;
}

static int bq2588x_write_reg(struct bq2588x *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2588x_write_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	if (ret)
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}


static int bq2588x_update_bits(struct bq2588x *bq, u8 reg,
					u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2588x_read_reg(bq, reg, &tmp);
	if (ret) {
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2588x_write_reg(bq, reg, tmp);
	if (ret)
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static void bq2588x_stay_awake(struct bq2588x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);

	if (!__test_and_set_bit(wk_src, &source->enabled_bitmap)) {
		__pm_stay_awake(&source->source);
		pr_debug("enabled source %s, wakeup_src %d\n",
			source->source.name, wk_src);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);
}

static void bq2588x_relax(struct bq2588x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);
	if (__test_and_clear_bit(wk_src, &source->enabled_bitmap) &&
		!(source->enabled_bitmap & WAKEUP_SRC_MASK)) {
		__pm_relax(&source->source);
		pr_debug("disabled source %s\n", source->source.name);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);

	pr_debug("relax source %s, wakeup_src %d\n",
		source->source.name, wk_src);
}

static void bq2588x_wakeup_src_init(struct bq2588x *bq)
{
	spin_lock_init(&bq->bq2588x_ws.ws_lock);
	wakeup_source_init(&bq->bq2588x_ws.source, "bq2588x");
}


static int bq2588x_set_charge_voltage(struct bq2588x *bq, int volt)
{
	int ret;
	u8 reg_val;

	if (volt < BQ2588X_VREG_BASE)
		volt = BQ2588X_VREG_BASE;

	volt -= BQ2588X_VREG_BASE;
	reg_val = volt / BQ2588X_VREG_LSB;
	reg_val <<= BQ2588X_VREG_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHARGE_VOLT,
				BQ2588X_VREG_MASK, reg_val);

	return ret;
}

static int bq2588x_set_hiz_mode(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_HIZ_DISABLE;
	else
		reg_val = BQ2588X_HIZ_ENABLE;

	reg_val <<= BQ2588X_EN_HIZ_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHARGE_CURRENT,
				BQ2588X_EN_HIZ_MASK, reg_val);

	return ret;
}


static int bq2588x_get_hiz_mode(struct bq2588x *bq, u8 *status)
{
	int ret;
	u8 reg_val;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHARGE_CURRENT, &reg_val);

	if (!ret)
		*status = reg_val & BQ2588X_EN_HIZ_MASK;

	return ret;
}

static int bq2588x_set_ilim_pin(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_ILIM_PIN_DISABLE;
	else
		reg_val = BQ2588X_ILIM_PIN_ENABLE;

	reg_val <<= BQ2588X_EN_ILIM_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHARGE_CURRENT,
				BQ2588X_EN_ILIM_MASK, reg_val);

	return ret;
}

static int bq2588x_set_charge_current(struct bq2588x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2588X_ICHG_BASE)
		curr = BQ2588X_ICHG_BASE;

	curr -= BQ2588X_ICHG_BASE;
	reg_val = curr / BQ2588X_ICHG_LSB;
	reg_val <<= BQ2588X_ICHG_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHARGE_CURRENT,
				BQ2588X_ICHG_MASK, reg_val);

	return ret;
}


static int bq2588x_set_input_volt_limit(struct bq2588x *bq, int volt)
{
	int ret;
	u8 reg_val;

	if (volt < BQ2588X_VINDPM_TH_BASE)
		volt = BQ2588X_VINDPM_TH_BASE;

	volt -= BQ2588X_VINDPM_TH_BASE;
	reg_val = volt / BQ2588X_VINDPM_TH_LSB;
	reg_val <<= BQ2588X_VINDPM_TH_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_VINDPM,
				BQ2588X_VINDPM_TH_MASK, reg_val);

	return ret;

}


static int bq2588x_set_ico_mode(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_ICO_DISABLE;
	else
		reg_val = BQ2588X_ICO_ENABLE;

	reg_val <<= BQ2588X_EN_ICO_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_IINDPM,
				BQ2588X_EN_ICO_MASK, reg_val);

	return ret;
}

static int bq2588x_set_input_current_limit(struct bq2588x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2588X_IINDPM_TH_BASE)
		curr = BQ2588X_IINDPM_TH_BASE;

	curr -= BQ2588X_IINDPM_TH_BASE;
	reg_val = curr / BQ2588X_IINDPM_TH_LSB;
	reg_val <<= BQ2588X_IINDPM_TH_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_IINDPM,
				BQ2588X_IINDPM_TH_MASK, reg_val);
	return ret;
}


static int bq2588x_set_prechg_current(struct bq2588x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2588X_IPRECHG_BASE)
		curr = BQ2588X_IPRECHG_BASE;

	curr -= BQ2588X_IPRECHG_BASE;
	reg_val = curr / BQ2588X_IPRECHG_LSB;
	reg_val <<= BQ2588X_IPRECHG_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_PRECHG_TERM,
				BQ2588X_IPRECHG_MASK, reg_val);

	return ret;
}

static int bq2588x_set_term_current(struct bq2588x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2588X_ITERM_BASE)
		curr = BQ2588X_ITERM_BASE;

	curr -= BQ2588X_ITERM_BASE;
	reg_val = curr / BQ2588X_ITERM_LSB;
	reg_val <<= BQ2588X_ITERM_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_PRECHG_TERM,
				BQ2588X_ITERM_MASK, reg_val);

	return ret;
}

static int bq2588x_set_wdt_timer(struct bq2588x *bq, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = BQ2588X_WDT_TIMER_DISABLE;
	else if (time == 40)
		reg_val = BQ2588X_WDT_TIMER_40S;
	else if (time == 80)
		reg_val = BQ2588X_WDT_TIMER_80S;
	else
		reg_val = BQ2588X_WDT_TIMER_160S;

	reg_val <<= BQ2588X_WDT_TIMER_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL1,
				BQ2588X_WDT_TIMER_MASK, reg_val);

	return ret;
}

static int bq2588x_enable_safety_timer(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_SAFETY_TIMER_DISABLE;
	else
		reg_val = BQ2588X_SAFETY_TIMER_ENABLE;

	reg_val <<= BQ2588X_SAFETY_TIMER_EN_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL1,
				BQ2588X_SAFETY_TIMER_EN_MASK, reg_val);

	return ret;
}

static int bq2588x_set_safety_timer(struct bq2588x *bq, int time)
{
	int ret;
	u8 reg_val;

	if (time == 5)
		reg_val = BQ2588X_SAFETY_TIMER_5H;
	else if (time == 8)
		reg_val = BQ2588X_SAFETY_TIMER_8H;
	else if (time == 12)
		reg_val = BQ2588X_SAFETY_TIMER_12H;
	else
		reg_val = BQ2588X_SAFETY_TIMER_20H;

	reg_val <<= BQ2588X_SAFETY_TIMER_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL1,
				BQ2588X_SAFETY_TIMER_MASK, reg_val);

	return ret;
}

static int bq2588x_charge_enable(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_CHARGE_DISABLE;
	else
		reg_val = BQ2588X_CHARGE_ENABLE;

	reg_val <<= BQ2588X_CHARGE_EN_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL2,
				BQ2588X_CHARGE_EN_MASK, reg_val);

	return ret;
}


static int bq2588x_otg_enable(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_OTG_DISABLE;
	else
		reg_val = BQ2588X_OTG_ENABLE;

	reg_val <<= BQ2588X_OTG_EN_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL2,
				BQ2588X_OTG_EN_MASK, reg_val);

	return ret;

}

static int bq2588x_reset_wdt(struct bq2588x *bq)
{
	int ret;
	u8 reg_val;

	reg_val = BQ2588X_WDT_RESET << BQ2588X_WDT_RESET_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL3,
				BQ2588X_WDT_RESET_MASK, reg_val);

	return ret;
}

static int bq2588x_set_topoff_timer(struct bq2588x *bq, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = BQ2588X_TOPOFF_TIMER_DISABLE;
	else if (time == 15)
		reg_val = BQ2588X_TOPOFF_TIMER_15M;
	else if (time == 30)
		reg_val = BQ2588X_TOPOFF_TIMER_30M;
	else
		reg_val = BQ2588X_TOPOFF_TIMER_45M;

	reg_val <<= BQ2588X_TOPOFF_TIMER_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL3,
				BQ2588X_TOPOFF_TIMER_MASK, reg_val);

	return ret;
}

static int bq2588x_set_sys_min_volt(struct bq2588x *bq, int volt)
{
	int ret;
	u8 reg_val;

	if (volt < BQ2588X_SYS_MIN_VOLT_BASE)
		volt = BQ2588X_SYS_MIN_VOLT_BASE;

	volt -= BQ2588X_SYS_MIN_VOLT_BASE;
	reg_val = volt / BQ2588X_SYS_MIN_VOLT_LSB;
	reg_val <<= BQ2588X_SYS_MIN_VOLT_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL3,
				BQ2588X_SYS_MIN_VOLT_MASK, reg_val);

	return ret;
}

static int bq2588x_set_otg_current_limit(struct bq2588x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2588X_OTG_ILIM_BASE)
		curr = BQ2588X_OTG_ILIM_BASE;

	curr -= BQ2588X_OTG_ILIM_BASE;
	reg_val = curr / BQ2588X_OTG_ILIM_LSB;
	reg_val <<= BQ2588X_OTG_ILIM_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_OTG_CTRL,
				BQ2588X_OTG_ILIM_MASK, reg_val);

	return ret;
}

static int bq2588x_set_otg_volt_limit(struct bq2588x *bq, int volt)
{
	int ret;
	u8 reg_val;

	if (volt < BQ2588X_OTG_VLIM_BASE)
		volt = BQ2588X_OTG_VLIM_BASE;

	volt -= BQ2588X_OTG_VLIM_BASE;
	reg_val = volt / BQ2588X_OTG_VLIM_LSB;
	reg_val <<= BQ2588X_OTG_VLIM_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_OTG_CTRL,
				BQ2588X_OTG_VLIM_MASK, reg_val);

	return ret;
}

static int bq2588x_get_ico_limit(struct bq2588x *bq, int *curr)
{
	int ret;
	u8 reg_val;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_ICO_LIMIT, &reg_val);
	if (ret)
		return ret;

	*curr = reg_val & BQ2588X_ICO_ILIM_MASK;
	*curr >>= BQ2588X_ICO_ILIM_SHIFT;
	*curr *= BQ2588X_ICO_ILIM_LSB;
	*curr += BQ2588X_ICO_ILIM_BASE;

	return 0;
}


static int bq2588x_set_int_mask(struct bq2588x *bq, unsigned int mask)
{
	int ret;
	u8 reg_val;

	reg_val = mask & 0xFF;
	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_INT_MASK1, 0xFF, reg_val);
	if (!ret)
		return ret;

	reg_val = (mask >> 8) & 0xFF;
	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_INT_MASK2, 0xFF, reg_val);
	if (!ret)
		return ret;

	reg_val = (mask >> 16) & 0xFF;
	ret = bq2588x_update_bits(bq, BQ2588X_REG_FAULT_INT_MASK, 0xFF, reg_val);

	return ret;
}

static int bq2588x_enable_adc_scan(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_ADC_SCAN_DISABLE;
	else
		reg_val = BQ2588X_ADC_SCAN_ENABLE;

	reg_val <<= BQ2588X_ADC_SCAN_EN_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_ADC_CTRL,
				BQ2588X_ADC_SCAN_EN_MASK, reg_val);

	return ret;
}

static int bq2588x_set_adc_scan_mode(struct bq2588x *bq, bool oneshot)
{
	int ret;
	u8 reg_val;

	if (oneshot == false)
		reg_val = BQ2588X_ADC_SCAN_CONTINUOUS;
	else
		reg_val = BQ2588X_ADC_SCAN_ONESHOT;

	reg_val <<= BQ2588X_ADC_SCAN_RATE_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_ADC_CTRL,
				BQ2588X_ADC_SCAN_RATE_MASK, reg_val);

	return ret;
}

static int bq2588x_set_adc_scan_bits(struct bq2588x *bq, int bits)
{
	int ret;
	u8 reg_val;

	if (bits == 15)
		reg_val = BQ2588X_ADC_SCAN_15BITS;
	else if (bits == 14)
		reg_val = BQ2588X_ADC_SCAN_14BITS;
	else if (bits == 13)
		reg_val = BQ2588X_ADC_SCAN_13BITS;
	else
		reg_val = BQ2588X_ADC_SCAN_12BITS;

	reg_val <<= BQ2588X_ADC_SCAN_BITS_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_ADC_CTRL,
				BQ2588X_ADC_SCAN_BITS_MASK, reg_val);

	return ret;
}


static int bq2588x_check_adc_scan_done(struct bq2588x *bq, bool *done)
{
	int ret;
	u8 reg_val;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHG_STATUS1, &reg_val);
	if (!ret) 
		*done = !!(reg_val & BQ2588X_ADC_DONE_STAT_MASK);

	return ret;
}

static int bq2588x_wait_adc_scan_done(struct bq2588x *bq)
{
	int ret;
	bool done = false;
	int retry = 0;

	while(retry++ < 20)
		ret = bq2588x_check_adc_scan_done(bq, &done);
		if (!ret && done)
			return 1;
		msleep(100);
	}

	return 0;
}

#define ADC_RES_REG_BASE	0x17

static int bq2588x_read_adc_data(struct bq2588x *bq, u8 channel, int *val)
{
	int ret;
	u16 res;
	u8 reg;

	if (channel >= ADC_MAX_NUM)
		return -EINVAL;
	reg = ADC_RES_REG_BASE + (channel << 1);

	ret = bq2588x_read_word(bq, reg, &res);
	if (ret >= 0)
		*val = (int)res;

	return ret;
}

static int bq2588x_read_bus_volt(struct bq2588x *bq, int *volt)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_VBUS, &reg_val);

	if (ret >= 0) {
		*volt = reg_val * BQ2588X_VBUS_ADC_LB_LSB;
		*volt += BQ2588X_VBUS_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_bus_curr(struct bq2588x *bq, int *curr)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_IBUS, &reg_val);

	if (ret >= 0) {
		*curr = reg_val * BQ2588X_IBUS_ADC_LB_LSB;
		*curr += BQ2588X_IBUS_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_bat_volt(struct bq2588x *bq, int *volt)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_VBAT, &reg_val);

	if (ret >= 0) {
		*volt = reg_val * BQ2588X_VBAT_ADC_LB_LSB;
		*volt += BQ2588X_VBAT_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_bat_curr(struct bq2588x *bq, int *curr)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_ICHG, &reg_val);

	if (ret >= 0) {
		*curr = reg_val * BQ2588X_ICHG_ADC_LB_LSB;
		*curr += BQ2588X_ICHG_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_sys_volt(struct bq2588x *bq, int *volt)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_VSYS, &reg_val);

	if (ret >= 0) {
		*volt = reg_val * BQ2588X_VSYS_ADC_LB_LSB;
		*volt += BQ2588X_VSYS_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_ts_temp(struct bq2588x *bq, int *temp)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_TS, &reg_val);

	if (ret >= 0) {
		*temp = reg_val * BQ2588X_TS_ADC_LB_LSB;
		*temp += BQ2588X_TS_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_die_temp(struct bq2588x *bq, int *temp)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_TDIE, &reg_val);

	if (ret >= 0) {
		*temp = reg_val * BQ2588X_TDIE_ADC_LB_LSB;
		*temp += BQ2588X_TDIE_ADC_LB_BASE;
	}

	return ret;
}


static int bq2588x_charging_disable(struct bq2588x *bq, int reason,
					int disable)
{

	int ret = 0;
	int disabled;

	mutex_lock(&bq->charging_disable_lock);

	disabled = bq->charging_disabled_status;

	bq_log("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled && bq->charge_enabled)
		ret = bq2588x_charge_enable(bq, false);
	else if (!disabled && !bq->charge_enabled)
		ret = bq2588x_charge_enable(bq, true);

	if (ret) {
		bq_err("Couldn't disable/enable charging for reason=%d ret=%d\n",
							ret, reason);
	} else {
		bq->charging_disabled_status = disabled;
		mutex_lock(&bq->data_lock);
		bq->charge_enabled = !disabled;
		mutex_unlock(&bq->data_lock);
	}
	mutex_unlock(&bq->charging_disable_lock);

	return ret;
}


static struct power_supply *get_bms_psy(struct bq2588x *bq)
{
	if (bq->bms_psy)
		return bq->bms_psy;
	bq->bms_psy = power_supply_get_by_name("bms");
	if (!bq->bms_psy)
		pr_debug("bms power supply not found\n");

	return bq->bms_psy;
}

static int bq2588x_get_batt_property(struct bq2588x *bq,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct power_supply *bms_psy = get_bms_psy(bq);

	int ret;

	if (!bms_psy)
		return -EINVAL;

	ret = power_supply_get_property(bms_psy, psp, val);

	return ret;
}

static inline bool is_device_suspended(struct bq2588x *bq);
static int bq2588x_get_prop_charge_type(struct bq2588x *bq)
{
	u8 val = 0;

	if (is_device_suspended(bq))
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	bq2588x_read_reg(bq, BQ2588X_REG_CHG_STATUS1, &val);
	val &= BQ2588X_CHRG_STAT_MASK;
	val >>= BQ2588X_CHRG_STAT_SHIFT;

	switch (val) {
	case BQ2588X_CHRG_STAT_FAST:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2588X_CHRG_STAT_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case BQ2588X_CHRG_STAT_DONE:
	case BQ2588X_CHRG_STAT_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static int bq2588x_get_prop_batt_present(struct bq2588x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq2588x_get_batt_property(bq,
			POWER_SUPPLY_PROP_PRESENT, &batt_prop);
	if (!ret)
		bq->batt_present = batt_prop.intval;

	return ret;

}

static int bq2588x_get_prop_batt_full(struct bq2588x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq2588x_get_batt_property(bq,
			POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret)
		bq->batt_full = (batt_prop.intval == POWER_SUPPLY_STATUS_FULL);

	return ret;
}

static int bq2588x_get_prop_charge_status(struct bq2588x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;
	u8 status;

	ret = bq2588x_get_batt_property(bq,
			POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret && batt_prop.intval == POWER_SUPPLY_STATUS_FULL)
		return POWER_SUPPLY_STATUS_FULL;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHG_STATUS1, &status);
	if (ret)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	mutex_lock(&bq->data_lock);
	bq->charge_state = (status & BQ2588X_CHRG_STAT_MASK) >> BQ2588X_CHRG_STAT_SHIFT;
	mutex_unlock(&bq->data_lock);

	switch (bq->charge_state) {
	case BQ2588X_CHRG_STAT_TRICKLE:
	case BQ2588X_CHRG_STAT_PRECHG:
	case BQ2588X_CHRG_STAT_FAST:
		return POWER_SUPPLY_STATUS_CHARGING;
	case BQ2588X_CHRG_STAT_DONE:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case BQ2588X_CHRG_STAT_IDLE:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

}

static int bq2588x_get_prop_health(struct bq2588x *bq)
{
	int ret;
	union power_supply_propval batt_prop = {0,};

	if (bq->software_jeita_supported) {
		if (bq->jeita_active) {
			if (bq->batt_hot)
				ret = POWER_SUPPLY_HEALTH_OVERHEAT;
			else if (bq->batt_warm)
				ret = POWER_SUPPLY_HEALTH_WARM;
			else if (bq->batt_cool)
				ret = POWER_SUPPLY_HEALTH_COOL;
			else if (bq->batt_cold)
				ret = POWER_SUPPLY_HEALTH_COLD;
		} else {
			ret = POWER_SUPPLY_HEALTH_GOOD;
		}
	} else {/* get health status from gauge */
		ret = bq2588x_get_batt_property(bq,
					POWER_SUPPLY_PROP_HEALTH, &batt_prop);
		if (!ret)
			ret = batt_prop.intval;
		else
			ret = POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	return ret;
}


static enum power_supply_property bq2588x_charger_props[] = {

	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,

	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	/*POWER_SUPPLY_PROP_TIME_TO_EMPTY,*/
	POWER_SUPPLY_PROP_CHARGE_FULL,

	/*POWER_SUPPLY_PROP_CYCLE_COUNT,*/

	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
};

static int bq2588x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{

	struct bq2588x *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2588x_get_prop_charge_type(bq);
		pr_debug("POWER_SUPPLY_PROP_CHARGE_TYPE:%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 3080;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2588x_get_prop_charge_status(bq);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq2588x_get_prop_health(bq);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		bq2588x_get_batt_property(bq, psp, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_TECHNOLOGY:
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		return bq2588x_get_batt_property(bq, psp, val);
	default:
		return -EINVAL;

	}

	return 0;
}

static int bq2588x_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct bq2588x *bq = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq2588x_charging_disable(bq, USER, !val->intval);

		power_supply_changed(bq->batt_psy);
		power_supply_changed(bq->usb_psy);
		bq_log("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n",
					val->intval ? "enable" : "disable");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2588x_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int bq2588x_update_charging_profile(struct bq2588x *bq)
{
	int ret;
	int chg_ma;
	int chg_mv;
	int icl;
	union power_supply_propval prop = {0,};


	if (!bq->usb_present)
		return 0;

	ret = power_supply_get_property(bq->usb_psy,
				POWER_SUPPLY_PROP_TYPE, &prop);

	if (ret < 0) {
		bq_err("couldn't read USB TYPE property, ret=%d\n", ret);
		return ret;
	}

	mutex_lock(&bq->profile_change_lock);
	if (bq->jeita_active) {
		chg_ma = bq->jeita_ma;
		chg_mv = bq->jeita_mv;
	} else {
		if (prop.intval == POWER_SUPPLY_TYPE_USB_DCP || prop.intval == POWER_SUPPLY_TYPE_USB_CDP) {
			chg_ma = bq->platform_data->ta.ichg;
			chg_mv = bq->platform_data->ta.vreg;
		} else {
			chg_ma = bq->platform_data->usb.ichg;
			chg_mv = bq->platform_data->usb.vreg;
		}
	}

	icl = bq->usb_psy_ma;

	bq_log("charge volt = %d, charge curr = %d, input curr limit = %d\n",
				chg_mv, chg_ma, icl);

	ret = bq2588x_set_input_current_limit(bq, icl);
	if (ret < 0)
		bq_err("couldn't set input current limit, ret=%d\n", ret);

	ret = bq2588x_set_input_volt_limit(bq, bq->platform_data->ta.vlim);
	if (ret < 0)
		bq_err("couldn't set input voltage limit, ret=%d\n", ret);

	ret = bq2588x_set_charge_voltage(bq, chg_mv);
	if (ret < 0)
		bq_err("couldn't set charge voltage ret=%d\n", ret);

	ret = bq2588x_set_charge_current(bq, chg_ma);
	if (ret < 0)
		bq_err("couldn't set charge current, ret=%d\n", ret);

	if (bq->jeita_active && (bq->batt_hot || bq->batt_cold))
		bq2588x_charging_disable(bq, JEITA, true);
	else
		bq2588x_charging_disable(bq, JEITA, false);

	mutex_unlock(&bq->profile_change_lock);

	return 0;
}


static void bq2588x_external_power_changed(struct power_supply *psy)
{
	struct bq2588x *bq = power_supply_get_drvdata(psy);

	union power_supply_propval prop = {0,};
	int ret, current_limit = 0;


	ret = power_supply_get_property(bq->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (ret < 0)
		bq_err("could not read USB current_max property, ret=%d\n", ret);
	else
		current_limit = prop.intval / 1000;

	bq_log("current_limit = %d\n", current_limit);

	if (bq->usb_psy_ma != current_limit) {
		bq->usb_psy_ma = current_limit;
		bq2588x_update_charging_profile(bq);
	}

	ret = power_supply_get_property(bq->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
		bq_err("could not read USB ONLINE property, ret=%d\n", ret);
	else
		bq_log("usb online status =%d\n", prop.intval);

	ret = 0;
	bq2588x_get_prop_charge_status(bq);
	if (bq->usb_present && bq->charge_state != BQ2588X_CHRG_STAT_IDLE) {
		if (prop.intval == 0) {
			bq_log("set usb online\n");
			ret = power_supply_set_online(bq->usb_psy, true);
		}
	} else {
		if (prop.intval == 1) {
			bq_log("set usb offline\n");
			ret = power_supply_set_online(bq->usb_psy, false);
		}
	}

	if (ret < 0)
		bq_err("could not set usb online state, ret=%d\n", ret);

}


static int bq2588x_psy_register(struct bq2588x *bq)
{
	int ret;
	struct power_supply_config batt_psy_cfg = {};

	bq->batt_psy_d.name = "battery";
	bq->batt_psy_d.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->batt_psy_d.properties = bq2588x_charger_props;
	bq->batt_psy_d.num_properties = ARRAY_SIZE(bq2588x_charger_props);
	bq->batt_psy_d.get_property = bq2588x_charger_get_property;
	bq->batt_psy_d.set_property = bq2588x_charger_set_property;
	bq->batt_psy_d.external_power_changed = bq2588x_external_power_changed;
	bq->batt_psy_d.property_is_writeable = bq2588x_charger_is_writeable;

	batt_psy_cfg.drv_data = bq;
	batt_psy_cfg.num_supplicants = 0;

	bq->batt_psy = power_supply_register(bq->dev,
				&bq->batt_psy_d,
				&batt_psy_cfg);
	if (IS_ERR(bq->batt_psy)) {
		bq_err("couldn't register battery psy, ret = %ld\n",
				PTR_ERR(bq->batt_psy));
		return ret;
	}

	return 0;
}

static void bq2588x_psy_unregister(struct bq2588x *bq)
{
	power_supply_unregister(bq->batt_psy);
}


static int bq2588x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2588x *bq = rdev_get_drvdata(rdev);

	ret = bq2588x_otg_enable(bq, true);
	if (ret) {
		bq_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		bq->otg_enabled = true;
		bq_log("bq2588x OTG mode Enabled!\n");
	}

	return ret;
}


static int bq2588x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2588x *bq = rdev_get_drvdata(rdev);

	ret = bq2588x_otg_enable(bq, false);
	if (ret) {
		bq_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		bq->otg_enabled = false;
		bq_log("bq2588x OTG mode Disabled\n");
	}

	return ret;
}


static int bq2588x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int ret;
	u8 status;
	struct bq2588x *bq = rdev_get_drvdata(rdev);

	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHG_STATUS2, &status);
	if (ret)
		return ret;
	status &= BQ2588X_VBUS_STAT_MASK;
	status >>= BQ2588X_VBUS_STAT_SHIFT;

	return (status == BQ2588X_VBUS_OTG_MODE) ? 1 : 0;

}


struct regulator_ops bq2588x_otg_reg_ops = {
	.enable		= bq2588x_otg_regulator_enable,
	.disable	= bq2588x_otg_regulator_disable,
	.is_enabled	= bq2588x_otg_regulator_is_enable,
};

static int bq2588x_regulator_init(struct bq2588x *bq)
{
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(bq->dev, bq->dev->of_node);
	if (!init_data) {
		dev_err(bq->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		bq->otg_vreg.rdesc.owner = THIS_MODULE;
		bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		bq->otg_vreg.rdesc.ops = &bq2588x_otg_reg_ops;
		bq->otg_vreg.rdesc.name = init_data->constraints.name;
		bq_log("regualtor name = %s\n", bq->otg_vreg.rdesc.name);

		cfg.dev = bq->dev;
		cfg.init_data = init_data;
		cfg.driver_data = bq;
		cfg.of_node = bq->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		bq->otg_vreg.rdev = regulator_register(
					&bq->otg_vreg.rdesc, &cfg);
		if (IS_ERR(bq->otg_vreg.rdev)) {
			ret = PTR_ERR(bq->otg_vreg.rdev);
			bq->otg_vreg.rdev = NULL;
			if (ret != -EPROBE_DEFER)
				dev_err(bq->dev,
					"OTG reg failed, rc=%d\n", ret);
		}
	}

	return ret;
}


static int bq2588x_parse_jeita_dt(struct device *dev, struct bq2588x *bq)
{
	struct device_node *np = dev->of_node;
	int ret;

	ret = of_property_read_u32(np, "ti,bq2588x,jeita-hot-degc",
						&bq->batt_hot_degc);
	if (ret) {
		bq_err("Failed to read ti,bq2588x,jeita-hot-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2588x,jeita-warm-degc",
						&bq->batt_warm_degc);
	if (ret) {
		bq_err("Failed to read ti,bq2588x,jeita-warm-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2588x,jeita-cool-degc",
						&bq->batt_cool_degc);
	if (ret) {
		bq_err("Failed to read ti,bq2588x,jeita-cool-degc\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2588x,jeita-cold-degc",
						&bq->batt_cold_degc);
	if (ret) {
		bq_err("Failed to read ti,bq2588x,jeita-cold-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2588x,jeita-hot-hysteresis",
						&bq->hot_temp_hysteresis);
	if (ret) {
		bq_err("Failed to read ti,bq2588x,jeita-hot-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2588x,jeita-cold-hysteresis",
						&bq->cold_temp_hysteresis);
	if (ret) {
		bq_err("Failed to read ti,bq2588x,jeita-cold-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2588x,jeita-cool-ma",
						&bq->batt_cool_ma);
	if (ret) {
		bq_err("Failed to read ti,bq2588x,jeita-cool-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2588x,jeita-cool-mv",
						&bq->batt_cool_mv);
	if (ret) {
		bq_err("Failed to read ti,bq2588x,jeita-cool-mv\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2588x,jeita-warm-ma",
						&bq->batt_warm_ma);
	if (ret) {
		bq_err("Failed to read ti,bq2588x,jeita-warm-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2588x,jeita-warm-mv",
						&bq->batt_warm_mv);
	if (ret) {
		bq_err("Failed to read ti,bq2588x,jeita-warm-mv\n");
		return ret;
	}

	bq->software_jeita_supported =
		of_property_read_bool(np, "ti,bq2588x,software-jeita-supported");

	return 0;
}


static struct bq2588x_platform_data *bq2588x_parse_dt(struct device *dev,
						struct bq2588x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;
	struct bq2588x_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(struct bq2588x_platform_data),
						GFP_KERNEL);
	if (!pdata)
		return NULL;

	ret = of_property_read_u32(np, "ti,bq2588x,usb-vlim", &pdata->usb.vlim);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,usb-vlim\n");

	ret = of_property_read_u32(np, "ti,bq2588x,usb-ilim", &pdata->usb.ilim);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,usb-ilim\n");

	ret = of_property_read_u32(np, "ti,bq2588x,usb-vreg", &pdata->usb.vreg);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,usb-vreg\n");

	ret = of_property_read_u32(np, "ti,bq2588x,usb-ichg", &pdata->usb.ichg);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,usb-ichg\n");

	ret = of_property_read_u32(np, "ti,bq2588x,ta-vlim", &pdata->ta.vlim);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,ta-vlim\n");

	ret = of_property_read_u32(np, "ti,bq2588x,ta-ilim", &pdata->ta.ilim);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,ta-ilim\n");

	ret = of_property_read_u32(np, "ti,bq2588x,ta-vreg", &pdata->ta.vreg);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,ta-vreg\n");

	ret = of_property_read_u32(np, "ti,bq2588x,ta-ichg", &pdata->ta.ichg);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,ta-ichg\n");

	ret = of_property_read_u32(np, "ti,bq2588x,precharge-current", &pdata->iprechg);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,precharge-current\n");

	ret = of_property_read_u32(np, "ti,bq2588x,termination-current", &pdata->iterm);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,termination-current\n");

	ret = of_property_read_u32(np, "ti,bq2588x,otg-voltage", &pdata->otg_volt);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,boost-voltage\n");

	ret = of_property_read_u32(np, "ti,bq2588x,otg-current", &pdata->otg_current);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,boost-current\n");

	return pdata;
}


static void bq2588x_init_jeita(struct bq2588x *bq)
{

	bq->batt_temp = -EINVAL;

	/* set default value in case of dts read fail */
	bq->batt_hot_degc = 600;
	bq->batt_warm_degc = 450;
	bq->batt_cool_degc = 100;
	bq->batt_cold_degc = 0;

	bq->hot_temp_hysteresis = 50;
	bq->cold_temp_hysteresis = 50;

	bq->batt_cool_ma = 400;
	bq->batt_cool_mv = 4100;
	bq->batt_warm_ma = 400;
	bq->batt_warm_mv = 4100;

	bq->software_jeita_supported = true;

	/* DTS setting will overwrite above default value */

	bq2588x_parse_jeita_dt(&bq->client->dev, bq);
}

static int bq2588x_init_device(struct bq2588x *bq)
{
	int ret;
	unsigned int mask;

	ret = bq2588x_set_prechg_current(bq, bq->platform_data->iprechg);
	if (ret)
		bq_err("Failed to set prechg current, ret = %d\n", ret);

	ret = bq2588x_set_term_current(bq, bq->platform_data->iterm);
	if (ret)
		bq_err("Failed to set termination current, ret = %d\n", ret);

	ret = bq2588x_set_otg_volt_limit(bq, bq->platform_data->otg_volt);
	if (ret)
		bq_err("Failed to set otg voltage, ret = %d\n", ret);

	ret = bq2588x_set_otg_current_limit(bq, bq->platform_data->otg_current);
	if (ret)
		bq_err("Failed to set otg current, ret = %d\n", ret);

	ret = bq2588x_charge_enable(bq, true);
	if (ret) {
		bq_err("Failed to enable charger, ret = %d\n", ret);
	} else {
		bq->charge_enabled = true;
		bq_log("Charger Enabled Successfully!\n");
	}

	bq2588x_set_ilim_pin(bq, true);

	mask = INT_MASK_ADC_DONE | INT_MASK_VSYS | INT_MASK_TREG |
		INT_MASK_TS | INT_MASK_TSHUT | INT_MASK_SYS_SHORT |
		INT_MASK_OTG | INT_MASK_IINDPM | INT_MASK_VINDPM;
	bq2588x_set_int_mask(bq, mask);

	bq2588x_set_adc_scan_bits(bq, 15);

	return 0;
}

static int bq2588x_detect_device(struct bq2588x *bq)
{
	int ret;
	u8 data;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_PART_NUM, &data);

	if (ret == 0) {
		bq->part_no = data & BQ2588X_PART_NO_MASK;
		bq->part_no >>= BQ2588X_PART_NO_SHIFT;
		bq->revision = data & BQ2588X_REVISION_MASK;
		bq->revision >>= BQ2588X_REVISION_SHIFT;
	}

	return ret;
}

static void bq2588x_update_status(struct bq2588x *bq)
{
	bq2588x_enable_adc_scan(bq, true);
	/* do one-shot scan */
	bq2588x_set_adc_scan_mode(bq, true);

	bq2588x_wait_adc_scan_done(bq);
	
	bq2588x_read_bus_volt(bq, &bq->vbus_volt);
	bq2588x_read_bat_volt(bq, &bq->vbat_volt);
	bq2588x_read_bus_curr(bq, &bq->ibus_curr);
	bq2588x_read_bat_curr(bq, &bq->ichg_curr);
	bq2588x_read_ts_temp(bq, &bq->ts_temp);
	bq2588x_read_die_temp(bq, &bq->die_temp);

	bq_log("vbus:%d, vbat:%d, ibus:%d, ichg:%d\n",
		bq->vbus_volt, bq->vbat_volt,
		bq->ibus_curr, bq->ichg_curr);

}

static void bq2588x_dump_status(struct bq2588x *bq)
{
	bq2588x_update_status(bq);
}



static void bq2588x_check_jeita(struct bq2588x *bq)
{

	int ret;
	bool last_hot, last_warm, last_cool, last_cold;
	bool chg_disabled_jeita, jeita_hot_cold;
	union power_supply_propval batt_prop = {0,};

	ret = bq2588x_get_batt_property(bq,
				POWER_SUPPLY_PROP_TEMP, &batt_prop);
	if (!ret)
		bq->batt_temp = batt_prop.intval;

	if (bq->batt_temp == -EINVAL)
		return;

	last_hot = bq->batt_hot;
	last_warm = bq->batt_warm;
	last_cool = bq->batt_cool;
	last_cold = bq->batt_cold;

	if (bq->batt_temp >= bq->batt_hot_degc) {/* HOT */
		if (!bq->batt_hot) {
			bq->batt_hot  = true;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_ma = 0;
			bq->jeita_mv = 0;
		}
	} else if (bq->batt_temp >= bq->batt_warm_degc) {/* WARM */
		if (!bq->batt_hot ||
			(bq->batt_temp < bq->batt_hot_degc - bq->hot_temp_hysteresis)) {
			bq->batt_hot  = false;
			bq->batt_warm = true;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_mv = bq->batt_warm_mv;
			bq->jeita_ma = bq->batt_warm_ma;
		}
	} else if (bq->batt_temp < bq->batt_cold_degc) {/* COLD */
		if (!bq->batt_cold) {
			bq->batt_hot  = false;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = true;
			bq->jeita_ma = 0;
			bq->jeita_mv = 0;
		}
	} else if (bq->batt_temp < bq->batt_cool_degc) {/* COOL */
		if (!bq->batt_cold ||
			(bq->batt_temp > bq->batt_cold_degc + bq->cold_temp_hysteresis)) {
			bq->batt_hot  = false;
			bq->batt_warm = false;
			bq->batt_cool = true;
			bq->batt_cold = false;
			bq->jeita_mv = bq->batt_cool_mv;
			bq->jeita_ma = bq->batt_cool_ma;
		}
	} else {/* NORMAL */
		bq->batt_hot  = false;
		bq->batt_warm = false;
		bq->batt_cool = false;
		bq->batt_cold = false;
	}

	bq->jeita_active = bq->batt_cool || bq->batt_hot ||
				   bq->batt_cold || bq->batt_warm;

	if ((last_cold != bq->batt_cold) || (last_warm != bq->batt_warm) ||
		(last_cool != bq->batt_cool) || (last_hot != bq->batt_hot)) {
		bq2588x_update_charging_profile(bq);
		power_supply_changed(bq->batt_psy);
		power_supply_changed(bq->usb_psy);
	} else if (bq->batt_hot || bq->batt_cold) { /*continuely update event*/
		power_supply_changed(bq->batt_psy);
		power_supply_changed(bq->usb_psy);
	}

	jeita_hot_cold = bq->jeita_active && (bq->batt_hot || bq->batt_cold);
	chg_disabled_jeita = !!(bq->charging_disabled_status & JEITA);
	if (jeita_hot_cold ^ chg_disabled_jeita)
		bq2588x_charging_disable(bq, JEITA, jeita_hot_cold);


}

static void bq2588x_check_batt_pres(struct bq2588x *bq)
{
	int ret = 0;
	bool chg_disabled_pres;

	ret = bq2588x_get_prop_batt_present(bq);
	if (!ret) {
		chg_disabled_pres = !!(bq->charging_disabled_status & BATT_PRES);
		if (chg_disabled_pres ^ !bq->batt_present) {
			ret = bq2588x_charging_disable(bq, BATT_PRES, !bq->batt_present);
			if (ret) {
				bq_err("failed to %s charging, ret = %d\n",
					bq->batt_present ? "disable" : "enable",
					ret);
			}
			bq_log("battery present:%d\n", bq->batt_present);
			power_supply_changed(bq->batt_psy);
			power_supply_changed(bq->usb_psy);
		}
	}


}

static void bq2588x_check_batt_full(struct bq2588x *bq)
{
	int ret = 0;
	bool chg_disabled_fc;

	ret = bq2588x_get_prop_batt_full(bq);
	if (!ret) {
		chg_disabled_fc = !!(bq->charging_disabled_status & BATT_FC);
		if (chg_disabled_fc ^ bq->batt_full) {
			ret = bq2588x_charging_disable(bq, BATT_FC, bq->batt_full);
			if (ret) {
				bq_err("failed to %s charging, ret = %d\n",
					bq->batt_full ? "disable" : "enable",
					ret);
			}
			bq_log("battery full:%d\n", bq->batt_present);
			power_supply_changed(bq->batt_psy);
			power_supply_changed(bq->usb_psy);
		}
	}
}


static int calculate_jeita_poll_interval(struct bq2588x *bq)
{
	int interval;

	if (bq->batt_hot || bq->batt_cold)
		interval = 5;
	else if (bq->batt_warm || bq->batt_cool)
		interval = 10;
	else
		interval = 15;
	return interval;
}

static enum alarmtimer_restart bq2588x_jeita_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	struct bq2588x *bq = container_of(alarm,
				struct bq2588x, jeita_alarm);
	unsigned long ns;

	bq2588x_stay_awake(&bq->bq2588x_ws, WAKEUP_SRC_JEITA);
	schedule_delayed_work(&bq->charge_jeita_work, HZ/2);

	ns = calculate_jeita_poll_interval(bq) * 1000000000LL;
	alarm_forward_now(alarm, ns_to_ktime(ns));
	return ALARMTIMER_RESTART;
}

static void bq2588x_charge_jeita_workfunc(struct work_struct *work)
{
	struct bq2588x *bq = container_of(work,
			struct bq2588x, charge_jeita_work.work);

	bq2588x_reset_wdt(bq);

	bq2588x_check_batt_pres(bq);
	bq2588x_check_batt_full(bq);

	bq2588x_check_jeita(bq);
	bq2588x_dump_status(bq);
	bq2588x_relax(&bq->bq2588x_ws, WAKEUP_SRC_JEITA);
}


static void bq2588x_discharge_jeita_workfunc(struct work_struct *work)
{
	struct bq2588x *bq = container_of(work,
			struct bq2588x, discharge_jeita_work.work);

	bq2588x_check_batt_pres(bq);
	bq2588x_check_batt_full(bq);

	bq2588x_check_jeita(bq);
	schedule_delayed_work(&bq->discharge_jeita_work,
			calculate_jeita_poll_interval(bq) * HZ);
}


static irqreturn_t bq2588x_charger_interrupt(int irq, void *dev_id)
{
	struct bq2588x *bq = dev_id;

	u8 status;
	int ret;

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		dev_dbg(bq->dev, "IRQ triggered before device-resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;
	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHG_STATUS2, &status);
	if (ret) {
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}

	mutex_lock(&bq->data_lock);
	bq->power_good = !!(status & BQ2588X_PG_STAT_MASK);
	mutex_unlock(&bq->data_lock);

	if (!bq->power_good) {
		if (bq->usb_present) {
			bq->usb_present = false;
			power_supply_set_present(bq->usb_psy, bq->usb_present);
		}

		if (bq->software_jeita_supported)
			alarm_try_to_cancel(&bq->jeita_alarm);

		bq2588x_set_wdt_timer(bq, 0);

		schedule_delayed_work(&bq->discharge_jeita_work,
					calculate_jeita_poll_interval(bq) * HZ);

		bq_err("usb removed, set usb present = %d\n", bq->usb_present);
	} else if (bq->power_good && !bq->usb_present) {
		bq->usb_present = true;
		msleep(10);/*for cdp detect*/
		power_supply_set_present(bq->usb_psy, bq->usb_present);

		cancel_delayed_work(&bq->discharge_jeita_work);

		if (bq->software_jeita_supported)
			alarm_start_relative(&bq->jeita_alarm,
						ns_to_ktime(1 * 1000000000LL));

		bq2588x_set_wdt_timer(bq, 80);

		bq_err("usb plugged in, set usb present = %d\n", bq->usb_present);
	}

	bq2588x_update_status(bq);

	mutex_unlock(&bq->irq_complete);

	power_supply_changed(bq->batt_psy);

	return IRQ_HANDLED;
}


static void determine_initial_status(struct bq2588x *bq)
{
	bq2588x_charger_interrupt(bq->client->irq, bq);
}


static ssize_t bq2588x_show_registers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq2588x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq2588x Reg");
	for (addr = 0x0; addr <= 0x25; addr++) {
		ret = bq2588x_read_reg(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				"Reg[0x%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2588x_store_registers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2588x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x16)
		bq2588x_write_reg(bq, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR,
			bq2588x_show_registers,
			bq2588x_store_registers);

static struct attribute *bq2588x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2588x_attr_group = {
	.attrs = bq2588x_attributes,
};

static int bq2588x_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq2588x *bq;
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;

	int ret;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		dev_dbg(&client->dev, "bms supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2588x), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->usb_psy = usb_psy;
	bq->bms_psy = bms_psy;

	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	ret = bq2588x_detect_device(bq);
	if (ret) {
		bq_err("No bq2588x device found!\n");
		return -ENODEV;
	}

	bq2588x_init_jeita(bq);

	if (client->dev.of_node)
		bq->platform_data = bq2588x_parse_dt(&client->dev, bq);
	else
		bq->platform_data = client->dev.platform_data;

	if (!bq->platform_data) {
		bq_err("No platform data provided.\n");
		return -EINVAL;
	}

	ret = bq2588x_init_device(bq);
	if (ret) {
		bq_err("Failed to init device\n");
		return ret;
	}


	ret = bq2588x_psy_register(bq);
	if (ret)
		return ret;
	ret = bq2588x_regulator_init(bq);
	if (ret) {
		bq_err("Couldn't initialize bq2588x regulator ret=%d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&bq->charge_jeita_work,
				bq2588x_charge_jeita_workfunc);
	INIT_DELAYED_WORK(&bq->discharge_jeita_work,
				bq2588x_discharge_jeita_workfunc);

	alarm_init(&bq->jeita_alarm, ALARM_BOOTTIME, bq2588x_jeita_alarm_cb);

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL,
				bq2588x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2588x charger irq", bq);
		if (ret < 0) {
			bq_err("request irq for irq=%d failed, ret =%d\n",
				client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
	}

	bq2588x_wakeup_src_init(bq);

	device_init_wakeup(bq->dev, 1);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2588x_attr_group);
	if (ret)
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);

	determine_initial_status(bq);

	bq_err("bq2588x probe successfully, Part Num:%d, Revision:%d\n!",
				bq->part_no, bq->revision);

	return 0;
err_1:
	bq2588x_psy_unregister(bq);

	return ret;
}


static inline bool is_device_suspended(struct bq2588x *bq)
{
	return !bq->resume_completed;
}

static int bq2588x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2588x *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);

	return 0;
}

static int bq2588x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2588x *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int bq2588x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2588x *bq = i2c_get_clientdata(client);


	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		bq2588x_charger_interrupt(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(bq->batt_psy);

	return 0;
}
static int bq2588x_charger_remove(struct i2c_client *client)
{
	struct bq2588x *bq = i2c_get_clientdata(client);

	alarm_try_to_cancel(&bq->jeita_alarm);

	cancel_delayed_work_sync(&bq->charge_jeita_work);
	cancel_delayed_work_sync(&bq->discharge_jeita_work);

	regulator_unregister(bq->otg_vreg.rdev);

	bq2588x_psy_unregister(bq);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);

	sysfs_remove_group(&bq->dev->kobj, &bq2588x_attr_group);


	return 0;
}


static void bq2588x_charger_shutdown(struct i2c_client *client)
{
}

static const struct of_device_id bq2588x_charger_match_table[] = {
	{.compatible = "ti,bq25880-charger",},
	{.compatible = "ti,bq25882-charger",},
	{},
};
MODULE_DEVICE_TABLE(of, bq2588x_charger_match_table);

static const struct i2c_device_id bq2588x_charger_id[] = {
	{ "bq25880-charger", BQ25880 },
	{ "bq25882-charger", BQ25882 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2588x_charger_id);

static const struct dev_pm_ops bq2588x_pm_ops = {
	.resume		= bq2588x_resume,
	.suspend_noirq = bq2588x_suspend_noirq,
	.suspend	= bq2588x_suspend,
};
static struct i2c_driver bq2588x_charger_driver = {
	.driver	= {
		.name	= "bq2588x-charger",
		.owner	= THIS_MODULE,
		.of_match_table = bq2588x_charger_match_table,
		.pm		= &bq2588x_pm_ops,
	},
	.id_table	= bq2588x_charger_id,

	.probe		= bq2588x_charger_probe,
	.remove		= bq2588x_charger_remove,
	.shutdown	= bq2588x_charger_shutdown,

};

module_i2c_driver(bq2588x_charger_driver);

MODULE_DESCRIPTION("TI BQ2588x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
