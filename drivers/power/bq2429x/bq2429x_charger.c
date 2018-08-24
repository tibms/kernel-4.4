/*
 * BQ2560x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#define pr_fmt(fmt)	"[bq2429x]: %s: " fmt, __func__

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
/*#include "bq2589x_notifier.h"*/

#include "bq2429x_reg.h"
#include "bq2429x.h"

enum bq2429x_vbus_type {
	BQ2429X_VBUS_NONE = REG08_VBUS_TYPE_NONE,
	BQ2429X_VBUS_USB = REG08_VBUS_TYPE_USB,
	BQ2429X_VBUS_ADAPTER = REG08_VBUS_TYPE_ADAPTER,
	BQ2429X_VBUS_OTG = REG08_VBUS_TYPE_OTG,
};

enum bq2429x_part_no {
	BQ24295 = 0x06,
	BQ24296 = 0x01,
	BQ24297 = 0x03,
};

enum bq2429x_charge_state {
	CHARGE_STATE_IDLE = REG08_CHRG_STAT_IDLE,
	CHARGE_STATE_PRECHG = REG08_CHRG_STAT_PRECHG,
	CHARGE_STATE_FASTCHG = REG08_CHRG_STAT_FASTCHG,
	CHARGE_STATE_CHGDONE = REG08_CHRG_STAT_CHGDONE,
};

enum {
	USER		= BIT(0),
	JEITA		= BIT(1),
	BATT_FC		= BIT(2),
	BATT_PRES	= BIT(3),
	BATT_TUNE	= BIT(4),
};


struct bq2429x {
	struct device *dev;
	struct i2c_client *client;

	enum bq2429x_part_no part_no;
	int revision;

	int status;
	int fake_rsoc;

	struct mutex charging_disable_lock;
	struct mutex profile_change_lock;
	struct mutex data_lock;
	struct mutex i2c_rw_lock;

	int usb_type;
	int vbus_type;

	bool usb_present;
	bool power_good;

	bool charge_enabled;
	bool otg_enabled;
	bool batfet_enabled;

	bool dpm_triggered;

	bool in_therm_regulation;
	bool in_vsys_regulation;

	int charge_state;
	int charging_disabled_status;

	int fault_status;

	struct bq2429x_platform_data *platform_data;

	struct delayed_work monitor_work;

	struct dentry *debug_root;

	struct power_supply_desc usb_psy_desc;
	struct power_supply *usb_psy;

	struct power_supply_desc ac_psy_desc;
	struct power_supply *ac_psy;

	struct power_supply_desc otg_psy_desc;
	struct power_supply *otg_psy;

	struct power_supply *batt_psy;

};


static int bq2429x_read_byte(struct bq2429x *bq, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("failed to read 0x%.2x\n", reg);
		mutex_unlock(&bq->i2c_rw_lock);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&bq->i2c_rw_lock);

	return 0;
}

static int bq2429x_write_byte(struct bq2429x *bq, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int bq2429x_update_bits(struct bq2429x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = bq2429x_read_byte(bq, reg, &tmp);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2429x_write_byte(bq, reg, tmp);
}
static int bq2429x_enable_otg(struct bq2429x *bq)
{
	u8 val = REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_01,
			   REG01_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_enable_otg);

static int bq2429x_disable_otg(struct bq2429x *bq)
{
	u8 val = REG01_OTG_DISABLE << REG01_OTG_CONFIG_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_01,
			   REG01_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_disable_otg);


static int bq2429x_enable_charger(struct bq2429x *bq)
{
	int ret;
	u8 val = REG01_CHG_ENABLE << REG01_CHG_CONFIG_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_CHG_CONFIG_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_enable_charger);

static int bq2429x_disable_charger(struct bq2429x *bq)
{
	int ret;
	u8 val = REG01_CHG_DISABLE << REG01_CHG_CONFIG_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_CHG_CONFIG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_disable_charger);

int bq2429x_set_chargecurrent(struct bq2429x *bq, int curr)
{
	u8 ichg;

	if (curr < REG02_ICHG_BASE)
		curr = REG02_ICHG_BASE;

	ichg = (curr - REG02_ICHG_BASE)/REG02_ICHG_LSB;
	ichg <<= REG02_ICHG_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_02, REG02_ICHG_MASK, ichg);

}

int bq2429x_set_term_current(struct bq2429x *bq, int curr)
{
	u8 iterm;

	if (curr < REG03_ITERM_BASE)
		curr = REG03_ITERM_BASE;

	iterm = (curr - REG03_ITERM_BASE) / REG03_ITERM_LSB;
	iterm <<=  REG03_ITERM_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_03, REG03_ITERM_MASK, iterm);
}


int bq2429x_set_prechg_current(struct bq2429x *bq, int curr)
{
	u8 iprechg;

	if (curr < REG03_IPRECHG_BASE)
		curr = REG03_IPRECHG_BASE;

	iprechg = (curr - REG03_IPRECHG_BASE) / REG03_IPRECHG_LSB;
	iprechg <<= REG03_IPRECHG_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_03, REG03_IPRECHG_MASK, iprechg);
}

int bq2429x_set_chargevoltage(struct bq2429x *bq, int volt)
{
	u8 val;

	if (volt < REG04_VREG_BASE)
		volt = REG04_VREG_BASE;

	val = (volt - REG04_VREG_BASE)/REG04_VREG_LSB;

	val <<= REG04_VREG_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_04, REG04_VREG_MASK, val);
}


int bq2429x_set_input_volt_limit(struct bq2429x *bq, int volt)
{
	u8 val;

	if (volt < REG00_VINDPM_BASE)
		volt = REG00_VINDPM_BASE;

	val = (volt - REG00_VINDPM_BASE) / REG00_VINDPM_LSB;
	val <<=  REG00_VINDPM_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_VINDPM_MASK, val);
}

int bq2429x_set_input_current_limit(struct bq2429x *bq, int curr)
{
	u8 val;

	switch (curr) {
	case BQ2429X_ILIM_100mA:
		val = REG00_IINLIM_100MA;
		break;
	case BQ2429X_ILIM_150mA:
		val = REG00_IINLIM_150MA;
		break;
	case BQ2429X_ILIM_900mA:
		val = REG00_IINLIM_900MA;
		break;
	case BQ2429X_ILIM_1000mA:
		val = REG00_IINLIM_1000MA;
		break;
	case BQ2429X_ILIM_1500mA:
		val = REG00_IINLIM_1500MA;
		break;
	case BQ2429X_ILIM_2000mA:
		val = REG00_IINLIM_2000MA;
		break;
	case BQ2429X_ILIM_3000mA:
		val = REG00_IINLIM_3000MA;
		break;
	default:
		val = REG00_IINLIM_500MA;
	}

	val <<= REG00_IINLIM_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_IINLIM_MASK, val);
}


int bq2429x_set_watchdog_timer(struct bq2429x *bq, u8 timeout)
{
	u8 val;

	if (val < REG05_WDT_BASE)
		val = REG05_WDT_BASE;

	val = (timeout - REG05_WDT_BASE) / REG05_WDT_LSB;
	val <<= REG05_WDT_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2429x_set_watchdog_timer);

int bq2429x_disable_watchdog_timer(struct bq2429x *bq)
{
	u8 val = REG05_WDT_DISABLE << REG05_WDT_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2429x_disable_watchdog_timer);

int bq2429x_reset_watchdog_timer(struct bq2429x *bq)
{
	u8 val = REG01_WDT_RESET << REG01_WDT_RESET_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2429x_reset_watchdog_timer);

int bq2429x_reset_chip(struct bq2429x *bq)
{
	int ret;
	u8 val = REG01_REG_RESET << REG01_REG_RESET_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_REG_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_reset_chip);

int bq2429x_enter_hiz_mode(struct bq2429x *bq)
{
	u8 val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_enter_hiz_mode);

int bq2429x_exit_hiz_mode(struct bq2429x *bq)
{

	u8 val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_exit_hiz_mode);

int bq2429x_get_hiz_mode(struct bq2429x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2429x_read_byte(bq, BQ2429X_REG_00, &val);
	if (ret)
		return ret;
	*state = (val & REG00_ENHIZ_MASK) >> REG00_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2429x_get_hiz_mode);


static int bq2429x_enable_term(struct bq2429x *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = REG05_TERM_ENABLE << REG05_EN_TERM_SHIFT;
	else
		val = REG05_TERM_DISABLE << REG05_EN_TERM_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_enable_term);

int bq2429x_set_boost_volt(struct bq2429x *bq, int volt)
{
	u8 val;

	val = (volt - REG06_BOOSTV_BASE) / REG06_BOOSTV_LSB;

	val <<=  REG06_BOOSTV_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_06, REG06_BOOSTV_MASK, val);
}


void bq2429x_set_otg(struct bq2429x *bq, bool enable)
{
	int ret;

	if (enable)
		ret = bq2429x_enable_otg(bq);
	else
		ret = bq2429x_disable_otg(bq);

	if (!ret)
		bq->otg_enabled = enable;
	else
		dev_err(bq->dev, "%s:Failed to %s otg:%d\n", __func__,
				enable ? "enable" : "disable", ret);
}

static int bq2429x_charging_disable(struct bq2429x *bq, int reason,
						int disable)
{
	int ret = 0;
	int disabled;

	mutex_lock(&bq->charging_disable_lock);

	disabled = bq->charging_disabled_status;

	pr_info("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled && bq->charge_enabled)
		ret = bq2429x_disable_charger(bq);
	else if (!disabled && !bq->charge_enabled)
		ret = bq2429x_enable_charger(bq);

	if (ret) {
		pr_err("Couldn't disable/enable charging for reason=%d ret=%d\n",
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

static ssize_t bq2429x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2429x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq2429x");
	for (addr = 0x0; addr <= 0x0A; addr++) {
		ret = bq2429x_read_byte(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				"Reg[%02X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2429x_store_register(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2429x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < 0x08)
		bq2429x_write_byte(bq, (u8)reg, (u8)val);

	return count;
}


static DEVICE_ATTR(registers, 0660, bq2429x_show_registers, bq2429x_store_register);

static struct attribute *bq2429x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2429x_attr_group = {
	.attrs = bq2429x_attributes,
};

#if 0
static int bq2429x_get_prop_charge_status(struct bq2429x *bq)
{
	u8 val = 0;

	bq2429x_read_byte(bq, BQ2429X_REG_08, &val);
	val &= REG08_CHRG_STAT_MASK;
	val >>= REG08_CHRG_STAT_SHIFT;
	switch (val) {
	case CHARGE_STATE_FASTCHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case CHARGE_STATE_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHARGE_STATE_CHGDONE:
	case CHARGE_STATE_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}


static int bq2429x_get_prop_charger_health(struct bq2429x *bq)
{
	u8 ntc_fault;
	int ret;

	ntc_fault = bq->fault_status & REG09_FAULT_NTC_MASK;
	ntc_fault >>= REG09_FAULT_NTC_SHIFT;

	if (ntc_fault == REG09_FAULT_NTC_HOT)
		ret = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (ntc_fault == REG09_FAULT_NTC_COLD)
		ret = POWER_SUPPLY_HEALTH_COLD;
	else
		ret = POWER_SUPPLY_HEALTH_GOOD;
	return ret;
}
#endif

static enum power_supply_property bq2429x_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,	
	POWER_SUPPLY_PROP_TYPE,
};


static int bq2429x_usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{

	struct bq2429x *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bq->usb_present && bq->charge_enabled &&
				(bq->usb_type == POWER_SUPPLY_TYPE_USB))
			val->intval = 1;
		else
			val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq->usb_present;
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->charge_enabled;
		break;

	case POWER_SUPPLY_PROP_TYPE:
		val->intval = bq->usb_type;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2429x_usb_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct bq2429x *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq2429x_charging_disable(bq, USER, !val->intval);
		power_supply_changed(bq->usb_psy);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int bq2429x_usb_is_writeable(struct power_supply *psy,
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
static int bq2429x_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{

	struct bq2429x *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bq->usb_present && bq->charge_enabled &&
			(bq->usb_type == POWER_SUPPLY_TYPE_USB_DCP))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq->usb_present;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = bq->usb_type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property bq2429x_otg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int bq2429x_otg_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq2429x *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq->otg_enabled;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq2429x_otg_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct bq2429x *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		bq2429x_set_otg(bq, !!val->intval);
		power_supply_changed(bq->otg_psy);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq2429x_otg_is_writeable(struct power_supply *psy,
					enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}


static char *bq2429x_supplied_to[] = {
	"battery",
};

static int bq2429x_psy_register(struct bq2429x *bq)
{
	struct power_supply_config psy_cfg = {};

	psy_cfg.drv_data = bq;
	psy_cfg.supplied_to = bq2429x_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(bq2429x_supplied_to);

	bq->usb_psy_desc.name = "usb";
	bq->usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
	bq->usb_psy_desc.properties = bq2429x_charger_props;
	bq->usb_psy_desc.num_properties = ARRAY_SIZE(bq2429x_charger_props);
	bq->usb_psy_desc.get_property = bq2429x_usb_get_property;
	bq->usb_psy_desc.set_property = bq2429x_usb_set_property;
	bq->usb_psy_desc.property_is_writeable = bq2429x_usb_is_writeable;

	bq->usb_psy = devm_power_supply_register(bq->dev, &bq->usb_psy_desc, &psy_cfg);
	if (IS_ERR(bq->usb_psy)) {
		pr_err("failed to register usb psy\n");
		return PTR_ERR(bq->usb_psy);
	}
	bq->ac_psy_desc.name = "ac";
	bq->ac_psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
	bq->ac_psy_desc.properties = bq2429x_charger_props;
	bq->ac_psy_desc.num_properties = ARRAY_SIZE(bq2429x_charger_props);
	bq->ac_psy_desc.get_property = bq2429x_ac_get_property;

	bq->ac_psy = power_supply_register(bq->dev, &bq->ac_psy_desc, &psy_cfg);
	if (IS_ERR(bq->ac_psy)) {
		pr_err("failed to register ac psy\n");
		return PTR_ERR(bq->ac_psy);
	}

	bq->otg_psy_desc.name = "otg";
	bq->otg_psy_desc.type = POWER_SUPPLY_TYPE_OTG;
	bq->otg_psy_desc.properties = bq2429x_otg_props;
	bq->otg_psy_desc.num_properties = ARRAY_SIZE(bq2429x_otg_props);
	bq->otg_psy_desc.get_property = bq2429x_otg_get_property;
	bq->otg_psy_desc.set_property = bq2429x_otg_set_property;
	bq->otg_psy_desc.property_is_writeable = bq2429x_otg_is_writeable;

	bq->otg_psy = devm_power_supply_register(bq->dev, &bq->otg_psy_desc, &psy_cfg);
	if (IS_ERR(bq->otg_psy)) {
		pr_err("failed to register otg psy\n");
		return PTR_ERR(bq->otg_psy);
	}

	return 0;
}


static struct bq2429x_platform_data *bq2429x_parse_dt(struct device *dev, struct bq2429x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	struct bq2429x_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(struct bq2429x_platform_data), GFP_KERNEL);
	if (!pdata)
		return NULL;

	ret = of_property_read_u32(np, "ti,bq2429x,usb-vlim", &pdata->usb.vlim);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,usb-vlim\n");

	ret = of_property_read_u32(np, "ti,bq2429x,usb-ilim", &pdata->usb.ilim);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,usb-ilim\n");

	ret = of_property_read_u32(np, "ti,bq2429x,usb-vreg", &pdata->usb.vreg);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,usb-vreg\n");

	ret = of_property_read_u32(np, "ti,bq2429x,usb-ichg", &pdata->usb.ichg);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,usb-ichg\n");

	ret = of_property_read_u32(np, "ti,bq2429x,ta-vlim", &pdata->ta.vlim);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,ta-vlim\n");

	ret = of_property_read_u32(np, "ti,bq2429x,ta-ilim", &pdata->ta.ilim);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,ta-ilim\n");

	ret = of_property_read_u32(np, "ti,bq2429x,ta-vreg", &pdata->ta.vreg);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,ta-vreg\n");

	ret = of_property_read_u32(np, "ti,bq2429x,ta-ichg", &pdata->ta.ichg);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,ta-ichg\n");

	ret = of_property_read_u32(np, "ti,bq2429x,precharge-current", &pdata->iprechg);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,precharge-current\n");

	ret = of_property_read_u32(np, "ti,bq2429x,termination-current", &pdata->iterm);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,termination-current\n");

	ret = of_property_read_u32(np, "ti,bq2429x,boost-volt", &pdata->boostv);
	if (ret)
		pr_err("Failed to read node of ti,bq2429x,boost-volt\n");

	pdata->enable_term = of_property_read_bool(np, "ti,bq2429x,enable_term");

	return pdata;
}

static int bq2429x_init_device(struct bq2429x *bq)
{
	int ret;

	bq2429x_disable_watchdog_timer(bq);

	ret = bq2429x_set_prechg_current(bq, bq->platform_data->iprechg);
	if (ret)
		pr_err("Failed to set prechg current, ret = %d\n", ret);

	ret = bq2429x_set_term_current(bq, bq->platform_data->iterm);
	if (ret)
		pr_err("Failed to set termination current, ret = %d\n", ret);

	ret = bq2429x_set_boost_volt(bq, bq->platform_data->boostv);
	if (ret)
		pr_err("Failed to set boost voltage, ret = %d\n", ret);

	bq2429x_enable_term(bq, bq->platform_data->enable_term);

	ret = bq2429x_enable_charger(bq);
	if (ret) {
		pr_err("Failed to enable charger, ret = %d\n", ret);
	} else {
		bq->charge_enabled = true;
		pr_info("Charger Enabled Successfully!");
	}

	return 0;
}


static int bq2429x_detect_device(struct bq2429x *bq)
{
	int ret;
	u8 data;

	ret = bq2429x_read_byte(bq, BQ2429X_REG_0A, &data);
	if (ret == 0) {
		bq->part_no = (data & REG0A_PN_MASK) >> REG0A_PN_SHIFT;
		bq->revision = (data & REG0A_DEV_REV_MASK) >> REG0A_DEV_REV_SHIFT;
	}

	return ret;
}

static const unsigned char *charge_stat_str[] = {
	"Not Charging",
	"Precharging",
	"Fast Charging",
	"Charge Done",
};

static void bq2429x_update_status(struct bq2429x *bq)
{
	u8 status;
	int ret;

	ret = bq2429x_read_byte(bq, BQ2429X_REG_01, &status);
	if (ret)
		return;
	mutex_lock(&bq->data_lock);
	bq->charge_enabled = !!(status & REG01_CHG_CONFIG_MASK);
	bq->otg_enabled = !!(status & REG01_OTG_CONFIG_MASK);
	mutex_unlock(&bq->data_lock);

	ret = bq2429x_read_byte(bq, BQ2429X_REG_08, &status);
	if (ret)
		return;

	mutex_lock(&bq->data_lock);
	bq->charge_state = (status & REG08_CHRG_STAT_MASK) >> REG08_CHRG_STAT_SHIFT;
	bq->in_therm_regulation = !!(status & REG08_THERM_STAT_MASK);
	bq->in_vsys_regulation = !!(status & REG08_VSYS_STAT_MASK);
	bq->dpm_triggered = !!(status & REG08_INDPM_STAT_MASK);
	mutex_unlock(&bq->data_lock);

	ret = bq2429x_read_byte(bq, BQ2429X_REG_09, &status);
	if (ret)
		return;

	mutex_lock(&bq->data_lock);
	bq->fault_status = status;
	mutex_unlock(&bq->data_lock);


	if (!bq->power_good)
		pr_info("Power Poor\n");
	if (bq->in_therm_regulation)
		pr_info("In thermal regulation!\n");
	if (bq->in_vsys_regulation)
		pr_info("In VSYS regulation!\n");
	if (bq->dpm_triggered)
		pr_info("VINDPM or IINDPM triggered\n");

	if (bq->fault_status & REG09_FAULT_WDT_MASK)
		pr_info("Watchdog timer expired!\n");
	if (bq->fault_status & REG09_FAULT_BOOST_MASK)
		pr_info("Boost fault occurred!\n");

	status = bq->fault_status & REG09_FAULT_CHRG_MASK;
	status >>= REG09_FAULT_CHRG_SHIFT;
	if (status == REG09_FAULT_CHRG_INPUT)
		pr_info("input fault!\n");
	else if (status == REG09_FAULT_CHRG_THERMAL)
		pr_info("charge thermal shutdown fault!\n");
	else if (status == REG09_FAULT_CHRG_TIMER)
		pr_info("charge timer expired fault!\n");

	if (bq->fault_status & REG09_FAULT_BAT_MASK)
		pr_info("battery ovp fault!\n");

	status = bq->fault_status & REG09_FAULT_NTC_MASK;
	status >>= REG09_FAULT_NTC_SHIFT;

	if (status == REG09_FAULT_NTC_HOT)
		pr_info("NTC HOT\n");
	else if (status == REG09_FAULT_NTC_COLD)
		pr_info("NTC COLD\n");
	else
		pr_info("NTC Normal\n");

	pr_info("%s\n", charge_stat_str[bq->charge_state]);
}


static void bq2429x_dump_registers(struct bq2429x *bq)
{
	int ret;
	int i;
	u8 reg_val;

	for (i = 0; i <= 0x0A; i++) {
		ret = bq2429x_read_byte(bq, i, &reg_val);
		if (!ret)
			pr_err("Reg[%02X] = 0x%02X\n", i, reg_val);
	}
}

static void bq2429x_monitor_workfunc(struct work_struct *work)
{
	struct bq2429x *bq = container_of(work, struct bq2429x, monitor_work.work);

	/*TODO: do some routine work here*/
	bq2429x_reset_watchdog_timer(bq);

	bq2429x_update_status(bq);

	bq2429x_dump_registers(bq);

	schedule_delayed_work(&bq->monitor_work, 10 * HZ);
}

static void bq2429x_adapter_in_handler(struct bq2429x *bq)
{
	int ret;
	int vlim, ilim, ichg, vreg;

	if (bq->vbus_type == BQ2429X_VBUS_USB) {
		bq->usb_type = POWER_SUPPLY_TYPE_USB;
		vlim = bq->platform_data->usb.vlim;
		ilim = bq->platform_data->usb.ilim;
		ichg = bq->platform_data->usb.ichg;
		vreg = bq->platform_data->usb.vreg;
	} else if (bq->vbus_type == BQ2429X_VBUS_ADAPTER) {
		bq->usb_type = POWER_SUPPLY_TYPE_USB_DCP;
		vlim = bq->platform_data->ta.vlim;
		ilim = bq->platform_data->ta.ilim;
		ichg = bq->platform_data->ta.ichg;
		vreg = bq->platform_data->ta.vreg;
	} else {
		bq->usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
		return;
	}

	ret = bq2429x_set_input_volt_limit(bq, vlim);
	if (ret)
		pr_err("Failed to set input volt limit with ret=%d\n", ret);

	ret = bq2429x_set_input_current_limit(bq, ilim);
	if (ret)
		pr_err("Failed to set input current limit with ret=%d\n", ret);

	ret = bq2429x_set_chargecurrent(bq, ichg);
	if (ret)
		pr_err("Failed to set charge current with ret=%d\n", ret);

	ret = bq2429x_set_chargevoltage(bq, vreg);
	if (ret)
		pr_err("Failed to set charge voltage with ret=%d\n", ret);

/*	bq2429x_notifier_attach_attached_dev(bq->usb_type);*/

	pr_err("adapter plugin, usb type:%d\n", bq->usb_type);

	schedule_delayed_work(&bq->monitor_work, HZ);
}


static void bq2429x_adapter_out_handler(struct bq2429x *bq)
{

/*	bq2429x_notifier_detach_attached_dev(bq->usb_type);*/

	pr_err("adapter removed\n");
	cancel_delayed_work_sync(&bq->monitor_work);
}

static irqreturn_t bq2429x_charger_interrupt(int irq, void *dev_id)
{
	struct bq2429x *bq = dev_id;

	u8 status;
	int ret;

	ret = bq2429x_read_byte(bq, BQ2429X_REG_08, &status);
	if (ret)
		return ret;

	mutex_lock(&bq->data_lock);
	bq->vbus_type = (status & REG08_VBUS_STAT_MASK) >> REG08_VBUS_STAT_SHIFT;
	bq->power_good = !!(status & REG08_PG_STAT_MASK);
	mutex_unlock(&bq->data_lock);


	if (!bq->power_good) {
		if (bq->usb_present)
			bq->usb_present = false;

		bq2429x_disable_watchdog_timer(bq);

		bq2429x_adapter_out_handler(bq);

	} else if (bq->power_good && !bq->usb_present) {
		bq->usb_present = true;

		bq2429x_set_watchdog_timer(bq, 80);

		bq2429x_adapter_in_handler(bq);
	}

	power_supply_changed(bq->usb_psy);

	bq2429x_update_status(bq);

	return IRQ_HANDLED;
}

static int bq2429x_charger_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct bq2429x *bq;

	int ret;

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2429x), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->data_lock);

	ret = bq2429x_detect_device(bq);
	if (ret) {
		pr_err("No bq2429x device found!");
		return -ENODEV;
	}

	if (client->dev.of_node)
		bq->platform_data = bq2429x_parse_dt(&client->dev, bq);
	else
		bq->platform_data = client->dev.platform_data;

	if (!bq->platform_data) {
		pr_err("No platform data provided.\n");
		return -EINVAL;
	}

	ret = bq2429x_init_device(bq);
	if (ret) {
		pr_err("Failed to init device");
		return ret;
	}

	ret = bq2429x_psy_register(bq);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&bq->monitor_work, bq2429x_monitor_workfunc);

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				bq2429x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2429x charger irq", bq);
		if (ret < 0) {
			pr_err("request irq for irq=%d failed, ret =%d\n", client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
	}

	pr_info("bq2429x probe successfully, Part Num:%d, Revision:%d\n!",
				bq->part_no, bq->revision);

	/*call explicitly to check adapter state, */
	bq2429x_charger_interrupt(client->irq, bq);

	return 0;

err_1:

	return ret;
}


static void bq2429x_charger_shutdown(struct i2c_client *client)
{
	struct bq2429x *bq = i2c_get_clientdata(client);

	pr_info("shutdown\n");

	cancel_delayed_work_sync(&bq->monitor_work);

}
static struct of_device_id bq2429x_charger_match_table[] = {
	{.compatible = "ti,charger,bq24295",},
	{.compatible = "ti,charger,bq24296",},
	{.compatible = "ti,charger,bq24297",},
	{},
};
MODULE_DEVICE_TABLE(of, bq2429x_charger_match_table);

static const struct i2c_device_id bq2429x_charger_id[] = {
	{ "bq24295-charger", BQ24295 },
	{ "bq24296-charger", BQ24296 },
	{ "bq24297-charger", BQ24297 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2429x_charger_id);

static struct i2c_driver bq2429x_charger_driver = {
	.driver		= {
	.name		= "bq2429x-charger",
		.owner	= THIS_MODULE,
		.of_match_table = bq2429x_charger_match_table,
	},
	.id_table	= bq2429x_charger_id,

	.probe		= bq2429x_charger_probe,
	.shutdown	= bq2429x_charger_shutdown,

};

module_i2c_driver(bq2429x_charger_driver);

MODULE_DESCRIPTION("TI BQ2429x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
