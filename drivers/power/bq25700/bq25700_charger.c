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


#define pr_fmt(fmt)	"[bq25700] %s: " fmt, __func__

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

#include "bq25700_reg.h"
#include "bq25700.h"

#define	bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_err
#define bq_log	pr_err

enum {
	USER = BIT(0),
	BATT = BIT(1),		/* Batt FULL */
};

enum {
	BQ25700 = 0x00,
};

enum {
	CHARGE_STATE_NOT_CHARGING,
	CHARGE_STATE_PRECHARGE,
	CHARGE_STATE_FASTCHARGE,
	CHARGE_STATE_OTG,
};


struct bq25700_otg_regulator {
	struct regulator_desc rdesc;
	struct regulator_dev *rdev;
};


struct bq25700 {
	struct device *dev;
	struct i2c_client *client;

	int part_no;
	int revision;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex profile_change_lock;
	struct mutex charging_disable_lock;
	struct mutex usb_online_lock;
	struct mutex adc_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool usb_present;
	bool usb_online;

	bool vbus_present;

	bool batt_full;

	int vbus_volt;
	int vbat_volt;
	int vsys_volt;
	int psys_volt;
	int cmpin_volt;
	int ibus_curr;
	int chg_curr;
	int dsg_curr;

	int batt_temp;

	bool charge_enabled;	/* Register bit status */
	bool otg_enabled;
	bool vindpm_triggered;
	bool iindpm_triggered;

	int usb_psy_ma;

	int icl_ma;
	int ivl_mv;

	int chg_ma;
	int chg_mv;

	int charge_state;
	int charging_disabled_status;
	int fault_status;

	int dev_id;

	enum power_supply_type supply_type;

	struct delayed_work monitor_work;

	struct bq25700_platform_data *platform_data;
	struct bq25700_otg_regulator otg_vreg;

	int skip_writes;
	int skip_reads;

	struct dentry *debug_root;

	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply *batt_psy;
	struct power_supply_desc batt_psy_d;
};


static int __bq25700_read_word(struct bq25700 *bq, u8 reg, u16 *data)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u16) ret;

	return 0;
}


static int __bq25700_write_word(struct bq25700 *bq, u8 reg, u16 data)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(bq->client, reg, data);
	if (ret < 0) {
		bq_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	return 0;
}


static int bq25700_read_reg(struct bq25700 *bq, u8 reg, u16 *data)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq25700_read_word(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq25700_write_reg(struct bq25700 *bq, u8 reg, u16 data)
{
	int ret;

	if (bq->skip_writes) {
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq25700_write_word(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq25700_update_bits(struct bq25700 *bq, u8 reg,
				    u16 mask, u16 data)
{
	int ret;
	u16 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq25700_read_word(bq, reg, &tmp);
	if (ret) {
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq25700_write_word(bq, reg, tmp);
	if (ret)
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}


static int bq25700_set_lwpwr_mode(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == true)
		reg_val = BQ25700_LWPWR_MODE_ENABLE;
	else 
		reg_val = BQ25700_LWPWR_MODE_DISABLE;

	reg_val <<= BQ25700_LWPWR_MODE_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_LWPWR_MODE_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_watchdog_timer(struct bq25700 *bq, int time)
{
	int ret;
	u16 reg_val;

	if (time == 0)
		reg_val = BQ25700_WDTMR_DISABLE;
	else if (time == 5)
		reg_val = BQ25700_WDTMR_5S;
	else if (time == 88)
		reg_val = BQ25700_WDTMR_88S;
	else
		reg_val = BQ25700_WDTMR_175S;

	reg_val <<= BQ25700_WDTMR_ADJ_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_WDTMR_ADJ_MASK, reg_val);

	return ret;
}

static int bq25700_set_idpm_auto_enable(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == true)
		reg_val = BQ25700_IDPM_AUTO_ENABLE;
	else 
		reg_val = BQ25700_IDPM_AUTO_DISABLE;

	reg_val <<= BQ25700_IDPM_AUTO_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_IDPM_AUTO_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_otg_on_chrgok(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_OTG_ON_CHRGOK_DISABLE;
	else 
		reg_val = BQ25700_OTG_ON_CHRGOK_ENABLE;

	reg_val <<= BQ25700_OTG_ON_CHRGOK_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_OTG_ON_CHRGOK_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_pwm_freq(struct bq25700 *bq, int freq)
{
	int ret;
	u16 reg_val;

	if (freq == 1200)
		reg_val = BQ25700_PWM_FREQ_1200KHZ;
	else
		reg_val = BQ25700_PWM_FREQ_800KHZ;

	reg_val <<= BQ25700_PWM_FREQ_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_PWM_FREQ_MASK, reg_val);

	return ret;
}

static int bq25700_set_learn_mode(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_LEARN_MODE_DISABLE;
	else 
		reg_val = BQ25700_LEARN_MODE_ENABLE;

	reg_val <<= BQ25700_LEARN_MODE_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_LEARN_MODE_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_iadpt_gain(struct bq25700 *bq, int gain)
{
	int ret;
	u16 reg_val;

	if (gain == 20)
		reg_val = BQ25700_IADPT_GAIN_20X;
	else
		reg_val = BQ25700_IADPT_GAIN_40X;

	reg_val <<= BQ25700_IADPT_GAIN_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_IADPT_GAIN_MASK, reg_val);

	return ret;
}

static int bq25700_set_ibat_gain(struct bq25700 *bq, int gain)
{
	int ret;
	u16 reg_val;

	if (gain == 8)
		reg_val = BQ25700_IBAT_GAIN_8X;
	else
		reg_val = BQ25700_IBAT_GAIN_16X;

	reg_val <<= BQ25700_IBAT_GAIN_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_IBAT_GAIN_MASK, reg_val);

	return ret;
}

static int bq25700_set_ldo_mode(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_LDO_MODE_DISABLE;
	else 
		reg_val = BQ25700_LDO_MODE_ENABLE;

	reg_val <<= BQ25700_LDO_MODE_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_LDO_MODE_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_idpm_enable(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_IDPM_DISABLE;
	else 
		reg_val = BQ25700_IDPM_ENABLE;

	reg_val <<= BQ25700_IDPM_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_IDPM_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_charge_enable(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == true)
		reg_val = BQ25700_CHRG_ENABLE;
	else 
		reg_val = BQ25700_CHRG_DISABLE;

	reg_val <<= BQ25700_CHRG_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION0,
					BQ25700_CHRG_CTRL_MASK, reg_val);

	return ret;
}


static int bq25700_set_rsns_rac(struct bq25700 *bq, int mohm)
{
	int ret;
	u16 reg_val;

	if (mohm == 10)
		reg_val = BQ25700_RSNS_RAC_10MOHM;
	else
		reg_val = BQ25700_RSNS_RAC_20MOHM;

	reg_val <<= BQ25700_RSNS_RAC_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION1,
				BQ25700_RSNS_RAC_MASK, reg_val);

	return ret;
}

static int bq25700_set_rsns_rsr(struct bq25700 *bq, int mohm)
{
	int ret;
	u16 reg_val;

	if (mohm == 10)
		reg_val = BQ25700_RSNS_RSR_10MOHM;
	else
		reg_val = BQ25700_RSNS_RSR_20MOHM;

	reg_val <<= BQ25700_RSNS_RSR_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION1,
				BQ25700_RSNS_RSR_MASK, reg_val);

	return ret;
}

static int bq25700_set_acoc_enable(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_ACOC_DISABLE;
	else
		reg_val = BQ25700_ACOC_ENABLE;

	reg_val <<= BQ25700_ACOC_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION2,
				BQ25700_ACOC_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_acoc_volt_pct(struct bq25700 *bq, int volt_pct)
{
	int ret;
	u16 reg_val;

	if (volt_pct == 125)
		reg_val = BQ25700_ACOC_VTH_125PCT;
	else
		reg_val = BQ25700_ACOC_VTH_200PCT;

	reg_val <<= BQ25700_ACOC_VTH_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION2,
				BQ25700_ACOC_VTH_MASK, reg_val);

	return ret;
}


static int bq25700_set_batoc_enable(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_BATOC_DISABLE;
	else
		reg_val = BQ25700_BATOC_ENABLE;

	reg_val <<= BQ25700_BATOC_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION2,
				BQ25700_BATOC_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_batoc_volt_pct(struct bq25700 *bq, int volt_pct)
{
	int ret;
	u16 reg_val;

	if (volt_pct == 125)
		reg_val = BQ25700_BATOC_VTH_125PCT;
	else
		reg_val = BQ25700_BATOC_VTH_200PCT;

	reg_val <<= BQ25700_BATOC_VTH_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION2,
				BQ25700_BATOC_VTH_MASK, reg_val);

	return ret;
}


static int bq25700_set_hiz_mode(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_HIZ_MODE_DISABLE;
	else 
		reg_val = BQ25700_HIZ_MODE_ENABLE;

	reg_val <<= BQ25700_HIZ_MODE_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION3,
					BQ25700_HIZ_MODE_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_reset_reg(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_REG_RESET_IDLE;
	else 
		reg_val = BQ25700_REG_RESET;

	reg_val <<= BQ25700_REG_RESET_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION3,
					BQ25700_REG_RESET_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_reset_vindpm(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_VINDPM_RESET_IDLE;
	else 
		reg_val = BQ25700_VINDPM_RESET;

	reg_val <<= BQ25700_VINDPM_RESET_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION3,
					BQ25700_VINDPM_RESET_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_otg_enable(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_OTG_DISABLE;
	else 
		reg_val = BQ25700_OTG_ENABLE;

	reg_val <<= BQ25700_OTG_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION3,
					BQ25700_OTG_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_ico_enable(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == false)
		reg_val = BQ25700_ICO_DISABLE;
	else 
		reg_val = BQ25700_ICO_ENABLE;

	reg_val <<= BQ25700_ICO_MODE_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION3,
					BQ25700_ICO_MODE_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_batfet_in_hiz(struct bq25700 *bq, bool enable)
{
	int ret;
	u16 reg_val;

	if (enable == true)
		reg_val = BQ25700_HIZ_BATFET_ON;
	else 
		reg_val = BQ25700_HIZ_BATFET_OFF;

	reg_val <<= BQ25700_HIZ_BATFET_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_OPTION3,
					BQ25700_HIZ_BATFET_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_get_ilim2_vth(struct bq25700 *bq, int *vth)
{
	int ret;
	u16 reg_val;

	ret = bq25700_read_reg(bq, BQ25700_REG_PROHOT_OPTION0, &reg_val);
	if (ret)
		return ret;

	*vth = reg_val & BQ25700_ILIM2_VTH_MASK;
	*vth >>= BQ25700_ILIM2_VTH_SHIFT;
	*vth *= BQ25700_ILIM2_VTH_LSB;
	*vth += BQ25700_ILIM2_VTH_BASE;

	return 0;
}

static int bq25700_set_icrit_deglitch(struct bq25700 *bq, int deg)
{
	int ret;
	u16 reg_val;

	if (deg == 15)
		reg_val = BQ25700_ICRIT_DEG_15US;
	else if (deg == 100)
		reg_val = BQ25700_ICRIT_DEG_100US;
	else if (deg == 400)
		reg_val = BQ25700_ICRIT_DEG_400US;
	else
		reg_val = BQ25700_ICRIT_DEG_800US;

	reg_val <<= BQ25700_ICRIT_DEG_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_PROHOT_OPTION0,
					BQ25700_ICRIT_DEG_MASK, reg_val);

	return ret;
}

static int bq25700_set_vsys_threshold(struct bq25700 *bq, int th)
{
	int ret;
	u16 reg_val;

	if (th == 5750)
		reg_val = BQ25700_VSYS_VTH_5P75V_2P85V;
	else if (th == 6000)
		reg_val = BQ25700_VSYS_VTH_6P0V_3P1V;
	else if (th == 6250)
		reg_val = BQ25700_VSYS_VTH_6P25V_3P35V;
	else
		reg_val = BQ25700_VSYS_VTH_6P5V_3P6V;

	reg_val <<= BQ25700_VSYS_VTH_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_PROHOT_OPTION0,
					BQ25700_VSYS_VTH_MASK, reg_val);

	return ret;
}

static int bq25700_set_dsg_th(struct bq25700 *bq, int vth)
{
	int ret;
	u16 reg_val;

	if (vth < BQ25700_DSG_VTH_BASE)
		vth = BQ25700_DSG_VTH_BASE;

	vth -= BQ25700_DSG_VTH_BASE;
	reg_val = vth / BQ25700_DSG_VTH_LSB;
	reg_val <<= BQ25700_DSG_VTH_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_PROHOT_OPTION1,
					BQ25700_DSG_VTH_MASK, reg_val);

	return ret;
}

static int bq25700_set_adc_scan_mode(struct bq25700 *bq, bool oneshot)
{
	int ret;
	u16 reg_val;

	if (oneshot == true)
		reg_val = BQ25700_ADC_CONV_ONESHOT;
	else 
		reg_val = BQ25700_ADC_CONV_CONT;

	reg_val <<= BQ25700_ADC_CONV_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_ADC_OPTION,
					BQ25700_ADC_CONV_MASK, reg_val);

	return ret;
}

static int bq25700_adc_start(struct bq25700 *bq, bool start)
{
	int ret;
	u16 reg_val;

	if (start == false)
		reg_val = BQ25700_ADC_START_IDLE;
	else 
		reg_val = BQ25700_ADC_START;

	reg_val <<= BQ25700_ADC_START_CTRL_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_ADC_OPTION,
					BQ25700_ADC_START_CTRL_MASK, reg_val);

	return ret;
}

static int bq25700_set_adc_full_scale(struct bq25700 *bq, int scale)
{
	int ret;
	u16 reg_val;

	if (scale == 2040)
		reg_val = BQ25700_ADC_FULLSCALE_2P04V;
	else
		reg_val = BQ25700_ADC_FULLSCALE_3P06V;

	reg_val <<= BQ25700_ADC_FULLSCALE_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_ADC_OPTION,
					BQ25700_ADC_FULLSCALE_MASK, reg_val);

	return ret;
}

static int bq25700_set_adc_scan_channel(struct bq25700 *bq, int mask)
{

	int ret;
	u16 reg_val;
	
	reg_val = mask & 0xFF;

	ret = bq25700_update_bits(bq, BQ25700_REG_ADC_OPTION,
					0xFF, reg_val);
	return ret;
}
static int bq25700_set_charge_current(struct bq25700 *bq, int curr)
{
	int ret;
	u16 reg_val;

	if (curr < BQ25700_CHARGE_CURRENT_BASE)
		curr = BQ25700_CHARGE_CURRENT_BASE;

	curr -= BQ25700_CHARGE_CURRENT_BASE;
	reg_val = curr / BQ25700_CHARGE_CURRENT_LSB;
	reg_val <<= BQ25700_CHARGE_CURRENT_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_CHARGE_CURRENT,
					BQ25700_CHARGE_CURRENT_MASK, reg_val);

	return ret;
}

static int bq25700_set_charge_volt(struct bq25700 *bq, int volt)
{
	int ret;
	u16 reg_val;

	if (volt < BQ25700_MAX_CHG_VOLT_BASE)
		volt = BQ25700_MAX_CHG_VOLT_BASE;

	volt -= BQ25700_MAX_CHG_VOLT_BASE;
	reg_val = volt / BQ25700_MAX_CHG_VOLT_LSB;
	reg_val <<= BQ25700_MAX_CHG_VOLT_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_MAX_CHG_VOLT,
					BQ25700_MAX_CHG_VOLT_MASK, reg_val);

	return ret;
}

static int bq25700_set_min_sys_volt(struct bq25700 *bq, int volt)
{
	int ret;
	u16 reg_val;

	if (volt < BQ25700_MIN_SYS_VOLT_BASE)
		volt = BQ25700_MIN_SYS_VOLT_BASE;

	volt -= BQ25700_MIN_SYS_VOLT_BASE;
	reg_val = volt / BQ25700_MIN_SYS_VOLT_LSB;
	reg_val <<= BQ25700_MIN_SYS_VOLT_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_MIN_SYS_VOLT,
					BQ25700_MIN_SYS_VOLT_MASK, reg_val);

	return ret;
}

static int bq25700_set_input_current_limit(struct bq25700 *bq, int curr)
{
	int ret;
	u16 reg_val;

	if (curr < BQ25700_INLIM_HOST_BASE)
		curr = BQ25700_INLIM_HOST_BASE;

	curr -= BQ25700_INLIM_HOST_BASE;
	reg_val = curr / BQ25700_INLIM_HOST_LSB;
	reg_val <<= BQ25700_INLIM_HOST_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_IIN_HOST,
					BQ25700_INLIM_HOST_MASK, reg_val);

	return ret;
}

static int bq25700_get_real_input_current_limit(struct bq25700 *bq, int curr)
{
	int ret;
	u16 reg_val;

	ret = bq25700_read_reg(bq, BQ25700_REG_IIN_DPM, &reg_val);
	if (ret)
		return ret;

	curr = reg_val & BQ25700_IIN_DPM_MASK;
	curr >>= BQ25700_IIN_DPM_SHIFT;
	curr *= BQ25700_IIN_DPM_LSB;
	curr += BQ25700_IIN_DPM_BASE;

	return 0;
}

static int bq25700_set_input_volt_limit(struct bq25700 *bq, int volt)
{
	int ret;
	u16 reg_val;

	if (volt < BQ25700_VIN_DPM_BASE)
		volt = BQ25700_VIN_DPM_BASE;

	volt -= BQ25700_VIN_DPM_BASE;
	reg_val = volt / BQ25700_VIN_DPM_LSB;
	reg_val <<= BQ25700_VIN_DPM_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_VIN_DPM,
					BQ25700_VIN_DPM_MASK, reg_val);

	return ret;
}

static int bq25700_set_otg_voltage(struct bq25700 *bq, int volt)
{
	int ret;
	u16 reg_val;

	if (volt < BQ25700_OTG_VOLT_BASE)
		volt = BQ25700_OTG_VOLT_BASE;

	volt -= BQ25700_OTG_VOLT_BASE;
	reg_val = volt / BQ25700_OTG_VOLT_LSB;
	reg_val <<= BQ25700_OTG_VOLT_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_OTG_VOLT,
					BQ25700_OTG_VOLT_MASK, reg_val);

	return ret;
}

static int bq25700_set_otg_current(struct bq25700 *bq, int curr)
{
	int ret;
	u16 reg_val;

	if (curr < BQ25700_OTG_CURRENT_BASE)
		curr = BQ25700_OTG_CURRENT_BASE;

	curr -= BQ25700_OTG_CURRENT_BASE;
	reg_val = curr / BQ25700_OTG_CURRENT_LSB;
	reg_val <<= BQ25700_OTG_CURRENT_SHIFT;

	ret = bq25700_update_bits(bq, BQ25700_REG_OTG_CURRENT,
					BQ25700_OTG_CURRENT_MASK, reg_val);

	return ret;
}

static int bq25700_read_vbus_volt(struct bq25700 *bq, int *volt)
{
	int ret;
	u16 reg_val;

	ret = bq25700_read_reg(bq, BQ25700_REG_ADC_VBUS_PSYS, &reg_val);
	if (ret)
		return ret;

	*volt = reg_val & BQ25700_ADC_VBUS_VOLT_MASK;
	*volt >>= BQ25700_ADC_VBUS_VOLT_SHIFT;
	*volt *= BQ25700_ADC_VBUS_VOLT_LSB;
	*volt += BQ25700_ADC_VBUS_VOLT_BASE;

	return 0;
}

static int bq25700_read_psys_volt(struct bq25700 *bq, int *volt)
{
	int ret;
	u16 reg_val;

	ret = bq25700_read_reg(bq, BQ25700_REG_ADC_VBUS_PSYS, &reg_val);
	if (ret)
		return ret;

	*volt = reg_val & BQ25700_ADC_PSYS_VOLT_MASK;
	*volt >>= BQ25700_ADC_PSYS_VOLT_SHIFT;
	*volt *= BQ25700_ADC_PSYS_VOLT_LSB;
	*volt += BQ25700_ADC_PSYS_VOLT_BASE;

	return 0;
}

static int bq25700_read_ibat_chg_current(struct bq25700 *bq, int *curr)
{
	int ret;
	u16 reg_val;

	ret = bq25700_read_reg(bq, BQ25700_REG_ADC_IBAT, &reg_val);
	if (ret)
		return ret;

	*curr = reg_val & BQ25700_ADC_CHG_CURRENT_MASK;
	*curr >>= BQ25700_ADC_CHG_CURRENT_SHIFT;
	*curr *= BQ25700_ADC_CHG_CURRENT_LSB;
	*curr += BQ25700_ADC_CHG_CURRENT_BASE;

	return 0;
}

static int bq25700_read_ibat_dsg_current(struct bq25700 *bq, int *curr)
{
	int ret;
	u16 reg_val;

	ret = bq25700_read_reg(bq, BQ25700_REG_ADC_IBAT, &reg_val);
	if (ret)
		return ret;

	*curr = reg_val & BQ25700_ADC_DSG_CURRENT_MASK;
	*curr >>= BQ25700_ADC_DSG_CURRENT_SHIFT;
	*curr *= BQ25700_ADC_DSG_CURRENT_LSB;
	*curr += BQ25700_ADC_DSG_CURRENT_BASE;

	return 0;
}

static int bq25700_read_ibus_current(struct bq25700 *bq, int *curr)
{
	int ret;
	u16 reg_val;

	ret = bq25700_read_reg(bq, BQ25700_REG_ADC_IIN_CMPIN, &reg_val);
	if (ret)
		return ret;

	*curr = reg_val & BQ25700_ADC_INPUT_CURRENT_MASK;
	*curr >>= BQ25700_ADC_INPUT_CURRENT_SHIFT;
	*curr *= BQ25700_ADC_INPUT_CURRENT_LSB;
	*curr += BQ25700_ADC_INPUT_CURRENT_BASE;

	return 0;
}

static int bq25700_read_cmpin_volt(struct bq25700 *bq, int *volt)
{
	int ret;
	u16 reg_val;

	ret = bq25700_read_reg(bq, BQ25700_REG_ADC_IIN_CMPIN, &reg_val);
	if (ret)
		return ret;

	*volt = reg_val & BQ25700_ADC_CMPIN_VOLT_MASK;
	*volt >>= BQ25700_ADC_CMPIN_VOLT_SHIFT;
	*volt *= BQ25700_ADC_CMPIN_VOLT_LSB;
	*volt += BQ25700_ADC_CMPIN_VOLT_BASE;

	return 0;
}

static int bq25700_read_vsys_volt(struct bq25700 *bq, int *volt)
{
	int ret;
	u16 reg_val;

	ret = bq25700_read_reg(bq, BQ25700_REG_ADC_VSYS_VBAT, &reg_val);
	if (ret)
		return ret;

	*volt = reg_val & BQ25700_ADC_SYS_VOLT_MASK;
	*volt >>= BQ25700_ADC_SYS_VOLT_SHIFT;
	*volt *= BQ25700_ADC_SYS_VOLT_LSB;
	*volt += BQ25700_ADC_SYS_VOLT_BASE;

	return 0;
}

static int bq25700_read_vbat_volt(struct bq25700 *bq, int *volt)
{
	int ret;
	u16 reg_val;

	ret = bq25700_read_reg(bq, BQ25700_REG_ADC_VSYS_VBAT, &reg_val);
	if (ret)
		return ret;

	*volt = reg_val & BQ25700_ADC_BAT_VOLT_MASK;
	*volt >>= BQ25700_ADC_BAT_VOLT_SHIFT;
	*volt *= BQ25700_ADC_BAT_VOLT_LSB;
	*volt += BQ25700_ADC_BAT_VOLT_BASE;

	return 0;
}

static int bq25700_get_vbus_present(struct bq25700 *bq)
{
	int ret;
	u16 status;

	ret = bq25700_read_reg(bq, BQ25700_REG_CHARGER_STATUS, &status);
	if (!ret)
		bq->vbus_present = status & BQ25700_AC_STAT_MASK;
	
	return ret;
}

static int bq25700_get_charge_state(struct bq25700 *bq)
{
	int ret;
	u16 mode;

	ret = bq25700_read_reg(bq, BQ25700_REG_CHARGER_STATUS, &mode);
	if (!ret) {
		if (mode & BQ25700_FAST_CHG_STAT_MASK)
			bq->charge_state = CHARGE_STATE_FASTCHARGE;
		else if (mode & BQ25700_PRE_CHG_STAT_MASK)
			bq->charge_state = CHARGE_STATE_PRECHARGE;
		else if (mode & BQ25700_OTG_STAT_MASK)
			bq->charge_state = CHARGE_STATE_OTG;
		else
			bq->charge_state = CHARGE_STATE_NOT_CHARGING;
	}
	return ret;
}


static int bq25700_get_batt_property(struct bq25700 *bq,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	int ret;

	if (bq->bms_psy == NULL) {
		pr_err("%s: bms power supply is NULL\n", __func__);
		return -ENODEV;
	}

	ret = power_supply_get_property(bq->bms_psy, psp, val);

	return ret;
}


static void set_usb_present(struct bq25700 *bq)
{

	bq25700_get_vbus_present(bq);

	if (!bq->vbus_present && bq->usb_present) {
		bq->usb_present = false;
		power_supply_set_present(bq->usb_psy, bq->usb_present);

		pr_info("usb removed, set usb present = %d\n", bq->usb_present);
	} else if (bq->vbus_present && !bq->usb_present) {
		bq->usb_present = true;
		power_supply_set_present(bq->usb_psy, bq->usb_present);

		pr_info("usb plugged in, set usb present = %d\n",
			bq->usb_present);
	}
}

static void set_usb_online(struct bq25700 *bq)
{
	int ret;

	mutex_lock(&bq->usb_online_lock);

	if (!bq->usb_online && bq->usb_present
	    && bq->charge_enabled && bq->chg_ma) {
		ret = power_supply_set_online(bq->usb_psy, true);
		if (!ret) {
			bq->usb_online = true;
			pr_info("set usb online %d successfully\n",
				bq->usb_online);
		} else {
			pr_err("set usb online 1 failed\n");
		}
	} else if (bq->usb_online && (!bq->usb_present || !bq->charge_enabled
				      || !bq->chg_ma)) {
		ret = power_supply_set_online(bq->usb_psy, false);
		if (!ret) {
			bq->usb_online = false;
			pr_info("set usb online %d successfully\n",
				bq->usb_online);
		} else {
			pr_err("set usb online 0 failed\n");
		}
	}

	mutex_unlock(&bq->usb_online_lock);
}


static int bq25700_charging_disable(struct bq25700 *bq, int reason, int disable)
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
		ret = bq25700_charge_enable(bq, false);
	else if (!disabled && !bq->charge_enabled)
		ret = bq25700_charge_enable(bq, true);

	if (ret) {
		pr_err
		    ("Couldn't disable/enable charging for reason=%d ret=%d\n",
		     ret, reason);
	} else {
		bq->charging_disabled_status = disabled;
		mutex_lock(&bq->data_lock);
		bq->charge_enabled = !disabled;
		mutex_unlock(&bq->data_lock);
	}
	mutex_unlock(&bq->charging_disable_lock);

	set_usb_online(bq);

	return ret;
}

static int bq25700_update_charging_profile(struct bq25700 *bq)
{
	int ret;

	mutex_lock(&bq->profile_change_lock);

	ret = bq25700_set_charge_volt(bq, bq->chg_mv);
	if (ret)
		pr_err("Failed to set chargevolt, ret=%d\n", ret);

	ret = bq25700_set_charge_current(bq, bq->chg_ma);
	if (ret)
		pr_err("Failed to set chargecurrent, ret=%d\n", ret);

	ret = bq25700_set_input_current_limit(bq, bq->icl_ma);
	if (ret < 0)
		pr_err("couldn't set input current limit, ret=%d\n", ret);

	ret = bq25700_set_input_volt_limit(bq, bq->ivl_mv);
	if (ret < 0)
		pr_err("couldn't set input voltage limit, ret=%d\n", ret);

	mutex_unlock(&bq->profile_change_lock);

	return 0;
}


static int bq25700_get_prop_charge_type(struct bq25700 *bq)
{

	switch (bq->charge_state) {
	case CHARGE_STATE_FASTCHARGE:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case CHARGE_STATE_PRECHARGE:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHARGE_STATE_NOT_CHARGING:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

#if 0
static int bq25700_get_prop_batt_present(struct bq25700 *bq)
{
	union power_supply_propval batt_prop = { 0, };
	int ret;

	ret = bq25700_get_batt_property(bq,
					POWER_SUPPLY_PROP_PRESENT, &batt_prop);
	if (!ret)
		bq->batt_present = batt_prop.intval;

	return ret;

}

static int bq25700_get_prop_batt_full(struct bq25700 *bq)
{
	union power_supply_propval batt_prop = { 0, };
	int ret;

	ret = bq25700_get_batt_property(bq,
					POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret)
		bq->batt_full = (batt_prop.intval == POWER_SUPPLY_STATUS_FULL);

	return ret;
}
#endif

static int bq25700_get_prop_charge_status(struct bq25700 *bq)
{
	union power_supply_propval batt_prop = { 0, };
	int ret;

	ret = bq25700_get_batt_property(bq,
				POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret && batt_prop.intval == POWER_SUPPLY_STATUS_FULL)
		return POWER_SUPPLY_STATUS_FULL;

	if (bq->charge_state == CHARGE_STATE_FASTCHARGE ||
	    bq->charge_state == CHARGE_STATE_PRECHARGE)
		return POWER_SUPPLY_STATUS_CHARGING;
	else if (bq->charge_state == CHARGE_STATE_NOT_CHARGING)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if (!bq->vbus_present)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else
		return POWER_SUPPLY_STATUS_UNKNOWN;

}

static enum power_supply_property bq25700_charger_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
/*	POWER_SUPPLY_PROP_TIME_TO_EMPTY,*/
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CURRENT_CAPABILITY,
	POWER_SUPPLY_PROP_TYPE,

};

static int bq25700_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{

	struct bq25700 *bq = power_supply_get_drvdata(psy); 

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq25700_get_prop_charge_type(bq);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 4800000; /* 4800 mAh */
		break;

	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = !bq->charging_disabled_status;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq25700_get_prop_charge_status(bq);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		return bq25700_get_batt_property(bq, psp, val);
	default:
		return -EINVAL;

	}

	return 0;
}

static int bq25700_charger_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	struct bq25700 *bq = power_supply_get_drvdata(psy); 

	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq25700_charging_disable(bq, USER, !val->intval);

		power_supply_changed(bq->batt_psy);
		power_supply_changed(bq->usb_psy);
		pr_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n",
			val->intval ? "enable" : "disable");
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		bq25700_charging_disable(bq, BATT, !val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		bq->chg_mv = val->intval / 1000;
		ret = bq25700_set_charge_volt(bq, bq->chg_mv);
		if (ret)
			pr_err("Failed to set chargevolt, ret=%d\n", ret);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		bq->chg_ma = val->intval / 1000;
		ret = bq25700_set_charge_current(bq, bq->chg_ma);
		if (ret)
			pr_err("Failed to set chargecurrent, ret=%d\n", ret);

		set_usb_online(bq);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		bq->supply_type = val->intval;
		/* notify USB of supply type */
		ret =
		    power_supply_set_supply_type(bq->usb_psy, bq->supply_type);
		pr_err("set power supply type :%d %s\n", bq->supply_type,
		       !ret ? "successfully" : "failed");
		set_usb_present(bq);
		set_usb_online(bq);
		break;

	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
		bq->icl_ma = val->intval / 1000; /* uA to mA */
		bq25700_update_charging_profile(bq);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		bq->ivl_mv = val->intval / 1000;
		bq25700_update_charging_profile(bq);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25700_charger_is_writeable(struct power_supply *psy,
					enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_TYPE:
	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
	case POWER_SUPPLY_PROP_TYPEC_MODE:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static void bq25700_external_power_changed(struct power_supply *psy)
{
#if 0
	struct bq25700 *bq = container_of(psy, struct bq25700, batt_psy);

	union power_supply_propval prop = { 0, };
	int ret, current_limit = 0;

	ret = bq->usb_psy->get_property(bq->usb_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (ret < 0)
		pr_err("could not read USB current_max property, ret=%d\n",
		       ret);
	else
		current_limit = prop.intval / 1000;

	pr_info("current_limit = %d\n", current_limit);

	if (bq->usb_psy_ma != current_limit) {
		bq->usb_psy_ma = current_limit;
		bq25700_update_charging_profile(bq);
	}

	ret = bq->usb_psy->get_property(bq->usb_psy,
					POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
		pr_err("could not read USB ONLINE property, ret=%d\n", ret);
	else
		pr_info("usb online status =%d\n", prop.intval);

	ret = 0;
	if (bq->usb_present
	/* && bq->charge_enabled *//*!bq->charging_disabled_status */
	    /*&& bq->usb_psy_ma != 0 */) {
		if (prop.intval == 0) {
			pr_err("set usb online\n");
			ret = power_supply_set_online(bq->usb_psy, true);
		}
	} else {
		if (prop.intval == 1) {
			pr_err("set usb offline\n");
			ret = power_supply_set_online(bq->usb_psy, false);
		}
	}

	if (ret < 0)
		pr_info("could not set usb online state, ret=%d\n", ret);
#endif

}

static int bq25700_psy_register(struct bq25700 *bq)
{
	int ret;
	struct power_supply_config batt_psy_cfg = {};

	bq->batt_psy_d.name = "battery";
	bq->batt_psy_d.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->batt_psy_d.properties = bq25700_charger_props;
	bq->batt_psy_d.num_properties = ARRAY_SIZE(bq25700_charger_props);
	bq->batt_psy_d.get_property = bq25700_charger_get_property;
	bq->batt_psy_d.set_property = bq25700_charger_set_property;
	bq->batt_psy_d.external_power_changed = bq25700_external_power_changed;
	bq->batt_psy_d.property_is_writeable = bq25700_charger_is_writeable;

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

static void bq25700_psy_unregister(struct bq25700 *bq)
{
	power_supply_unregister(bq->batt_psy);
}

static int bq25700_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct bq25700 *bq = rdev_get_drvdata(rdev);

	ret = bq25700_set_otg_enable(bq, true);
	if (ret) {
		pr_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		bq->otg_enabled = true;
		pr_info("bq25700 OTG mode Enabled!\n");
	}

	return ret;
}

static int bq25700_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq25700 *bq = rdev_get_drvdata(rdev);

	ret = bq25700_set_otg_enable(bq, false);
	if (ret) {
		pr_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		bq->otg_enabled = false;
		pr_info("bq25700 OTG mode Disabled\n");
	}

	return ret;
}

static int bq25700_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct bq25700 *bq = rdev_get_drvdata(rdev);

	bq25700_get_charge_state(bq);

	return (bq->charge_state == CHARGE_STATE_OTG) ? 1 : 0;

}

struct regulator_ops bq25700_otg_reg_ops = {
	.enable = bq25700_otg_regulator_enable,
	.disable = bq25700_otg_regulator_disable,
	.is_enabled = bq25700_otg_regulator_is_enable,
};

static int bq25700_regulator_init(struct bq25700 *bq)
{
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = { };

	init_data = of_get_regulator_init_data(bq->dev, bq->dev->of_node);
	if (!init_data) {
		dev_err(bq->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		bq->otg_vreg.rdesc.owner = THIS_MODULE;
		bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		bq->otg_vreg.rdesc.ops = &bq25700_otg_reg_ops;
		bq->otg_vreg.rdesc.name = init_data->constraints.name;
		pr_info("regualtor name = %s\n", bq->otg_vreg.rdesc.name);

		cfg.dev = bq->dev;
		cfg.init_data = init_data;
		cfg.driver_data = bq;
		cfg.of_node = bq->dev->of_node;

		init_data->constraints.valid_ops_mask
		    |= REGULATOR_CHANGE_STATUS;

		bq->otg_vreg.rdev =
		    regulator_register(&bq->otg_vreg.rdesc, &cfg);
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

static struct bq25700_platform_data *bq25700_parse_dt(struct device *dev,
						      struct bq25700 *bq)
{
	int ret;
	struct device_node *np = dev->of_node;
	struct bq25700_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(struct bq25700_platform_data),
			     GFP_KERNEL);
	if (!pdata) {
		pr_err("Out of memory\n");
		return NULL;
	}

	ret =
	    of_property_read_u32(np, "ti,bq25700,input-volt-limit",
				 &pdata->ivl_mv);
	if (ret)
		pr_err("Failed to read node of ti,bq25700,input-volt-limit\n");

	ret =
	    of_property_read_u32(np, "ti,bq25700,input-curr-limit",
				 &pdata->icl_ma);
	if (ret)
		pr_err("Failed to read node of ti,bq25700,input-curr-limit\n");

	ret =
	    of_property_read_u32(np, "ti,bq25700,charge-volt", &pdata->chg_mv);
	if (ret)
		pr_err("Failed to read node of ti,bq25700,charge-volt\n");

	ret =
	    of_property_read_u32(np, "ti,bq25700,charge-current",
				 &pdata->chg_ma);
	if (ret)
		pr_err("Failed to read node of ti,bq25700,charge-current\n");

	ret =
	    of_property_read_u32(np, "ti,bq25700,boost-voltage",
				&pdata->boostv);
	if (ret)
		pr_err("Failed to read node of ti,bq25700,boost-voltage\n");

	ret =
	    of_property_read_u32(np, "ti,bq25700,boost-current",
				 &pdata->boosti);
	if (ret)
		pr_err("Failed to read node of ti,bq25700,boost-current\n");

	return pdata;
}

static int bq25700_init_device(struct bq25700 *bq)
{
	int ret;

	bq25700_set_watchdog_timer(bq, BQ25700_WDTMR_175S);

	bq->ivl_mv = bq->platform_data->ivl_mv;
	bq->icl_ma = bq->platform_data->icl_ma;
	bq->chg_mv = bq->platform_data->chg_mv;
	bq->chg_ma = bq->platform_data->chg_ma;

	/* set initial charging profile */
	bq25700_update_charging_profile(bq);

	bq25700_set_rsns_rac(bq, 10);
	bq25700_set_rsns_rsr(bq, 10);

	ret = bq25700_set_otg_voltage(bq, bq->platform_data->boostv);
	if (ret)
		pr_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = bq25700_set_otg_current(bq, bq->platform_data->boosti);
	if (ret)
		pr_err("Failed to set boost current, ret = %d\n", ret);

	bq25700_set_acoc_volt_pct(bq, 125);
	bq25700_set_acoc_enable(bq, true);
	bq25700_set_batoc_volt_pct(bq, 125);
	bq25700_set_batoc_enable(bq, true);

	ret = bq25700_charge_enable(bq, true);
	if (ret) {
		pr_err("Failed to enable charger, ret = %d\n", ret);
	} else {
		bq->charge_enabled = true;
		pr_info("Charger Enabled Successfully!\n");
	}

	bq25700_set_adc_scan_channel(bq, 0xFF);

	ret = bq25700_adc_start(bq, false);
	pr_info("ADC start %s\n", !ret ? "successfully" : "failed");

	return 0;
}

static int bq25700_detect_device(struct bq25700 *bq)
{
	int ret;
	u16 data;

	ret = bq25700_read_reg(bq, BQ25700_REG_DEVICE_ID, &data);
	if (ret == 0)
		bq->dev_id = data & BQ25700_DEVICE_ID_MASK;

	return ret;
}

static const unsigned char *charge_stat_str[] = {
	"Not Charging",
	"Precharging",
	"Fast Charging",
	"Charge Done",
};

static void bq25700_dump_status(struct bq25700 *bq)
{
	pr_info("battery %s, temperature: %d\n",
		bq->batt_present ? "present" : "not present", bq->batt_temp);

	if (bq->vindpm_triggered)
		pr_info("VINDPM triggered\n");
	if (bq->iindpm_triggered)
		pr_info("IINDPM triggered\n");

	pr_info("vbus:%d, ibus:%d, vbat:%d,ichg:%d, idsg:%d\n",
	     bq->vbus_volt, bq->ibus_curr, bq->vbat_volt, bq->chg_curr,
	     bq->dsg_curr);

	pr_info("%s\n", charge_stat_str[bq->charge_state]);
}

static void bq25700_update_status(struct bq25700 *bq)
{
	u16 status;
	int ret;
	int last_charge_state;
	int last_fault_status;

	last_charge_state = bq->charge_state;
	bq25700_get_charge_state(bq);

	ret = bq25700_read_reg(bq, BQ25700_REG_CHARGER_STATUS, &status);
	if (ret)
		return;

	mutex_lock(&bq->data_lock);
	bq->vindpm_triggered = !!(status & BQ25700_VINDPM_STAT_MASK);
	bq->iindpm_triggered = !!(status & BQ25700_IINDPM_STAT_MASK);
	mutex_unlock(&bq->data_lock);

	last_fault_status = bq->fault_status;
	bq->fault_status = status & 0xFF;
	if (bq->charge_state != last_charge_state ||
	    bq->fault_status != last_fault_status)
		power_supply_changed(bq->batt_psy);

	bq25700_read_vbus_volt(bq, &bq->vbus_volt);
	bq25700_read_ibat_chg_current(bq, &bq->chg_curr);
	bq25700_read_ibat_dsg_current(bq, &bq->dsg_curr);
	bq25700_read_vbat_volt(bq, &bq->vbat_volt);
	bq25700_read_vsys_volt(bq, &bq->vsys_volt);
	bq25700_read_ibus_current(bq, &bq->ibus_curr);
}

static void bq25700_monitor_workfunc(struct work_struct *work)
{
	struct bq25700 *bq =
	    container_of(work, struct bq25700, monitor_work.work);

	bq25700_update_status(bq);
	bq25700_dump_status(bq);

	schedule_delayed_work(&bq->monitor_work, 5 * HZ);
}

const u8 reg_list[] = {
	0x12, 0x14, 0x15, 0x30,
	0x31, 0x32, 0x33, 0x34,
	0x35, 0x20, 0x31, 0x22,
	0x23, 0x24, 0x25, 0x26,
	0x3B, 0x3C, 0x3D, 0x3E,
	0x3F, 0xFE, 0xFF
	};
		
static int show_registers(struct seq_file *m, void *data)
{
	struct bq25700 *bq = m->private;
	u8 i;
	int ret;
	u16 val;
	
	for (i = 0x0; i <= ARRAY_SIZE(reg_list); i++) {
		ret = bq25700_read_reg(bq, reg_list[i], &val);
		if (!ret)
			seq_printf(m, "Reg[%02X] = 0x%04X\n", reg_list[i], val);
	}
	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq25700 *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}

static const struct file_operations reg_debugfs_ops = {
	.owner = THIS_MODULE,
	.open = reg_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_debugfs_entry(struct bq25700 *bq)
{
	bq->debug_root = debugfs_create_dir("bq25700", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
				    bq->debug_root, bq, &reg_debugfs_ops);

		debugfs_create_x32("charging_disable_status", S_IFREG | S_IRUGO,
				   bq->debug_root,
				   &(bq->charging_disabled_status));

		debugfs_create_x32("fault_status", S_IFREG | S_IRUGO,
				   bq->debug_root, &(bq->fault_status));

		debugfs_create_x32("charge_state", S_IFREG | S_IRUGO,
				   bq->debug_root, &(bq->charge_state));

		debugfs_create_x32("skip_reads",
				   S_IFREG | S_IWUSR | S_IRUGO,
				   bq->debug_root, &(bq->skip_reads));
		debugfs_create_x32("skip_writes",
				   S_IFREG | S_IWUSR | S_IRUGO,
				   bq->debug_root, &(bq->skip_writes));
	}
}

static int bq25700_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq25700 *bq;
	struct power_supply *usb_psy = NULL;
	struct power_supply *bms_psy = NULL;

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

	bq = devm_kzalloc(&client->dev, sizeof(struct bq25700), GFP_KERNEL);
	if (!bq) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->usb_psy = usb_psy;
	bq->bms_psy = bms_psy;

	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->usb_online_lock);
	mutex_init(&bq->adc_lock);

	create_debugfs_entry(bq);
	ret = bq25700_detect_device(bq);
	if (ret) {
		pr_err("No bq25700 device found!\n");
		return -ENODEV;
	}

	if (client->dev.of_node)
		bq->platform_data = bq25700_parse_dt(&client->dev, bq);
	else
		bq->platform_data = client->dev.platform_data;

	if (!bq->platform_data) {
		pr_err("No platform data provided.\n");
		return -EINVAL;
	}

	ret = bq25700_init_device(bq);
	if (ret) {
		pr_err("Failed to init device\n");
		return ret;
	}

	ret = bq25700_psy_register(bq);
	if (ret)
		return ret;

	ret = bq25700_regulator_init(bq);
	if (ret) {
		pr_err("Couldn't initialize bq25700 regulator ret=%d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&bq->monitor_work, bq25700_monitor_workfunc);

	bq25700_update_charging_profile(bq);
	set_usb_present(bq);
	set_usb_online(bq);
	schedule_delayed_work(&bq->monitor_work, 0);

	pr_info("bq25700 probe successfully, ManufactureID:%d\n!",
		bq->dev_id);

	return 0;

}

static int bq25700_charger_remove(struct i2c_client *client)
{
	struct bq25700 *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->monitor_work);

	regulator_unregister(bq->otg_vreg.rdev);

	bq25700_psy_unregister(bq);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->usb_online_lock);
	mutex_destroy(&bq->adc_lock);

	debugfs_remove_recursive(bq->debug_root);

	return 0;
}

static void bq25700_charger_shutdown(struct i2c_client *client)
{
	pr_info("shutdown\n");
}

static struct of_device_id bq25700_charger_match_table[] = {
	{.compatible = "ti,bq25703",},
	{},
};

MODULE_DEVICE_TABLE(of, bq25700_charger_match_table);

static const struct i2c_device_id bq25700_charger_id[] = {
	{"bq25700-charger", BQ25700},
	{},
};

MODULE_DEVICE_TABLE(i2c, bq25700_charger_id);

static struct i2c_driver bq25700_charger_driver = {
	.driver = {
		   .name = "bq25700-charger",
		   .owner = THIS_MODULE,
		   .of_match_table = bq25700_charger_match_table,
		   },
	.id_table = bq25700_charger_id,

	.probe = bq25700_charger_probe,
	.remove = bq25700_charger_remove,
	.shutdown = bq25700_charger_shutdown,

};

module_i2c_driver(bq25700_charger_driver);

MODULE_DESCRIPTION("TI BQ2570x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
