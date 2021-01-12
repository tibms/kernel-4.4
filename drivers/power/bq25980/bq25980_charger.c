// SPDX-License-Identifier: GPL-2.0
// BQ25980 Battery Charger Driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "bq25980_charger.h"

#define CONFIG_INTERRUPT_AS_GPIO

struct bq25980_state {
	bool dischg;
	bool ovp;
	bool ocp;
	bool wdt;
	bool tflt;
	bool online;
	bool ce;
	bool hiz;
	bool bypass;

	u32 vbat_adc;
	u32 vsys_adc;
	u32 ibat_adc;
	u32 fault_status;
};

enum bq_work_mode {
	BQ_STANDALONE,
	BQ_SLAVE,
	BQ_MASTER,
};

#define BQ_MODE_COUNT 3

enum bq_device_id {
	BQ25980 = 0,
	BQ25960 = 8,
	BQ25975,
};

enum bq_compatible_id {
	BQ25960_STANDALONE,
	BQ25960_SLAVE,
	BQ25960_MASTER,

	BQ25980_STANDALONE,
	BQ25980_SLAVE,
	BQ25980_MASTER,

	BQ25975_STANDALONE,
};

struct bq25980_chip_info {

	int model_id;

	const struct regmap_config *regmap_config;

	const struct reg_default *reg_init_values;

	int busocp_sc_def;
	int busocp_byp_def;
	int busocp_sc_max;
	int busocp_byp_max;
	int busocp_sc_min;
	int busocp_byp_min;
	int busocp_step;
	int busocp_offset;

	int busovp_sc_def;
	int busovp_byp_def;
	int busovp_sc_step;

	int busovp_sc_offset;
	int busovp_byp_step;
	int busovp_byp_offset;
	int busovp_sc_min;
	int busovp_sc_max;
	int busovp_byp_min;
	int busovp_byp_max;

	int batovp_def;
	int batovp_max;
	int batovp_min;
	int batovp_step;
	int batovp_offset;

	int batocp_def;
	int batocp_max;

	int vac_sc_ovp;
	int vac_byp_ovp;

	int adc_curr_step;
	int adc_vbat_volt_step;
	int adc_vbus_volt_step;
	int adc_vbus_volt_offset;
};

struct bq25980_init_data {
	u32 ichg;
	u32 bypass_ilim;
	u32 sc_ilim;
	u32 vreg;
	u32 iterm;
	u32 iprechg;
	u32 bypass_vlim;
	u32 sc_vlim;
	u32 ichg_max;
	u32 vreg_max;
};

struct bq25980_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;
	struct power_supply *battery;
	struct mutex lock;
	struct regmap *regmap;

	char model_name[I2C_NAME_SIZE];

	struct bq25980_init_data init_data;
	const struct bq25980_chip_info *chip_info;
	struct bq25980_state state;
	int watchdog_timer;
	int mode;
	int device_id;
	struct power_supply_desc psy_desc;
};

static struct reg_default bq25980_reg_init_val[] = {
	{BQ25980_BATOVP,	0x69},//0x69:9100mV
	{BQ25980_BATOVP_ALM,	0x61},//0x61:8940mV
	{BQ25980_BATOCP,	0xEE},//0xEE:disable for dual //0x64:11000mA for standalone
	{BQ25980_BATOCP_ALM,	0x7F},//0x7F:12700mA
	{BQ25980_CHRGR_CFG_1,	0xA8},
	{BQ25980_CHRGR_CTRL_1,	0x4A},
	{BQ25980_BUSOVP,	0x3C},//0X3c:20000mV
	{BQ25980_BUSOVP_ALM,	0x32},//0X32:19000mV
	{BQ25980_BUSOCP,	0x0C},//0X0c:4200mA
	{BQ25980_REG_09,	0x8C},
	{BQ25980_TEMP_CONTROL,	0x2C},
	{BQ25980_TDIE_ALM,	0x78},//0x78:85C
	{BQ25980_TSBUS_FLT,	0x15},
	{BQ25980_TSBAT_FLG,	0x15},
	{BQ25980_VAC_CONTROL,	0xD8},//0xD8:22V
	{BQ25980_CHRGR_CTRL_2,	0x00},
	{BQ25980_CHRGR_CTRL_3,	0x74},//0x74:watchdog disable 5s,275kHz
	{BQ25980_CHRGR_CTRL_4,	0x01},
	{BQ25980_CHRGR_CTRL_5,	0x00},

	{BQ25980_MASK1,		0x00},
	{BQ25980_MASK2,		0x00},
	{BQ25980_MASK3,		0x00},
	{BQ25980_MASK4,		0x00},
	{BQ25980_MASK5,		0x80},

	{BQ25980_ADC_CONTROL1,	0x04},//sample 14 bit
	{BQ25980_ADC_CONTROL2,	0xE6},
};

static struct reg_default bq25960_reg_init_val[] = {
	{BQ25980_BATOVP,	0x69},//0x69:4550mV
	{BQ25980_BATOVP_ALM,	0x61},//0x61:4470mV
	{BQ25980_BATOCP,	0xEE},//0xEE:disable for dual //0x46:7000mA for standalone
	{BQ25980_BATOCP_ALM,	0x7F},//0x7F:12700mA
	{BQ25980_CHRGR_CFG_1,	0xA8},
	{BQ25980_CHRGR_CTRL_1,	0x49},
	{BQ25980_BUSOVP,	0x50},//0X50:11000mV
	{BQ25980_BUSOVP_ALM,	0x46},//0X46:10500mV
	{BQ25980_BUSOCP,	0x0C},//0X0c:4000mA
	{BQ25980_REG_09,	0x8C},
	{BQ25980_TEMP_CONTROL,	0x2C},
	{BQ25980_TDIE_ALM,	0x78},//0x78:85C
	{BQ25980_TSBUS_FLT,	0x15},
	{BQ25980_TSBAT_FLG,	0x15},
	{BQ25980_VAC_CONTROL,	0x48},//0x48:12V*2
	{BQ25980_CHRGR_CTRL_2,	0x00},
	{BQ25980_CHRGR_CTRL_3,	0x94},//0x94:watchdog disable 5s,500kHz
	{BQ25980_CHRGR_CTRL_4,	0x01},
	{BQ25980_CHRGR_CTRL_5,	0x00},

	{BQ25980_MASK1,		0x00},
	{BQ25980_MASK2,		0x00},
	{BQ25980_MASK3,		0x00},
	{BQ25980_MASK4,		0x00},
	{BQ25980_MASK5,		0x80},

	{BQ25980_ADC_CONTROL1,	0x04},//sample 14 bit
	{BQ25980_ADC_CONTROL2,	0xE6},

};

static struct reg_default bq25980_reg_defs[] = {
	{BQ25980_BATOVP, 0x5A},
	{BQ25980_BATOVP_ALM, 0x46},
	{BQ25980_BATOCP, 0x51},
	{BQ25980_BATOCP_ALM, 0x50},
	{BQ25980_CHRGR_CFG_1, 0x28},
	{BQ25980_CHRGR_CTRL_1, 0x0},
	{BQ25980_BUSOVP, 0x26},
	{BQ25980_BUSOVP_ALM, 0x22},
	{BQ25980_BUSOCP, 0xD},
	{BQ25980_REG_09, 0xC},
	{BQ25980_TEMP_CONTROL, 0x30},
	{BQ25980_TDIE_ALM, 0xC8},
	{BQ25980_TSBUS_FLT, 0x15},
	{BQ25980_TSBAT_FLG, 0x15},
	{BQ25980_VAC_CONTROL, 0x0},
	{BQ25980_CHRGR_CTRL_2, 0x0},
	{BQ25980_CHRGR_CTRL_3, 0x20},
	{BQ25980_CHRGR_CTRL_4, 0x1D},
	{BQ25980_CHRGR_CTRL_5, 0x18},
	{BQ25980_STAT1, 0x0},
	{BQ25980_STAT2, 0x0},
	{BQ25980_STAT3, 0x0},
	{BQ25980_STAT4, 0x0},
	{BQ25980_STAT5, 0x0},
	{BQ25980_FLAG1, 0x0},
	{BQ25980_FLAG2, 0x0},
	{BQ25980_FLAG3, 0x0},
	{BQ25980_FLAG4, 0x0},
	{BQ25980_FLAG5, 0x0},
	{BQ25980_MASK1, 0x0},
	{BQ25980_MASK2, 0x0},
	{BQ25980_MASK3, 0x0},
	{BQ25980_MASK4, 0x0},
	{BQ25980_MASK5, 0x0},
	{BQ25980_DEVICE_INFO, 0x8},
	{BQ25980_ADC_CONTROL1, 0x0},
	{BQ25980_ADC_CONTROL2, 0x0},
	{BQ25980_IBUS_ADC_LSB, 0x0},
	{BQ25980_IBUS_ADC_MSB, 0x0},
	{BQ25980_VBUS_ADC_LSB, 0x0},
	{BQ25980_VBUS_ADC_MSB, 0x0},
	{BQ25980_VAC1_ADC_LSB, 0x0},
	{BQ25980_VAC2_ADC_LSB, 0x0},
	{BQ25980_VOUT_ADC_LSB, 0x0},
	{BQ25980_VBAT_ADC_LSB, 0x0},
	{BQ25980_IBAT_ADC_MSB, 0x0},
	{BQ25980_IBAT_ADC_LSB, 0x0},
	{BQ25980_TSBUS_ADC_LSB, 0x0},
	{BQ25980_TSBAT_ADC_LSB, 0x0},
	{BQ25980_TDIE_ADC_LSB, 0x0},
	{BQ25980_DEGLITCH_TIME, 0x0},
	{BQ25980_CHRGR_CTRL_6, 0x0},
};

static struct reg_default bq25975_reg_defs[] = {
	{BQ25980_BATOVP, 0x5A},
	{BQ25980_BATOVP_ALM, 0x46},
	{BQ25980_BATOCP, 0x51},
	{BQ25980_BATOCP_ALM, 0x50},
	{BQ25980_CHRGR_CFG_1, 0x28},
	{BQ25980_CHRGR_CTRL_1, 0x0},
	{BQ25980_BUSOVP, 0x26},
	{BQ25980_BUSOVP_ALM, 0x22},
	{BQ25980_BUSOCP, 0xD},
	{BQ25980_REG_09, 0xC},
	{BQ25980_TEMP_CONTROL, 0x30},
	{BQ25980_TDIE_ALM, 0xC8},
	{BQ25980_TSBUS_FLT, 0x15},
	{BQ25980_TSBAT_FLG, 0x15},
	{BQ25980_VAC_CONTROL, 0x0},
	{BQ25980_CHRGR_CTRL_2, 0x0},
	{BQ25980_CHRGR_CTRL_3, 0x20},
	{BQ25980_CHRGR_CTRL_4, 0x1D},
	{BQ25980_CHRGR_CTRL_5, 0x18},
	{BQ25980_STAT1, 0x0},
	{BQ25980_STAT2, 0x0},
	{BQ25980_STAT3, 0x0},
	{BQ25980_STAT4, 0x0},
	{BQ25980_STAT5, 0x0},
	{BQ25980_FLAG1, 0x0},
	{BQ25980_FLAG2, 0x0},
	{BQ25980_FLAG3, 0x0},
	{BQ25980_FLAG4, 0x0},
	{BQ25980_FLAG5, 0x0},
	{BQ25980_MASK1, 0x0},
	{BQ25980_MASK2, 0x0},
	{BQ25980_MASK3, 0x0},
	{BQ25980_MASK4, 0x0},
	{BQ25980_MASK5, 0x0},
	{BQ25980_DEVICE_INFO, 0x8},
	{BQ25980_ADC_CONTROL1, 0x0},
	{BQ25980_ADC_CONTROL2, 0x0},
	{BQ25980_IBUS_ADC_LSB, 0x0},
	{BQ25980_IBUS_ADC_MSB, 0x0},
	{BQ25980_VBUS_ADC_LSB, 0x0},
	{BQ25980_VBUS_ADC_MSB, 0x0},
	{BQ25980_VAC1_ADC_LSB, 0x0},
	{BQ25980_VAC2_ADC_LSB, 0x0},
	{BQ25980_VOUT_ADC_LSB, 0x0},
	{BQ25980_VBAT_ADC_LSB, 0x0},
	{BQ25980_IBAT_ADC_MSB, 0x0},
	{BQ25980_IBAT_ADC_LSB, 0x0},
	{BQ25980_TSBUS_ADC_LSB, 0x0},
	{BQ25980_TSBAT_ADC_LSB, 0x0},
	{BQ25980_TDIE_ADC_LSB, 0x0},
	{BQ25980_DEGLITCH_TIME, 0x0},
	{BQ25980_CHRGR_CTRL_6, 0x0},
};

static struct reg_default bq25960_reg_defs[] = {
	{BQ25980_BATOVP, 0x5A},
	{BQ25980_BATOVP_ALM, 0x46},
	{BQ25980_BATOCP, 0x51},
	{BQ25980_BATOCP_ALM, 0x50},
	{BQ25980_CHRGR_CFG_1, 0x28},
	{BQ25980_CHRGR_CTRL_1, 0x0},
	{BQ25980_BUSOVP, 0x26},
	{BQ25980_BUSOVP_ALM, 0x22},
	{BQ25980_BUSOCP, 0xD},
	{BQ25980_REG_09, 0xC},
	{BQ25980_TEMP_CONTROL, 0x30},
	{BQ25980_TDIE_ALM, 0xC8},
	{BQ25980_TSBUS_FLT, 0x15},
	{BQ25980_TSBAT_FLG, 0x15},
	{BQ25980_VAC_CONTROL, 0x0},
	{BQ25980_CHRGR_CTRL_2, 0x0},
	{BQ25980_CHRGR_CTRL_3, 0x20},
	{BQ25980_CHRGR_CTRL_4, 0x1D},
	{BQ25980_CHRGR_CTRL_5, 0x18},
	{BQ25980_STAT1, 0x0},
	{BQ25980_STAT2, 0x0},
	{BQ25980_STAT3, 0x0},
	{BQ25980_STAT4, 0x0},
	{BQ25980_STAT5, 0x0},
	{BQ25980_FLAG1, 0x0},
	{BQ25980_FLAG2, 0x0},
	{BQ25980_FLAG3, 0x0},
	{BQ25980_FLAG4, 0x0},
	{BQ25980_FLAG5, 0x0},
	{BQ25980_MASK1, 0x0},
	{BQ25980_MASK2, 0x0},
	{BQ25980_MASK3, 0x0},
	{BQ25980_MASK4, 0x0},
	{BQ25980_MASK5, 0x0},
	{BQ25980_DEVICE_INFO, 0x8},
	{BQ25980_ADC_CONTROL1, 0x0},
	{BQ25980_ADC_CONTROL2, 0x0},
	{BQ25980_IBUS_ADC_LSB, 0x0},
	{BQ25980_IBUS_ADC_MSB, 0x0},
	{BQ25980_VBUS_ADC_LSB, 0x0},
	{BQ25980_VBUS_ADC_MSB, 0x0},
	{BQ25980_VAC1_ADC_LSB, 0x0},
	{BQ25980_VAC2_ADC_LSB, 0x0},
	{BQ25980_VOUT_ADC_LSB, 0x0},
	{BQ25980_VBAT_ADC_LSB, 0x0},
	{BQ25980_IBAT_ADC_MSB, 0x0},
	{BQ25980_IBAT_ADC_LSB, 0x0},
	{BQ25980_TSBUS_ADC_LSB, 0x0},
	{BQ25980_TSBAT_ADC_LSB, 0x0},
	{BQ25980_TDIE_ADC_LSB, 0x0},
	{BQ25980_DEGLITCH_TIME, 0x0},
	{BQ25980_CHRGR_CTRL_6, 0x0},
};

//static int bq25980_watchdog_time[BQ25980_NUM_WD_VAL] = {5000, 10000, 50000,
//							300000};

static void dump_reg(struct bq25980_device *bq, int start, int end)
{
	int ret;
	unsigned int val;
	int addr;

	for (addr = start; addr <= end; addr++) {
		ret = regmap_read(bq->regmap, addr, &val);
		if (!ret)
			dev_notice(bq->dev, "Reg[%02X] = 0x%02X\n", addr, val);
	}
}

static int bq25980_get_adc_enable(struct bq25980_device *bq)
{
	unsigned int reg_code;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_ADC_CONTROL1, &reg_code);
	if (ret)
		return ret;

	return !!(reg_code & BQ25980_ADC_EN);
}

static int bq25980_set_adc_enable(struct bq25980_device *bq, bool enable)
{
	int ret;

	dev_notice(bq->dev, "%s %d", __FUNCTION__, enable);

	if (enable)
		ret = regmap_update_bits(bq->regmap, BQ25980_ADC_CONTROL1,
				BQ25980_ADC_EN, BQ25980_ADC_EN);
	else
		ret = regmap_update_bits(bq->regmap, BQ25980_ADC_CONTROL1,
				BQ25980_ADC_EN, 0);

	return ret;
}

static int bq25980_get_input_curr_lim(struct bq25980_device *bq)
{
	unsigned int busocp_reg_code;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_BUSOCP, &busocp_reg_code);
	if (ret)
		return ret;

	return (busocp_reg_code * bq->chip_info->busocp_step) + bq->chip_info->busocp_offset;
}

static int bq25980_set_hiz(struct bq25980_device *bq, int setting)
{
	return regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
			BQ25980_EN_HIZ, setting);
}

static int bq25980_set_input_curr_lim(struct bq25980_device *bq, int busocp)
{
	unsigned int busocp_reg_code;

	if (!busocp)
		return bq25980_set_hiz(bq, BQ25980_ENABLE_HIZ);

	bq25980_set_hiz(bq, BQ25980_DISABLE_HIZ);

	if (bq->state.bypass) {
		busocp = max(busocp, bq->chip_info->busocp_byp_min);
		busocp = min(busocp, bq->chip_info->busocp_byp_max);
	} else {
		busocp = max(busocp, bq->chip_info->busocp_sc_min);
		busocp = min(busocp, bq->chip_info->busocp_sc_max);
	}
	
	busocp_reg_code = (busocp - bq->chip_info->busocp_offset)
						/ bq->chip_info->busocp_step;

	return regmap_write(bq->regmap, BQ25980_BUSOCP, busocp_reg_code);
}

static int bq25980_get_input_volt_lim(struct bq25980_device *bq)
{
	unsigned int busovp_reg_code;
	unsigned int busovp_offset;
	unsigned int busovp_step;
	int ret;

	if (bq->state.bypass) {
		busovp_step = bq->chip_info->busovp_byp_step;
		busovp_offset = bq->chip_info->busovp_byp_offset;
	} else {
		busovp_step = bq->chip_info->busovp_sc_step;
		busovp_offset = bq->chip_info->busovp_sc_offset;
	}

	ret = regmap_read(bq->regmap, BQ25980_BUSOVP, &busovp_reg_code);
	if (ret)
		return ret;

	return (busovp_reg_code * busovp_step) + busovp_offset;
}

static int bq25980_set_input_volt_lim(struct bq25980_device *bq, int busovp)
{
	unsigned int busovp_reg_code;
	unsigned int busovp_step;
	unsigned int busovp_offset;
	int ret;

	if (bq->state.bypass) {
		busovp_step = bq->chip_info->busovp_byp_step;
		busovp_offset = bq->chip_info->busovp_byp_offset;
		if (busovp > bq->chip_info->busovp_byp_max)
			busovp = bq->chip_info->busovp_byp_max;
		else if (busovp < bq->chip_info->busovp_byp_min)
			busovp = bq->chip_info->busovp_byp_min;
	} else {
		busovp_step = bq->chip_info->busovp_sc_step;
		busovp_offset = bq->chip_info->busovp_sc_offset;
		if (busovp > bq->chip_info->busovp_sc_max)
			busovp = bq->chip_info->busovp_sc_max;
		else if (busovp < bq->chip_info->busovp_sc_min)
			busovp = bq->chip_info->busovp_sc_min;
	}

	busovp_reg_code = (busovp - busovp_offset) / busovp_step;

	ret = regmap_write(bq->regmap, BQ25980_BUSOVP, busovp_reg_code);
	if (ret)
		return ret;

	return regmap_write(bq->regmap, BQ25980_BUSOVP_ALM, busovp_reg_code);
}

static int bq25980_get_const_charge_curr(struct bq25980_device *bq)
{
	unsigned int batocp_reg_code;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_BATOCP, &batocp_reg_code);
	if (ret)
		return ret;

	return (batocp_reg_code & BQ25980_BATOCP_MASK) *
						BQ25980_BATOCP_STEP_uA;
}

static int bq25980_set_const_charge_curr(struct bq25980_device *bq, int batocp)
{
	unsigned int batocp_reg_code;
	int ret;

	batocp = max(batocp, BQ25980_BATOCP_MIN_uA);
	batocp = min(batocp, bq->chip_info->batocp_max);

	batocp_reg_code = batocp / BQ25980_BATOCP_STEP_uA;

	ret = regmap_update_bits(bq->regmap, BQ25980_BATOCP,
				BQ25980_BATOCP_MASK, batocp_reg_code);
	if (ret)
		return ret;

	return regmap_update_bits(bq->regmap, BQ25980_BATOCP_ALM,
				BQ25980_BATOCP_MASK, batocp_reg_code);
}

static int bq25980_get_const_charge_volt(struct bq25980_device *bq)
{
	unsigned int batovp_reg_code;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_BATOVP, &batovp_reg_code);
	if (ret)
		return ret;

	return ((batovp_reg_code * bq->chip_info->batovp_step) +
			bq->chip_info->batovp_offset);
}

static int bq25980_set_const_charge_volt(struct bq25980_device *bq, int batovp)
{
	unsigned int batovp_reg_code;
	int ret;

	if (batovp < bq->chip_info->batovp_min)
		batovp = bq->chip_info->batovp_min;

	if (batovp > bq->chip_info->batovp_max)
		batovp = bq->chip_info->batovp_max;

	batovp_reg_code = (batovp - bq->chip_info->batovp_offset) /
						bq->chip_info->batovp_step;

	ret = regmap_write(bq->regmap, BQ25980_BATOVP, batovp_reg_code);
	if (ret)
		return ret;

	return regmap_write(bq->regmap, BQ25980_BATOVP_ALM, batovp_reg_code);
}

static int bq25980_set_bypass(struct bq25980_device *bq, bool en_bypass)
{
	int ret;

	if (en_bypass)
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_EN_BYPASS, BQ25980_EN_BYPASS);
	else
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_EN_BYPASS, en_bypass);
	if (ret)
		return ret;

	bq->state.bypass = en_bypass;

	return 0;
}

static int bq25980_set_chg_en(struct bq25980_device *bq, bool en_chg)
{
	int ret;

	if (en_chg)
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_CHG_EN, BQ25980_CHG_EN);
	else
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_CHG_EN, en_chg);
	if (ret)
		return ret;

	bq->state.ce = en_chg;

	return 0;
}

static int bq25980_get_adc_ibus(struct bq25980_device *bq)
{
	int ibus_adc_lsb, ibus_adc_msb;
	u16 ibus_adc;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_IBUS_ADC_MSB, &ibus_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_IBUS_ADC_LSB, &ibus_adc_lsb);
	if (ret)
		return ret;

	ibus_adc = (ibus_adc_msb << 8) | ibus_adc_lsb;

	if (ibus_adc_msb & BQ25980_ADC_POLARITY_BIT)
		return ((ibus_adc ^ 0xffff) + 1) * bq->chip_info->adc_curr_step;

	return ibus_adc * bq->chip_info->adc_curr_step;
}

static int bq25980_get_adc_vbus(struct bq25980_device *bq)
{
	int vbus_adc_lsb, vbus_adc_msb;
	u16 vbus_adc;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_VBUS_ADC_MSB, &vbus_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_VBUS_ADC_LSB, &vbus_adc_lsb);
	if (ret)
		return ret;

	vbus_adc = (vbus_adc_msb << 8) | vbus_adc_lsb;

	return bq->chip_info->adc_vbus_volt_offset + vbus_adc * bq->chip_info->adc_vbus_volt_step /10;
}

static int bq25980_get_ibat_adc(struct bq25980_device *bq)
{
	int ret;
	int ibat_adc_lsb, ibat_adc_msb;
	int ibat_adc;

	ret = regmap_read(bq->regmap, BQ25980_IBAT_ADC_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_IBAT_ADC_LSB, &ibat_adc_lsb);
	if (ret)
		return ret;

	ibat_adc = (ibat_adc_msb << 8) | ibat_adc_lsb;

	if (ibat_adc_msb & BQ25980_ADC_POLARITY_BIT)
		return ((ibat_adc ^ 0xffff) + 1) * BQ25960_ADC_CURR_STEP_uA;

	return ibat_adc * BQ25960_ADC_CURR_STEP_uA;
}

static int bq25980_get_adc_vbat(struct bq25980_device *bq)
{
	int vsys_adc_lsb, vsys_adc_msb;
	u16 vsys_adc;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_VBAT_ADC_MSB, &vsys_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_VBAT_ADC_LSB, &vsys_adc_lsb);
	if (ret)
		return ret;

	vsys_adc = (vsys_adc_msb << 8) | vsys_adc_lsb;

	return vsys_adc * bq->chip_info->adc_vbat_volt_step / 10;
}

static int bq25980_get_state(struct bq25980_device *bq,
				struct bq25980_state *state)
{
	unsigned int chg_ctrl_2;
	unsigned int stat1;
	unsigned int stat2;
	unsigned int stat3;
	unsigned int stat4;
	unsigned int ibat_adc_msb;
	int ret;

	ret = regmap_read(bq->regmap, BQ25980_STAT1, &stat1);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_STAT2, &stat2);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_STAT3, &stat3);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_STAT4, &stat4);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &chg_ctrl_2);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25980_IBAT_ADC_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	state->dischg = ibat_adc_msb & BQ25980_ADC_POLARITY_BIT;
	state->ovp = (stat1 & BQ25980_STAT1_OVP_MASK) |
		(stat3 & BQ25980_STAT3_OVP_MASK);
	state->ocp = (stat1 & BQ25980_STAT1_OCP_MASK) |
		(stat2 & BQ25980_STAT2_OCP_MASK);
	state->tflt = stat4 & BQ25980_STAT4_TFLT_MASK;
	state->wdt = stat4 & BQ25980_WD_STAT;
	state->online = stat3 & BQ25980_PRESENT_MASK;
	state->ce = chg_ctrl_2 & BQ25980_CHG_EN;
	state->hiz = chg_ctrl_2 & BQ25980_EN_HIZ;
	state->bypass = chg_ctrl_2 & BQ25980_EN_BYPASS;

	return 0;
}

static int bq25980_set_battery_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct bq25980_device *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq25980_set_const_charge_curr(bq, val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq25980_set_const_charge_volt(bq, val->intval);
		if (ret)
			return ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25980_get_battery_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq25980_device *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = bq->init_data.ichg_max;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = bq->init_data.vreg_max;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq25980_get_ibat_adc(bq);
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq25980_get_adc_vbat(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25980_set_charger_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct bq25980_device *bq = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25980_set_input_curr_lim(bq, val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq25980_set_input_volt_lim(bq, val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = bq25980_set_chg_en(bq, val->intval);
		if (ret)
			return ret;
		break;

	case POWER_SUPPLY_PROP_TI_ADC:
		ret = bq25980_set_adc_enable(bq, val->intval);
		if (ret)
			return ret;
		break;
		
	case POWER_SUPPLY_PROP_TI_BYPASS:
		ret = bq25980_set_bypass(bq, val->intval);
		if (ret)
			return ret;
		if (val->intval)
			ret = bq25980_set_input_curr_lim(bq, bq->chip_info->busocp_byp_def);
		else
			ret = bq25980_set_input_curr_lim(bq, bq->chip_info->busocp_sc_def);
		if (ret)
			return ret;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25980_get_charger_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq25980_device *bq = power_supply_get_drvdata(psy);
	struct bq25980_state state;
	int ret = 0;
	unsigned int chg_ctrl_2 = 0;
	unsigned int stat1 = 0;
	unsigned int stat2 = 0;
	unsigned int stat3 = 0;
	unsigned int stat4 = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ25980_MANUFACTURER;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model_name;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(bq->regmap, BQ25980_STAT3, &stat3);
		if (ret)
			return ret;

		state.online = stat3 & BQ25980_PRESENT_MASK;
		val->intval = state.online;
		break;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq25980_get_input_volt_lim(bq);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25980_get_input_curr_lim(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

		ret = regmap_read(bq->regmap, BQ25980_STAT1, &stat1);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, BQ25980_STAT2, &stat2);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, BQ25980_STAT3, &stat3);
		if (ret)
			return ret;

		ret = regmap_read(bq->regmap, BQ25980_STAT4, &stat4);
		if (ret)
			return ret;

		state.ovp = (stat1 & BQ25980_STAT1_OVP_MASK) |
			(stat3 & BQ25980_STAT3_OVP_MASK);
		state.ocp = (stat1 & BQ25980_STAT1_OCP_MASK) |
			(stat2 & BQ25980_STAT2_OCP_MASK);
		state.tflt = stat4 & BQ25980_STAT4_TFLT_MASK;

		if (state.tflt)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (state.ovp)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (state.ocp)
			val->intval = POWER_SUPPLY_HEALTH_OVERCURRENT;

		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq25980_get_adc_ibus(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq25980_get_adc_vbus(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq25980_get_const_charge_curr(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq25980_get_const_charge_volt(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &chg_ctrl_2);
		if (ret)
			return ret;

		state.ce = chg_ctrl_2 & BQ25980_CHG_EN;
		val->intval = state.ce;
		break;

	case POWER_SUPPLY_PROP_TI_ADC:
		ret = bq25980_get_adc_enable(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_TI_BYPASS:
		ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_2, &chg_ctrl_2);
		if (ret)
			return ret;

		state.bypass = chg_ctrl_2 & BQ25980_EN_BYPASS;
		val->intval = state.bypass;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static bool bq25980_state_changed(struct bq25980_device *bq,
				  struct bq25980_state *new_state)
{
	struct bq25980_state old_state;

	mutex_lock(&bq->lock);
	old_state = bq->state;
	mutex_unlock(&bq->lock);

	return (old_state.dischg != new_state->dischg ||
		old_state.ovp != new_state->ovp ||
		old_state.ocp != new_state->ocp ||
		old_state.online != new_state->online ||
		old_state.wdt != new_state->wdt ||
		old_state.tflt != new_state->tflt ||
		old_state.ce != new_state->ce ||
		old_state.hiz != new_state->hiz ||
		old_state.bypass != new_state->bypass);
}

static irqreturn_t bq25980_irq_handler_thread(int irq, void *private)
{
	struct bq25980_device *bq = private;
	struct bq25980_state state;
	int ret;
	
	dump_reg(bq, 0x13, 0x1C);

	ret = bq25980_get_state(bq, &state);
	if (ret < 0)
		goto irq_out;

	if (!bq25980_state_changed(bq, &state))
		goto irq_out;

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);

	power_supply_changed(bq->charger);

irq_out:
	return IRQ_HANDLED;
}

static enum power_supply_property bq25980_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_TI_ADC,
	POWER_SUPPLY_PROP_TI_BYPASS,
};

static enum power_supply_property bq25980_battery_props[] = {
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static char *bq25980_charger_supplied_to[] = {
	"main-battery",
};

static int bq25980_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_TI_ADC:
	case POWER_SUPPLY_PROP_TI_BYPASS:
		return true;
	default:
		return false;
	}
}

static struct power_supply_desc bq25980_battery_desc = {
	.name			= "bq25980-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= bq25980_get_battery_property,
	.set_property		= bq25980_set_battery_property,
	.properties		= bq25980_battery_props,
	.num_properties		= ARRAY_SIZE(bq25980_battery_props),
	.property_is_writeable	= bq25980_property_is_writeable,
};


static bool bq25980_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case BQ25980_CHRGR_CTRL_2:
	case BQ25980_STAT1...BQ25980_FLAG5:
	case BQ25980_ADC_CONTROL1...BQ25980_TDIE_ADC_LSB:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config bq25980_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ25980_CHRGR_CTRL_6,
	.reg_defaults	= bq25980_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25980_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = bq25980_is_volatile_reg,
};

static const struct regmap_config bq25975_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ25980_CHRGR_CTRL_6,
	.reg_defaults	= bq25975_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25975_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = bq25980_is_volatile_reg,
};

static const struct regmap_config bq25960_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ25980_CHRGR_CTRL_6,
	.reg_defaults	= bq25960_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25960_reg_defs),
	.cache_type = REGCACHE_NONE,
	.volatile_reg = bq25980_is_volatile_reg,
};

static const struct bq25980_chip_info bq25980_chip_info_tbl[] = {
	[BQ25980] = {
		.model_id = BQ25980,
		.regmap_config = &bq25980_regmap_config,
		.reg_init_values = bq25980_reg_init_val,

		.busocp_sc_def = BQ25980_BUSOCP_SC_DFLT_uA,
		.busocp_byp_def = BQ25980_BUSOCP_BYP_DFLT_uA,
		.busocp_sc_min = BQ25980_BUSOCP_MIN_uA,
		.busocp_sc_max = BQ25980_BUSOCP_SC_MAX_uA,
		.busocp_byp_max = BQ25980_BUSOCP_BYP_MAX_uA,
		.busocp_byp_min = BQ25980_BUSOCP_MIN_uA,
		.busocp_step = BQ25980_BUSOCP_STEP_uA,
		.busocp_offset = BQ25980_BUSOCP_OFFSET_uA,

		.busovp_sc_def = BQ25980_BUSOVP_DFLT_uV,
		.busovp_byp_def = BQ25980_BUSOVP_BYPASS_DFLT_uV,
		.busovp_sc_step = BQ25980_BUSOVP_SC_STEP_uV,
		.busovp_sc_offset = BQ25980_BUSOVP_SC_OFFSET_uV,
		.busovp_byp_step = BQ25980_BUSOVP_BYP_STEP_uV,
		.busovp_byp_offset = BQ25980_BUSOVP_BYP_OFFSET_uV,
		.busovp_sc_min = BQ25980_BUSOVP_SC_MIN_uV,
		.busovp_sc_max = BQ25980_BUSOVP_SC_MAX_uV,
		.busovp_byp_min = BQ25980_BUSOVP_BYP_MIN_uV,
		.busovp_byp_max = BQ25980_BUSOVP_BYP_MAX_uV,

		.batovp_def = BQ25980_BATOVP_DFLT_uV,
		.batovp_max = BQ25980_BATOVP_MAX_uV,
		.batovp_min = BQ25980_BATOVP_MIN_uV,
		.batovp_step = BQ25980_BATOVP_STEP_uV,
		.batovp_offset = BQ25980_BATOVP_OFFSET_uV,

		.batocp_def = BQ25980_BATOCP_DFLT_uA,
		.batocp_max = BQ25980_BATOCP_MAX_uA,

		.adc_curr_step = BQ25980_ADC_CURR_STEP_IBUS_uA,
		.adc_vbat_volt_step = BQ25980_ADC_VOLT_STEP_VBAT_deciuV,
		.adc_vbus_volt_step = BQ25980_ADC_VOLT_STEP_VBUS_deciuV,
		.adc_vbus_volt_offset = BQ25980_ADC_VOLT_OFFSET_VBUS,
	},

	[BQ25960] = {
		.model_id = BQ25960,
		.regmap_config = &bq25960_regmap_config,
		.reg_init_values = bq25960_reg_init_val,

		.busocp_sc_def = BQ25960_BUSOCP_SC_DFLT_uA,
		.busocp_byp_def = BQ25960_BUSOCP_BYP_DFLT_uA,
		.busocp_sc_min = BQ25960_BUSOCP_MIN_uA,
		.busocp_sc_max = BQ25960_BUSOCP_SC_MAX_uA,
		.busocp_byp_min = BQ25960_BUSOCP_MIN_uA,
		.busocp_byp_max = BQ25960_BUSOCP_BYP_MAX_uA,
		.busocp_step = BQ25960_BUSOCP_STEP_uA,
		.busocp_offset = BQ25960_BUSOCP_OFFSET_uA,

		.busovp_sc_def = BQ25975_BUSOVP_DFLT_uV,
		.busovp_byp_def = BQ25975_BUSOVP_BYPASS_DFLT_uV,
		.busovp_sc_step = BQ25960_BUSOVP_SC_STEP_uV,
		.busovp_sc_offset = BQ25960_BUSOVP_SC_OFFSET_uV,
		.busovp_byp_step = BQ25960_BUSOVP_BYP_STEP_uV,
		.busovp_byp_offset = BQ25960_BUSOVP_BYP_OFFSET_uV,
		.busovp_sc_min = BQ25960_BUSOVP_SC_MIN_uV,
		.busovp_sc_max = BQ25960_BUSOVP_SC_MAX_uV,
		.busovp_byp_min = BQ25960_BUSOVP_BYP_MIN_uV,
		.busovp_byp_max = BQ25960_BUSOVP_BYP_MAX_uV,

		.batovp_def = BQ25960_BATOVP_DFLT_uV,
		.batovp_max = BQ25960_BATOVP_MAX_uV,
		.batovp_min = BQ25960_BATOVP_MIN_uV,
		.batovp_step = BQ25960_BATOVP_STEP_uV,
		.batovp_offset = BQ25960_BATOVP_OFFSET_uV,

		.batocp_def = BQ25960_BATOCP_DFLT_uA,
		.batocp_max = BQ25960_BATOCP_MAX_uA,

		.adc_curr_step = BQ25960_ADC_CURR_STEP_uA,
		.adc_vbat_volt_step = BQ25960_ADC_VOLT_STEP_deciuV,
		.adc_vbus_volt_step = BQ25960_ADC_VOLT_STEP_deciuV,
		.adc_vbus_volt_offset = 0,
	},

	[BQ25975] = {
		.model_id = BQ25975,
		.regmap_config = &bq25975_regmap_config,
	
		.busocp_sc_def = BQ25975_BUSOCP_DFLT_uA,
		.busocp_byp_def = BQ25975_BUSOCP_DFLT_uA,
		.busocp_sc_min = BQ25960_BUSOCP_MIN_uA,
		.busocp_sc_max = BQ25975_BUSOCP_SC_MAX_uA,
		.busocp_byp_min = BQ25960_BUSOCP_MIN_uA,
		.busocp_byp_max = BQ25975_BUSOCP_BYP_MAX_uA,
		.busocp_step = BQ25960_BUSOCP_STEP_uA,
		.busocp_offset = BQ25960_BUSOCP_OFFSET_uA,
	
		.busovp_sc_def = BQ25975_BUSOVP_DFLT_uV,
		.busovp_byp_def = BQ25975_BUSOVP_BYPASS_DFLT_uV,
		.busovp_sc_step = BQ25975_BUSOVP_SC_STEP_uV,
		.busovp_sc_offset = BQ25975_BUSOVP_SC_OFFSET_uV,
		.busovp_byp_step = BQ25975_BUSOVP_BYP_STEP_uV,
		.busovp_byp_offset = BQ25975_BUSOVP_BYP_OFFSET_uV,
		.busovp_sc_min = BQ25975_BUSOVP_SC_MIN_uV,
		.busovp_sc_max = BQ25975_BUSOVP_SC_MAX_uV,
		.busovp_byp_min = BQ25975_BUSOVP_BYP_MIN_uV,
		.busovp_byp_max = BQ25975_BUSOVP_BYP_MAX_uV,
	
		.batovp_def = BQ25975_BATOVP_DFLT_uV,
		.batovp_max = BQ25975_BATOVP_MAX_uV,
		.batovp_min = BQ25975_BATOVP_MIN_uV,
		.batovp_step = BQ25975_BATOVP_STEP_uV,
		.batovp_offset = BQ25975_BATOVP_OFFSET_uV,
	
		.batocp_def = BQ25980_BATOCP_DFLT_uA,
		.batocp_max = BQ25980_BATOCP_MAX_uA,

		.adc_curr_step = BQ25960_ADC_CURR_STEP_uA,
		.adc_vbat_volt_step = BQ25960_ADC_VOLT_STEP_deciuV,
		.adc_vbus_volt_step = BQ25960_ADC_VOLT_STEP_deciuV,
		.adc_vbus_volt_offset = 0,
	},

};

static int bq25980_power_supply_init(struct bq25980_device *bq,
							struct device *dev,
							int driver_data)
{
	struct power_supply_config psy_cfg = { .drv_data = bq,
						.of_node = dev->of_node, };

	psy_cfg.supplied_to = bq25980_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(bq25980_charger_supplied_to);

	switch (driver_data) {
	case BQ25980_MASTER:
		bq->psy_desc.name = "bq25980-master";
		break;
	case BQ25980_SLAVE:
		bq->psy_desc.name = "bq25980-slave";
		break;
	case BQ25980_STANDALONE:
		bq->psy_desc.name = "bq25980-standalone";
		break;
	case BQ25960_MASTER:
		bq->psy_desc.name = "bq25960-master";
		break;
	case BQ25960_SLAVE:
		bq->psy_desc.name = "bq25960-slave";
		break;
	case BQ25960_STANDALONE:
		bq->psy_desc.name = "bq25960-standalone";
		break;
	default:
		return -EINVAL;
	}

	bq->psy_desc.type = POWER_SUPPLY_TYPE_MAINS,
	bq->psy_desc.properties = bq25980_power_supply_props,
	bq->psy_desc.num_properties = ARRAY_SIZE(bq25980_power_supply_props),
	bq->psy_desc.get_property = bq25980_get_charger_property,
	bq->psy_desc.set_property = bq25980_set_charger_property,
	bq->psy_desc.property_is_writeable = bq25980_property_is_writeable,

	bq->charger = devm_power_supply_register(bq->dev,
						 &bq->psy_desc,
						 &psy_cfg);
	if (IS_ERR(bq->charger)) {
		dev_err(bq->dev, "bq register power supply fail");
		return -EINVAL;
	}

	if (bq->mode != BQ_SLAVE) {
		bq->battery = devm_power_supply_register(bq->dev,
						      &bq25980_battery_desc,
						      &psy_cfg);
		if (IS_ERR(bq->battery)) {
			dev_err(bq->dev, "battery register power supply fail");
			return -EINVAL;
		}
	}
	
	return 0;
}

static int bq25980_reg_init(struct bq25980_device *bq)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(bq25980_reg_init_val); i++) {
		ret = regmap_update_bits(bq->regmap, bq->chip_info->reg_init_values[i].reg,
			0xFF, bq->chip_info->reg_init_values[i].def);
		dev_notice(bq->dev, "init Reg[%02X] = 0x%02X\n",
			bq->chip_info->reg_init_values[i].reg,
			bq->chip_info->reg_init_values[i].def);
		if (ret)
		{
			dev_err(bq->dev, "Reg init fail ret=%d", ret);
			return ret;
		}
	}
	return 0;
}
#if 0
static int bq25980_hw_init(struct bq25980_device *bq)
{
	struct power_supply_battery_info bat_info = { };
	int wd_reg_val;
	int ret = 0;
	int curr_val;
	int volt_val;
	int i;

	if (!bq->watchdog_timer) {
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_3,
					 BQ25980_WATCHDOG_DIS,
					 BQ25980_WATCHDOG_DIS);
	} else {
		for (i = 0; i < BQ25980_NUM_WD_VAL; i++) {
			if (bq->watchdog_timer > bq25980_watchdog_time[i] &&
			    bq->watchdog_timer < bq25980_watchdog_time[i + 1]) {
				wd_reg_val = i;
				break;
			}
		}

		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_3,
					BQ25980_WATCHDOG_MASK, wd_reg_val);
	}
	if (ret)
		return ret;

	ret = power_supply_get_battery_info(bq->charger, &bat_info);
	if (ret) {
		dev_warn(bq->dev, "battery info missing\n");
		return -EINVAL;
	}

	bq->init_data.ichg_max = bat_info.constant_charge_current_max_ua;
	bq->init_data.vreg_max = bat_info.constant_charge_voltage_max_uv;

	if (bq->state.bypass) {
		ret = regmap_update_bits(bq->regmap, BQ25980_CHRGR_CTRL_2,
					BQ25980_EN_BYPASS, BQ25980_EN_BYPASS);
		if (ret)
			return ret;

		curr_val = bq->init_data.bypass_ilim;
		volt_val = bq->init_data.bypass_vlim;
	} else {
		curr_val = bq->init_data.sc_ilim;
		volt_val = bq->init_data.sc_vlim;
	}

	ret = bq25980_set_input_curr_lim(bq, curr_val);
	if (ret)
		return ret;

	ret = bq25980_set_input_volt_lim(bq, volt_val);
	if (ret)
		return ret;

	return regmap_update_bits(bq->regmap, BQ25980_ADC_CONTROL1,
				 BQ25980_ADC_EN, BQ25980_ADC_EN);
}
#endif
static int bq25980_parse_dt(struct bq25980_device *bq)
{
	int ret;

	ret = device_property_read_u32(bq->dev, "ti,watchdog-timeout-ms",
				       &bq->watchdog_timer);
	if (ret)
		bq->watchdog_timer = BQ25980_WATCHDOG_MIN;

	if (bq->watchdog_timer > BQ25980_WATCHDOG_MAX ||
	    bq->watchdog_timer < BQ25980_WATCHDOG_MIN)
		return -EINVAL;

	ret = device_property_read_u32(bq->dev,
				       "ti,sc-ovp-limit-microvolt",
				       &bq->init_data.sc_vlim);
	if (ret)
		bq->init_data.sc_vlim = bq->chip_info->busovp_sc_def;

	if (bq->init_data.sc_vlim > bq->chip_info->busovp_sc_max ||
	    bq->init_data.sc_vlim < bq->chip_info->busovp_sc_min) {
		dev_err(bq->dev, "SC ovp limit is out of range\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(bq->dev,
				       "ti,sc-ocp-limit-microamp",
				       &bq->init_data.sc_ilim);
	if (ret)
		bq->init_data.sc_ilim = bq->chip_info->busocp_sc_def;

	if (bq->init_data.sc_ilim > bq->chip_info->busocp_sc_max ||
	    bq->init_data.sc_ilim < bq->chip_info->busocp_sc_min) {
		dev_err(bq->dev, "SC ocp limit is out of range\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(bq->dev,
				       "ti,bypass-ovp-limit-microvolt",
				       &bq->init_data.bypass_vlim);
	if (ret)
		bq->init_data.bypass_vlim = bq->chip_info->busovp_byp_def;

	if (bq->init_data.bypass_vlim > bq->chip_info->busovp_byp_max ||
	    bq->init_data.bypass_vlim < bq->chip_info->busovp_byp_min) {
		dev_err(bq->dev, "Bypass ovp limit is out of range\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(bq->dev,
				       "ti,bypass-ocp-limit-microamp",
				       &bq->init_data.bypass_ilim);
	if (ret)
		bq->init_data.bypass_ilim = bq->chip_info->busocp_byp_def;

	if (bq->init_data.bypass_ilim > bq->chip_info->busocp_byp_max ||
	    bq->init_data.bypass_ilim < bq->chip_info->busocp_byp_min) {
		dev_err(bq->dev, "Bypass ocp limit is out of range\n");
		return -EINVAL;
	}


	bq->state.bypass = device_property_read_bool(bq->dev,
						      "ti,bypass-enable");
	return 0;
}

static int bq25980_check_work_mode(struct bq25980_device *bq)
{
	int ret;
	int val;

	ret = regmap_read(bq->regmap, BQ25980_CHRGR_CTRL_5, &val);
	if (ret) {
		dev_err(bq->dev, "Failed to read operation mode register\n");
		return ret;
	}

	val = (val & BQ25980_MS_MASK);
	if (bq->mode != val) {
		dev_err(bq->dev, "dts mode %d mismatch with hardware mode %d\n", bq->mode, val);
		return -EINVAL;
	}
	
	dev_info(bq->dev, "work mode:%s\n", bq->mode == BQ_STANDALONE ? "Standalone" :
			(bq->mode == BQ_SLAVE ? "Slave" : "Master"));
	return 0;
}

static int bq25980_check_device_id(struct bq25980_device *bq)
{
#if 0 //IC device id not confirmed
	int ret;
	int val;

	ret = regmap_read(bq->regmap, BQ25980_DEVICE_INFO, &val);
	if (ret) {
		dev_err(bq->dev, "Failed to read device id\n");
		return ret;
	}

	val = (val & BQ25980_DEVICE_ID_MASK);
	if (bq->device_id != val) {
		dev_err(bq->dev, "dts id %d mismatch with hardware id %d\n", bq->device_id, val);
		return -EINVAL;
	}
#endif
	return 0;
}

static int bq25980_parse_dt_id(struct bq25980_device *bq, int driver_data)
{
	switch (driver_data) {
	case BQ25960_STANDALONE:
		bq->device_id = BQ25960;
		bq->mode = BQ_STANDALONE;
		break;
	case BQ25960_SLAVE:
		bq->device_id = BQ25960;
		bq->mode = BQ_SLAVE;
		break;
	case BQ25960_MASTER:
		bq->device_id = BQ25960;
		bq->mode = BQ_MASTER;
		break;
	case BQ25980_STANDALONE:
		bq->device_id = BQ25980;
		bq->mode = BQ_STANDALONE;
		break;
	case BQ25980_SLAVE:
		bq->device_id = BQ25980;
		bq->mode = BQ_SLAVE;
		break;
	case BQ25980_MASTER:
		bq->device_id = BQ25980;
		bq->mode = BQ_MASTER;
		break;
	default:
		dev_err(bq->dev, "dts compatible id %d is unknown", driver_data);
		return -EINVAL;
		break;
	}

	return 0;
}

static int bq25980_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct bq25980_device *bq;
	int ret, irq_gpio, irqn;

	bq = devm_kzalloc(dev, sizeof(*bq), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->client = client;
	bq->dev = dev;

	mutex_init(&bq->lock);

	strncpy(bq->model_name, id->name, I2C_NAME_SIZE);

	ret = bq25980_parse_dt_id(bq, id->driver_data);
	if (ret)
		return ret;

	bq->chip_info = &bq25980_chip_info_tbl[bq->device_id];

	bq->regmap = devm_regmap_init_i2c(client,
					  bq->chip_info->regmap_config);
	if (IS_ERR(bq->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		return PTR_ERR(bq->regmap);
	}

	i2c_set_clientdata(client, bq);

	ret = bq25980_check_device_id(bq);
	if (ret)
		return ret;

	ret = bq25980_check_work_mode(bq);
	if (ret)
		return ret;

	ret = bq25980_parse_dt(bq);
	if (ret) {
		dev_err(dev, "Failed to read device tree properties%d\n", ret);
		return ret;
	}

#ifdef CONFIG_INTERRUPT_AS_GPIO
	irq_gpio = of_get_named_gpio(client->dev.of_node, "ti,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio))
	{
		dev_err(bq->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		return -EINVAL;
	}
	ret = gpio_request(irq_gpio, "bq25980 irq pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, irq_gpio);
		return ret;
	}
	gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(bq->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		return irqn;
	}
	client->irq = irqn;
#endif

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						bq25980_irq_handler_thread,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), bq);
		if (ret)
			return ret;
	}

	ret = bq25980_power_supply_init(bq, dev, id->driver_data);
	if (ret)
		return ret;

	ret = bq25980_reg_init(bq);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		return ret;
	}

	dump_reg(bq, 0x00, 0x3A);

	return 0;
}

static const struct i2c_device_id bq25980_i2c_ids[] = {
	{ "bq25980-standalone", BQ25980_STANDALONE },
	{ "bq25980-master", BQ25980_MASTER },
	{ "bq25980-slave", BQ25980_SLAVE },
	{ "bq25960-standalone", BQ25960_STANDALONE },
	{ "bq25960-master", BQ25960_MASTER },
	{ "bq25960-slave", BQ25960_SLAVE },
	{ "bq25975", BQ25975 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25980_i2c_ids);

static const struct of_device_id bq25980_of_match[] = {
	{ .compatible = "ti,bq25980-standalone", .data = (void *)BQ25980_STANDALONE},
	{ .compatible = "ti,bq25980-master", .data = (void *)BQ25980_MASTER},
	{ .compatible = "ti,bq25980-slave", .data = (void *)BQ25980_SLAVE},
	{ .compatible = "ti,bq25960-standalone", .data = (void *)BQ25960_STANDALONE},
	{ .compatible = "ti,bq25960-master", .data = (void *)BQ25960_MASTER},
	{ .compatible = "ti,bq25960-slave", .data = (void *)BQ25960_SLAVE},
	{ .compatible = "ti,bq25975", .data = (void *)BQ25975 },
	{ },
};
MODULE_DEVICE_TABLE(of, bq25980_of_match);

static struct i2c_driver bq25980_driver = {
	.driver = {
		.name = "bq25980-charger",
		.of_match_table = bq25980_of_match,
	},
	.probe = bq25980_probe,
	.id_table = bq25980_i2c_ids,
};
module_i2c_driver(bq25980_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_AUTHOR("Ricardo Rivera-Matos <r-rivera-matos@ti.com>");
MODULE_DESCRIPTION("bq25980 charger driver");
MODULE_LICENSE("GPL v2");
