
/*
 * BQ24725A battery charging driver
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

#define pr_fmt(fmt)	"[bq24725] %s: " fmt, __func__
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include <linux/power/bq24725-charger.h>

#define bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_err
#define bq_log	pr_err


#define BQ24725_CHG_OPT			0x12
#define BQ24725_CHG_OPT_CHARGE_DISABLE	(1 << 0)
#define BQ24725_CHG_OPT_CHARGE_ENABLE	(0)
#define BQ24725_CHG_OPT_CHARGE_MASK	1
#define BQ24725_CHG_OPT_AC_PRESENT	(1 << 4)
#define BQ24725_CHG_OPT_WDT_DISABLE	(0)
#define BQ24725_CHG_OPT_WDT_MASK	0x6000

#define BQ24725_CHARGE_CURRENT		0x14
#define BQ24725_CHARGE_CURRENT_BASE	0
#define BQ24725_CHARGE_CURRENT_LSB	64	/*assume 10mohm sns*/
#define BQ24725_CHARGE_CURRENT_SHIFT	6
#define BQ24725_CHARGE_CURRENT_MASK	0x1FC0

#define BQ24725_CHARGE_VOLTAGE		0x15
#define BQ24725_CHARGE_VOLTAGE_BASE	0
#define BQ24725_CHARGE_VOLTAGE_LSB	16
#define BQ24725_CHARGE_VOLTAGE_SHIFT	4
#define BQ24725_CHARGE_VOLTAGE_MASK	0x7FF0

#define BQ24725_INPUT_CURRENT		0x3F
#define BQ24725_INPUT_CURRENT_BASE	0
#define BQ24725_INPUT_CURRENT_LSB	128
#define BQ24725_INPUT_CURRENT_SHIFT	7
#define BQ24725_INPUT_CURRENT_MASK	0x1F80

#define BQ24725_MANUFACTURER_ID		0xFE
#define BQ24725_DEVICE_ID		0xFF



struct bq24725 {
	struct i2c_client *client;

	struct mutex i2c_rw_lock;

	/* here we use delay work, if system could sleep during charge,
	 * suggest to use alarm timer
	 * */
	struct delayed_work charging_work;

	struct power_supply *chg_psy;
	struct power_supply_desc chg_psy_d;

	struct bq24725_platform *pdata;

	int skip_writes;
	int skip_reads;

};


static inline int __bq24725_read_reg(struct bq24725 *bq, u8 reg)
{
	s32 ret = i2c_smbus_read_word_data(bq->client, reg);

	if (ret < 0) {
		bq_err("SMBus Read Fail,Cmd:0x%02X, ret:%d\n", reg, ret);
		return ret;
	}

	return le16_to_cpu(ret);
}

static inline int __bq24725_write_reg(struct bq24725 *bq, u8 reg, u16 value)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(bq->client, reg, le16_to_cpu(value));

	if (ret < 0) {
		bq_err("SMBus Write Fail,Cmd:0x%02X,Val:0x%02x,ret:%d\n",
			reg, value, ret);
		return ret;
	}
	return 0;
}

static inline int bq24725_write_reg(struct bq24725 *bq, u8 reg,
				     u16 value)
{
	s32 ret;

	if (bq->skip_reads || bq->skip_writes)
		return 0;


	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq24725_write_reg(bq, reg, value);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static inline int bq24725_read_reg(struct bq24725 *bq, u8 reg)
{
	s32 ret;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq24725_read_reg(bq, reg);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq24725_update_reg(struct bq24725 *bq, u8 reg,
			       u16 mask, u16 value)
{
	u16 tmp;
	int ret;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);

	ret = __bq24725_read_reg(bq, reg);
	if (ret < 0)
		goto out;

	tmp = ret & ~mask;
	tmp |= value & mask;

	ret = __bq24725_write_reg(bq, reg, tmp);
out:
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static inline int bq24725_enable_charging(struct bq24725 *bq)
{
	return bq24725_update_reg(bq, BQ24725_CHG_OPT,
				   BQ24725_CHG_OPT_CHARGE_MASK,
				   BQ24725_CHG_OPT_CHARGE_ENABLE);
}

static inline int bq24725_disable_charging(struct bq24725 *bq)
{
	return bq24725_update_reg(bq, BQ24725_CHG_OPT,
				   BQ24725_CHG_OPT_CHARGE_MASK,
				   BQ24725_CHG_OPT_CHARGE_DISABLE);
}

static int bq24725_set_charge_current(struct bq24725 *bq, int chg_current)
{
	int ret;
	u16 reg_val;

	if (chg_current < BQ24725_CHARGE_CURRENT_BASE)
		chg_current = BQ24725_CHARGE_CURRENT_BASE;

	reg_val = chg_current - BQ24725_CHARGE_CURRENT_BASE;
	reg_val /= BQ24725_CHARGE_CURRENT_LSB;
	reg_val <<= BQ24725_CHARGE_CURRENT_SHIFT;
	reg_val &= BQ24725_CHARGE_CURRENT_MASK;

	ret = bq24725_write_reg(bq, BQ24725_CHARGE_CURRENT, reg_val);

	return ret;
}

static int bq24725_set_charge_voltage(struct bq24725 *bq, int chg_voltage)
{
	int ret;
	u16 reg_val;

	if (chg_voltage < BQ24725_CHARGE_VOLTAGE_BASE)
		chg_voltage = BQ24725_CHARGE_VOLTAGE_BASE;
		
	reg_val = chg_voltage - BQ24725_CHARGE_VOLTAGE_BASE;
	reg_val /= BQ24725_CHARGE_VOLTAGE_LSB;
	reg_val <<= BQ24725_CHARGE_VOLTAGE_SHIFT;
	reg_val &= BQ24725_CHARGE_VOLTAGE_MASK;

	ret = bq24725_write_reg(bq, BQ24725_CHARGE_VOLTAGE, reg_val);

	return ret;

}

static int bq24725_set_input_current(struct bq24725 *bq, int in_current)
{
	int ret;
	u16 reg_val;

	if (in_current < BQ24725_INPUT_CURRENT_BASE)
		in_current = BQ24725_INPUT_CURRENT_BASE;

	reg_val = in_current - BQ24725_INPUT_CURRENT_BASE;
	reg_val /= BQ24725_INPUT_CURRENT_LSB;
	reg_val <<= BQ24725_INPUT_CURRENT_SHIFT;
	reg_val &= BQ24725_INPUT_CURRENT_MASK;

	ret = bq24725_write_reg(bq, BQ24725_INPUT_CURRENT, reg_val);

	return ret;
}


static int bq24725_config_charger(struct bq24725 *bq)
{
	struct bq24725_platform *pdata = bq->pdata;
	int ret;

	if (pdata->charge_current) {
		ret = bq24725_set_charge_current(bq, pdata->charge_current);
		if (ret < 0) {
			bq_err("Failed to write charger current : %d\n", ret);
			return ret;
		}
	}

	if (pdata->charge_voltage) {
		ret = bq24725_set_charge_voltage(bq, pdata->charge_voltage);
		if (ret < 0) {
			bq_err("Failed to write charger voltage : %d\n", ret);
			return ret;
		}
	}

	if (pdata->input_current) {
		ret = bq24725_set_input_current(bq, pdata->input_current);
		if (ret < 0) {
			bq_err("Failed to write input current : %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static bool bq24725_charger_is_present(struct bq24725 *bq)
{
	struct bq24725_platform *pdata = bq->pdata;
	int ret;

	if (pdata->status_gpio_valid) {
		ret = gpio_get_value_cansleep(pdata->status_gpio);
		return ret ^= pdata->status_gpio_active_low == 0;
	} else {
		int ac = 0;

		ac = bq24725_read_reg(bq, BQ24725_CHG_OPT);
		if (ac < 0) {
			bq_err("Failed to read charger options : %d\n",	ac);
			return false;
		}
		return (ac & BQ24725_CHG_OPT_AC_PRESENT) ? true : false;
	}

	return false;
}


static void bq24725_charging_workfunc(struct work_struct *work)
{
	struct bq24725 *bq = container_of(work, struct bq24725,
				charging_work.work);
	
	bq24725_config_charger(bq);

	if (bq24725_charger_is_present(bq) && bq->pdata->enable_wdt)
		schedule_delayed_work(&bq->charging_work, 60 * HZ);
}


static irqreturn_t bq24725_charger_interrupt(int irq, void *dev_id)
{
	struct bq24725 *bq =  dev_id;

	if (bq24725_charger_is_present(bq)) {
		if (bq->pdata->enable_wdt)
			schedule_delayed_work(&bq->charging_work, 0);
		bq24725_enable_charging(bq);
	} else {
		if (bq->pdata->enable_wdt)
			cancel_delayed_work(&bq->charging_work);
		bq24725_disable_charging(bq);
	}

	power_supply_changed(bq->chg_psy);

	return IRQ_HANDLED;
}


static enum power_supply_property bq24725_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	/* bq24725 has no charge termination feature, we have this 
	 * interface so that gauge/system can disable charging when
	 * it detects that battery is fully charged and re-enable 
	 * charging again when battery voltage drops
	 * */
	POWER_SUPPLY_PROP_CHARGING_ENABLED,

};

static int bq24725_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq24725 *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq24725_charger_is_present(bq) ? 1 : 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq24725_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct bq24725 *bq = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (val->intval)
			bq24725_enable_charging(bq);
		else
			bq24725_disable_charging(bq);
		break;
	default:
		bq_err("unsupported property set\n");
		break;
	}

	return rc;

}

static int bq24725_charger_is_writeable(struct power_supply *psy,
					enum power_supply_property prop)
{

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		return 1;
	default:
		return 0;
	}
}

static int bq24725_psy_register(struct bq24725 *bq)
{
	struct power_supply_config chg_psy_cfg = {};
	int ret;

	bq->chg_psy_d.name = "bq24725-charger";
	bq->chg_psy_d.type = POWER_SUPPLY_TYPE_MAINS;
	bq->chg_psy_d.properties = bq24725_charger_props;
	bq->chg_psy_d.num_properties = ARRAY_SIZE(bq24725_charger_props);
	bq->chg_psy_d.get_property = bq24725_charger_get_property;
	bq->chg_psy_d.set_property = bq24725_charger_set_property;
	bq->chg_psy_d.property_is_writeable = bq24725_charger_is_writeable;

	chg_psy_cfg.drv_data = bq;
	chg_psy_cfg.num_supplicants = 0;

	bq->chg_psy = devm_power_supply_register(&bq->client->dev,
			&bq->chg_psy_d,
			&chg_psy_cfg);
	
	if (IS_ERR(bq->chg_psy)) {
		bq_err("Couldn't register charger psy rc=%ld\n",
				PTR_ERR(bq->chg_psy));
		ret = PTR_ERR(bq->chg_psy);
		return ret;
	}

	
	return 0;

}

static int bq24725_detect_device(struct bq24725 *bq)
{
	int ret;

	ret = bq24725_read_reg(bq, BQ24725_MANUFACTURER_ID);
	if (ret < 0) {
		bq_err("Failed to read manufacturer id : %d\n",	ret);
		return ret;
	} else if (ret != 0x0040) {
		bq_err("manufacturer id mismatch. 0x0040 != 0x%04x\n", ret);
		return -ENODEV;
	}

	ret = bq24725_read_reg(bq, BQ24725_DEVICE_ID);
	if (ret < 0) {
		bq_err("Failed to read device id : %d\n", ret);
		return ret;
	} else if (ret != 0x000B) {
		bq_err(	"device id mismatch. 0x000b != 0x%04x\n", ret);
		return -ENODEV;
	}

	return 0;
}


static struct bq24725_platform *bq24725_parse_dt_data(struct i2c_client *client)
{
	struct bq24725_platform *pdata;
	struct device_node *np = client->dev.of_node;
	u32 val;
	int ret;
	enum of_gpio_flags flags;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev,
			"Memory alloc for bq24725 pdata failed\n");
		return NULL;
	}

	pdata->status_gpio = of_get_named_gpio_flags(np, "ti,ac-detect-gpios",
						     0, &flags);

	if (flags & OF_GPIO_ACTIVE_LOW)
		pdata->status_gpio_active_low = 1;

	ret = of_property_read_u32(np, "ti,charge-current", &val);
	if (!ret)
		pdata->charge_current = val;

	ret = of_property_read_u32(np, "ti,charge-voltage", &val);
	if (!ret)
		pdata->charge_voltage = val;

	ret = of_property_read_u32(np, "ti,input-current", &val);
	if (!ret)
		pdata->input_current = val;

	pdata->enable_wdt = of_property_read_bool(np, "ti,enable-wdt");

	return pdata;
}


static int bq24725_init_device(struct bq24725 *bq)
{
	int ret;

	ret = bq24725_config_charger(bq);
	if (ret < 0)
		return ret;

	if (!bq->pdata->enable_wdt) { 
		ret = bq24725_update_reg(bq, BQ24725_CHG_OPT,
				BQ24725_CHG_OPT_WDT_MASK,
				BQ24725_CHG_OPT_WDT_DISABLE);
		if (ret < 0) {
			bq_err("Failed to disable watchdog timer:%d\n", ret);
			/*assume it is enabled, feed watchdog*/
			bq->pdata->enable_wdt = true;
		}
	}

	return 0;
}

static void determine_initial_status(struct bq24725* bq)
{
	bq24725_charger_interrupt(bq->client->irq, bq);	
}

static int bq24725_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret;
	struct bq24725 *bq;

	bq = devm_kzalloc(&client->dev, sizeof(struct bq24725), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->client = client;
	bq->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);

	ret = bq24725_detect_device(bq);
	if (ret < 0)
		return -ENODEV;


	if (IS_ENABLED(CONFIG_OF) && !bq->pdata && client->dev.of_node)
		bq->pdata = bq24725_parse_dt_data(client);

	if (!bq->pdata) {
		bq_err("no platform data provided\n");
		return -EINVAL;
	}

	INIT_DELAYED_WORK(&bq->charging_work, bq24725_charging_workfunc);

	ret = bq24725_init_device(bq);
	if (ret < 0) {
		bq_err("Failed in initialize charger");
		return ret;
	}

	if (gpio_is_valid(bq->pdata->status_gpio)) {
		ret = devm_gpio_request(&client->dev,
					bq->pdata->status_gpio,
					"bq24725");
		if (ret) {
			bq_err("Failed GPIO request for GPIO %d: %d\n",
				bq->pdata->status_gpio, ret);
		}

		bq->pdata->status_gpio_valid = !ret;
	}

	bq24725_psy_register(bq);
	
	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						bq24725_charger_interrupt,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						"bq24725 charger irq",
						bq);
		if (ret) {
			bq_err("Unable to register IRQ %d err %d\n",
				client->irq, ret);
			return ret;
		}
	}


	determine_initial_status(bq);

	bq_log("bq24725 probe successfully\n");

	return 0;
}

static const struct i2c_device_id bq24725_charger_id[] = {
	{ "bq24725-charger", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, bq24725_charger_id);

static const struct of_device_id bq24725_match_ids[] = {
	{ .compatible = "ti,bq24725", },
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, bq24725_match_ids);

static struct i2c_driver bq24725_charger_driver = {
	.driver = {
		.name = "bq24725-charger",
		.of_match_table = bq24725_match_ids,
	},
	.probe = bq24725_charger_probe,
	.id_table = bq24725_charger_id,
};

module_i2c_driver(bq24725_charger_driver);

MODULE_DESCRIPTION("bq24725 battery charging driver");
MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL v2");
