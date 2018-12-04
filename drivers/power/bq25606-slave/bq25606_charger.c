/*
 * BQ2589x battery charging driver
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
#define pr_fmt(fmt) "BQ25606 %s: " fmt, __func__

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
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>


#ifdef pr_debug
#undef pr_debug
#define pr_debug pr_err
#endif


enum bq25606_part_no {
	BQ25606 = 0x01,
};


struct bq25606 {
	struct	device	*dev;
	struct	i2c_client *client;

	bool suspended;

	int ichg_ma;
	int vchg_mv;
	int icl_ma;
	int ivl_mv;
	int max_fcc;

	int c_health;
	
	int gpio_ce;	/*charge enable gpio*/

	struct power_supply *usb_psy;
	struct power_supply *batt_psy;
	struct power_supply *parallel_psy;
	struct power_supply_desc parallel_psy_d;

};


static bool bq25606_is_usb_present(struct bq25606 *bq)
{
	int rc;
	union power_supply_propval val = {0, };

	if (!bq->usb_psy)
		bq->usb_psy = power_supply_get_by_name("usb");
	if (!bq->usb_psy) {
		pr_err("USB psy not found\n");
		return false;
	}

	rc = power_supply_get_property(bq->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &val);

	if (rc < 0) {
		pr_err("Failed to get present property rc=%d\n", rc);
		return false;
	}

	if (val.intval)
		return true;

	return false;

}

static int bq25606_usb_suspend(struct bq25606 *bq, bool suspend)
{
	int rc = 0;
	
	rc = gpio_direction_ouput(bq->gpio_ce, suspend);

	
	if (rc) {
		pr_err("Couldn't %s rc = %d\n",
			suspend ? "suspend" : "resume",  rc);
		return rc;
	}
	
	bq->suspended = suspend;

	return rc;
}


#define	DP_DM_MAX_PULSE_COUNT	8
static bool bq25606_is_input_current_limited(struct bq25606 *bq)
{
	int rc;
	union power_supply_propval val = {0, };

	if (!bq->batt_psy)
		bq->batt_psy = power_supply_get_by_name("battery");
	if (!bq->batt_psy) {
		pr_err("battery psy not found\n");
		return false;
	}

	rc = power_supply_get_property(bq->batt_psy,
				POWER_SUPPLY_PROP_DP_DM, &val);

	if (!rc && !bq->suspended && val->intval > DP_DM_MAX_PULSE_COUNT)
		return true;
	else
		return false;
}

static enum power_supply_property bq25606_charger_properties[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_PIN_ENABLED,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_CHARGER_TEMP,
	POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_PARALLEL_MODE,
	POWER_SUPPLY_PROP_CONNECTOR_HEALTH,
	POWER_SUPPLY_PROP_PARALLEL_BATFET_MODE,
	POWER_SUPPLY_PROP_PARALLEL_FCC_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED, 
	POWER_SUPPLY_PROP_MIN_ICL,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
	POWER_SUPPLY_PROP_DIE_HEALTH,
};

static int bq25606_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct bq25606 *bq = power_supply_get_drvdata(psy);


	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		pr_debug("POWER_SUPPLY_PROP_INPUT_SUSPEND:%d\n",
						val->intval);
		bq25606_usb_suspend(bq, val->intval);

		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		pr_debug("POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:%d\n",
						val->intval);		
	
		bq->ichg_ma = val->intval / 1000;
		break;
		
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		pr_debug("POWER_SUPPLY_PROP_CURRENT_MAX:%d\n",
						val->intval);		
		bq->icl_ma = val->intval / 1000;
		
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pr_debug("POWER_SUPPLY_PROP_VOLTAGE_MAX:%d\n",
						val->intval);		
		bq->vchg_mv = val->intval / 1000;
		break;
	case POWER_SUPPLY_PROP_CONNECOTR_HEALTH:
		bq->c_health = val->intval;		
		power_supply_changed(bq->parallel_psy);
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		rc = 0;
		break;

	default:
		pr_err("unsupported prop:%d", prop);
		return -EINVAL;
	}


	return rc;


}

static int bq25606_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CONNECTOR_HEALTH:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

#define MIN_PARALLEL_ICL_UA 250000 
static int bq25606_charger_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct bq25606 *bq = power_supply_get_drvdata(psy);
	u8 temp;
	int itemp = 0;
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:

		val->intval = 1;
		pr_debug("POWER_SUPPLY_PROP_ONLINE:%d\n", 
				val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		/* assume it is always enabled, using SUSPEND to control charging */
		val->intval = 1;
		pr_debug("POWER_SUPPLY_PROP_CHARGING_ENABLED:%d\n", 
				val->intval);
		break;

	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		val->intval = bq->suspended;

		pr_debug("POWER_SUPPLY_PROP_INPUT_SUSPEND:%d\n", val->intval);

		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = bq->icl_ma * 1000;	
		pr_debug("POWER_SUPPLY_PROP_CURRENT_MAX:%d\n", 
			val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = bq->vchg_mv * 1000;
		pr_debug("POWER_SUPPLY_PROP_VOLTAGE_MAX:%d\n", 
			val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;

		if (bq25606_is_usb_present(bq))
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;

		pr_debug("POWER_SUPPLY_PROP_CHARGE_TYPE:%d\n", 
			val->intval);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = bq->ichg_ma * 1000;
		pr_debug("POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:%d\n", 
			val->intval);
		break;
		
	case POWER_SUPPLY_PROP_STATUS:
		if (bq->icl_ma > 0 && !bq->suspended)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

		pr_debug("POWER_SUPPLY_PROP_STATUS:%d\n", 
			val->intval);

		break;
		
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		val->intval = bq25606_is_input_current_limited(bq) ? 1 : 0;

		pr_debug("POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:%d\n", 
			val->intval);

		break;
		 
	case POWER_SUPPLY_PROP_PARALLEL_MODE:
		val->intval = POWER_SUPPLY_PL_USBIN_USBIN;
		/*val->intval = POWER_SUPPLY_PL_USBMID_USBMID;*/
		pr_debug("POWER_SUPPLY_PROP_PARALLEL_MODE:%d\n", 
			val->intval);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "smbq25606";
		break;

	case POWER_SUPPLY_PROP_PARALLEL_BATFET_MODE:
		/*val->intval = POWER_SUPPLY_PL_NON_STACKED_BATFET;*/
		val->intval = POWER_SUPPLY_PL_STACKED_BATFET;
		break;
	case POWER_SUPPLY_PROP_PIN_ENABLED:
		val->intval = 0;
		break;
#if 0
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = 0;/*TODO?*/
		break;
#endif

	case POWER_SUPPLY_PROP_CONNECTOR_HEALTH:
		//val->intval = bq->c_health;
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_CHARGER_TEMP:
		val->intval = 20000;
		break;
	case POWER_SUPPLY_PROP_CHARGER_TEMP_MAX:
		val->intval = 80000;
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_MIN_ICL:
		val->intval = MIN_PARALLEL_ICL_UA;
		break;

	case POWER_SUPPLY_PROP_PARALLEL_FCC_MAX:
		val->intval = bq->max_fcc;
		break;
	default:
		pr_err("unsupported prop:%d\n", prop);
		return -EINVAL;
	}
	return 0;

}

static int bq25606_parse_dt(struct device *dev, struct bq25606 *bq)
{
	int ret;
	struct device_node *np = dev->of_node;


	ret = of_property_read_u32(np, "ti,bq25606,gpio-chip-enable",
					&bq->gpio_ce);

	return ret;
}

static int bq25606_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq25606 *bq = NULL;
	int ret;
	struct power_supply_config parallel_psy_cfg = {};

	bq = devm_kzalloc(&client->dev, sizeof(struct bq25606), GFP_KERNEL);
	if (!bq) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;

	i2c_set_clientdata(client, bq);

	if (client->dev.of_node)
		bq25606_parse_dt(&client->dev, bq);

	bq->max_fcc = INT_MAX;
	bq->c_health = -EINVAL;


	if (gpio_is_valid(bq->gpio_ce)) {
		ret = devm_gpio_request(&client->dev, bq->gpio_ce, "bq25606_ce");
		if (ret) {
			pr_err("Failed to request chip enable gpio %d:, err: %d\n", bq->gpio_ce, ret);
			return ret;
		}
		/*disable CE by default*/
		gpio_direction_output(bq->gpio_ce, 1);
	} else {
		pr_err("charge enable gpio not specified\n");

		return -EINVAL;
	}
		
	bq->parallel_psy_d.name	= "parallel";
	bq->parallel_psy_d.type	= POWER_SUPPLY_TYPE_PARALLEL;
	bq->parallel_psy_d.get_property = bq25606_charger_get_property;
	bq->parallel_psy_d.set_property = bq25606_charger_set_property;
	bq->parallel_psy_d.properties   = bq25606_charger_properties;
	bq->parallel_psy_d.property_is_writeable = bq25606_charger_is_writeable;
	bq->parallel_psy_d.num_properties = ARRAY_SIZE(bq25606_charger_properties);

	parallel_psy_cfg.drv_data = bq;
	parallel_psy_cfg.num_supplicants = 0;
	bq->parallel_psy = devm_power_supply_register(bq->dev,
			&bq->parallel_psy_d,
			&parallel_psy_cfg);
	if (IS_ERR(bq->parallel_psy)) {
		pr_err("Couldn't register parallel psy rc=%ld\n",
				PTR_ERR(bq->parallel_psy));
		ret = PTR_ERR(bq->parallel_psy);
		return ret;
	}

	pr_info("BQ25606 PARALLEL charger driver probe successfully\n");

	return 0;

err_0:
	power_supply_unregister(bq->parallel_psy);
	return ret;
}

static int bq25606_charger_remove(struct i2c_client *client)
{
	struct bq25606 *bq = i2c_get_clientdata(client);

	power_supply_unregister(bq->parallel_psy);

	return 0;
}


static void bq25606_charger_shutdown(struct i2c_client *client)
{
	pr_info("shutdown\n");

}

static struct of_device_id bq25606_charger_match_table[] = {
	{.compatible = "ti,smbq25606"},
	{},
};


static const struct i2c_device_id bq25606_charger_id[] = {
	{ "bq25606", BQ25606 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq25606_charger_id);

static struct i2c_driver bq25606_charger_driver = {
	.driver		= {
		.name	= "smbq25606",
		.of_match_table = bq25606_charger_match_table,
	},
	.id_table	= bq25606_charger_id,

	.probe		= bq25606_charger_probe,
	.remove		= bq25606_charger_remove,
	.shutdown   = bq25606_charger_shutdown,
};

module_i2c_driver(bq25606_charger_driver);

MODULE_DESCRIPTION("TI BQ25606 Charger Driver");
MODULE_LICENSE("GPL2");
