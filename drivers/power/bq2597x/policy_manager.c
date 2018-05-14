
#define pr_fmt(fmt)	"[FC-PM]: %s: " fmt, __func__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/usb/usbpd.h>

#include "bq2597x.h"

#define	bq_dbg	pr_debug
#define bq_log	pr_err
#define bq_err	pr_err
#define bq_info pr_info

#define PD_SRC_PDO_TYPE(pdo)		(((pdo) >> 30) & 3)
#define PD_SRC_PDO_TYPE_FIXED		0
#define PD_SRC_PDO_TYPE_BATTERY		1
#define PD_SRC_PDO_TYPE_VARIABLE	2
#define PD_SRC_PDO_TYPE_AUGMENTED	3

#define PD_APDO_PPS(pdo)		(((pdo) >> 28) & 3)
#define PD_APDO_MAX_VOLT(pdo)		(((pdo) >> 17) & 0xFF)
#define PD_APDO_MIN_VOLT(pdo)		(((pdo) >> 8) & 0xFF)
#define PD_APDO_MAX_CURR(pdo)		((pdo) & 0x7F)


#define BATT_MAX_CHG_VOLT		4400
#define BATT_FAST_CHG_CURR		6000
#define	BUS_OVP_THRESHOLD		12000
#define	BUS_OVP_ALARM_THRESHOLD		9500

#define BAT_VOLT_LOOP_LMT		BATT_MAX_CHG_VOLT
#define BAT_CURR_LOOP_LMT		BATT_FAST_CHG_CURR
#define BUS_VOLT_LOOP_LMT		BUS_OVP_THRESHOLD

struct adapter_info {
	u32	pdos[7];
	u16	pps_max_volt;
	u16	pps_max_curr;

	u16	volt;	/* adapter output volt */
	u16	curr;	/* adapter output curr */
	u8	ptf;	/* */
	bool	omf;
	bool	pps_supported;	/* qualified PPS supported */
};

enum pm_sm_state {
	PD_PM_STATE_NONE,
	PD_PM_STATE_DISCONNECT,
	PD_PM_STATE_ENTRY,
	PD_PM_STATE_SW_ENTRY,
	PD_PM_STATE_SW_ENTRY_2,
	PD_PM_STATE_SW_LOOP,
	PD_PM_STATE_FC_ENTRY,
	PD_PM_STATE_FC_ENTRY_1,
	PD_PM_STATE_FC_ENTRY_2,
	PD_PM_STATE_FC_ENTRY_3,
	PD_PM_STATE_FC_GET_PPS_STATUS,
	PD_PM_STATE_FC_TUNE,
	PD_PM_STATE_STOP_CHARGE,
};


struct pm_config {
	u16		bat_volt_lp_lmt; /*bat volt loop limit*/
	u16		bat_curr_lp_lmt;
	u16		bus_volt_lp_lmt;
	u16		bus_curr_lp_lmt;
	u16		fc_taper_current;

	int		adapter_volt_required;

	int		down_steps;

	u16		min_vbat_start_fc;
};

static const struct pm_config pm_config = {
	.bat_volt_lp_lmt	= BAT_VOLT_LOOP_LMT,
	.bat_curr_lp_lmt	= BAT_CURR_LOOP_LMT + 1000,
	.bus_volt_lp_lmt	= BUS_VOLT_LOOP_LMT,
	.bus_curr_lp_lmt	= BAT_CURR_LOOP_LMT >> 1,

	.fc_taper_current	= 2000,
	.down_steps		= -1,
	.adapter_volt_required	= 11000;
	.min_vbat_start_fc	= 3500,
};

struct pd_pm {
	bool		pd_connected;
	bool		sw_is_charging;
	bool		fc_is_charging;
	bool		sw_from_fc;
	bool		sw_near_cv;
	bool		sw_fc_init_fail;

	struct hrtimer	timer;

	u16		request_volt;
	u16		request_curr;
	u8		pps_pdo_selected;

	int		ibus_lmt_change_timer;
	int		fc_taper_timer;

	enum pm_state	current_state;

	bool		sw_charge_enabled;
	bool		fc_charge_enabled;

	struct bq2597x_state bq2597x;

	struct power_supply *fc_psy;
	struct power_supply *sw_psy;

	struct delayed_work pm_work;

};

static struct adapter_info adapter;
static struct pd_pm pd_pm;

static void pd_pm_update_fc_status(void)
{

	bq2597x_read_state(&pd_pm.bq2597x);
}


static int pd_pm_enable_fc(bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pd_pm.fc_psy) {
		pd_pm.fc_psy = power_supply_get_by_name("bq2597x");
		if (!pd_pm.fc_psy)
			return -ENODEV;
	}

	val.intval = enable;
	ret = pd_pm.fc_psy->set_property(pd_pm.fc_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);

	return ret;
}

static int pd_pm_enable_sw(bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pd_pm.sw_psy) {
		pd_pm.sw_psy = power_supply_get_by_name("battery");
		if (!pd_pm.sw_psy)
			return -ENODEV;
	}

	val.intval = !enable;
	ret = pd_pm.sw_psy->set_property(pd_pm.sw_psy,
			POWER_SUPPLY_PROP_INPUT_SUSPEND, &val);
	return ret;
}

static int pd_pm_check_fc_enabled(void)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pd_pm.fc_psy) {
		pd_pm.fc_psy = power_supply_get_by_name("bq2597x");
		if (!pd_pm.fc_psy)
			return -ENODEV;
	}

	ret = pd_pm.fc_psy->get_property(pd_pm.fc_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pd_pm.fc_charge_enabled = !!val.intval;

	return ret;
}

static int pd_pm_check_sw_enabled(void)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pd_pm.sw_psy) {
		pd_pm.sw_psy = power_supply_get_by_name("battery");
		if (!pd_pm.sw_psy)
			return -ENODEV;
	}

	ret = pd_pm.sw_psy->get_property(pd_pm.sw_psy,
			POWER_SUPPLY_PROP_INPUT_SUSPEND, &val);
	if (!ret)
		pd_pm.sw_charge_enabled = !val.intval;

	return ret;
}

#define TAPER_TIMEOUT	50
#define IBUS_CHANGE_TIMEOUT  5
static int pd_pm_fc_charge(unsigned int port)
{
	int steps;
	int sw_ctrl_steps = 0;
	int hw_ctrl_steps = 0;
	int step_vbat = 0;

	int step_ibus = 0;
	int step_ibat = 0;
	int step_bat_reg = 0;

	static int ibus_limit;

	if (ibus_limit == 0)
		ibus_limit = pm_config.bus_curr_lp_lmt + 400;

	if (pd_pm.bq2597x.vbat_volt > pm_config.bat_volt_lp_lmt - 50) {
		if (pd_pm.ibus_lmt_change_timer++ > IBUS_CHANGE_TIMEOUT) {
			pd_pm.ibus_lmt_change_timer = 0;
			ibus_limit = pm_config.bus_curr_lp_lmt - 400;
		}
	} else if (pd_pm.bq2597x.vbat_volt < pm_config.bat_volt_lp_lmt - 250) {
		ibus_limit = pm_config.bus_curr_lp_lmt + 400;
		pd_pm.ibus_lmt_change_timer = 0;
	} else {
		pd_pm.ibus_lmt_change_timer = 0;
	}

	/* battery voltage loop*/
	if (pd_pm.bq2597x.vbat_volt > pm_config.bat_volt_lp_lmt)
		step_vbat = pm_config.down_steps;
	else if (pd_pm.bq2597x.vbat_volt < pm_config.bat_volt_lp_lmt - 7)
		step_vbat = 1;

	/* battery current loop */
	if (pd_pm.bq2597x.ibat_curr < pm_config.bat_curr_lp_lmt)
		step_ibat = 1;
	else if (pd_pm.bq2597x.ibat_curr > pm_config.bat_curr_lp_lmt + 100)
		step_ibat = pm_config.down_steps;

	/* cable bus current loop */
	if (pd_pm.bq2597x.ibus_curr < ibus_limit - 50)
		step_ibus = 1;
	else if (pd_pm.bq2597x.ibus_curr > ibus_limit)
		step_ibus = pm_config.down_steps;

	/* battery volt and current regulation loop */
	if (pd_pm.bq2597x.vbat_reg || pd_pm.bq2597x.ibat_reg)
		step_bat_reg = 5 * pm_config.down_steps;
	else
		step_bat_reg = 1;

	sw_ctrl_steps = min(min(step_vbat, step_ibus), step_ibat);
	sw_ctrl_steps = min(sw_ctrl_steps, step_bat_reg);

	if (pd_pm.bq2597x.bat_ocp_alarm
		/*|| pd_pm.bq2597x.bat_ovp_alarm */
		|| pd_pm.bq2597x.bus_ocp_alarm
		|| pd_pm.bq2597x.bus_ovp_alarm
		/* || pd_pm.bq2597x.tbat_temp > 60 */
		/* || pd_pm.bq2597x.tbus_temp > 50 */)
		hw_ctrl_steps = pm_config.down_steps;
	else
		hw_ctrl_steps = 1;

	steps = min(sw_ctrl_steps, hw_ctrl_steps);

	pd_pm_check_fc_enabled();

	if (pd_pm.bq2597x.bat_therm_fault)
	/* battery overheat, stop charge */
		return -1;
	else if (pd_pm.bq2597x.bat_ocp_fault
		|| pd_pm.bq2597x.bus_ocp_fault
		|| pd_pm.bq2597x.bat_ovp_fault
		|| pd_pm.bq2597x.bus_ovp_fault)
		/* go to switch, and try to ramp up if ok */
		return 2;
	else if (!pd_pm.fc_charge_enabled)
		return -2;

	/* check if need switch to switching charger */
	if (pd_pm.bq2597x.vbat_volt > pm_config.bat_volt_lp_lmt - 50 &&
		 pd_pm.bq2597x.ibat_curr < pm_config.fc_taper_current) {
		if (pd_pm.fc_taper_timer++ > TAPER_TIMEOUT) {
			pd_pm.fc_taper_timer = 0;
			return 1;
		}
	} else {
		pd_pm.fc_taper_timer = 0;
	}

#if 0
	bq_dbg("step_vbat=%d, step_ibat=%d, step_ibus = %d, hw_ctrl_steps=%d,
		step_bat_reg = %d, adjust step= %d\n",
		step_vbat, step_ibat, step_ibus, hw_ctrl_steps,
		step_bat_reg, steps);
#endif
	pd_pm.request_volt = pd_pm.request_volt + steps * 20;

	if (pd_pm.request_volt > adapter.pps_max_volt)
		pd_pm.request_volt = adapter.pps_max_volt;

	return 0;
}

const unsigned char *pd_pm_str[] = {
	"PD_PM_STATE_NONE",
	"PD_PM_STATE_DISCONNECT",
	"PD_PM_STATE_ENTRY",
	"PD_PM_STATE_SW_ENTRY",
	"PD_PM_STATE_SW_ENTRY_2",
	"PD_PM_STATE_SW_LOOP",
	"PD_PM_STATE_FC_ENTRY",
	"PD_PM_STATE_FC_ENTRY_1",
	"PD_PM_STATE_FC_ENTRY_2",
	"PD_PM_STATE_FC_ENTRY_3",
	"PD_PM_STATE_FC_GET_PPS_STATUS",
	"PD_PM_STATE_FC_TUNE",
	"PD_PM_STATE_STOP_CHARGE",
};

static void pd_pm_move_state(pm_sm_state_t state)
{
	bq_dbg("%s->%s\n", pd_pm_str[pd_pm.current_state], pd_pm_str[state]);
	pd_pm.current_state = state;
}

static void pd_pm_send_pps_request(void)
{
	pd_pm.request_curr = min(adapter.pps_max_curr,
				pm_config.bus_curr_lp_lmt);
	usbpd_select_pdo(pd_pm.pps_pdo_selected,
				pd_pm.request_volt * 1000,
				pd_pm.request_curr * 1000);

	bq_dbg("requst_volt:%d, requst_curr:%d\n",
			pd_pm.request_volt, pd_pm.request_curr);
}


static void pd_pm_sm(void)
{
	int ret;
	int upper_volt, lower_volt;
	static int tune_vbus_retry;

	pd_pm_update_fc_status();
	if (!pd_pm.bq2597x.data_valid)
		return;

	switch (pd_pm.current_state) {
	case PD_PM_STATE_NONE:
		break;

	case PD_PM_STATE_DISCONNECT:
		if (pd_pm.fc_charge_enabled) {
			pd_pm_enable_fc(false);
			pd_pm_check_fc_enabled();
		}

		if (!pd_pm.sw_charge_enabled) {
			pd_pm_enable_sw(true);
			pd_pm_check_sw_enabled();
		}

		pd_pm.sw_from_fc = false;
		pd_pm.sw_fc_init_fail = false;
		break;

	case PD_PM_STATE_ENTRY:
		bq_dbg("PPS supported:%s, vbat_volt:%d\n",
			adapter.pps_supported ? "true" : "false",
			pd_pm.bq2597x.vbat_volt);
		if (!adapter.pps_supported
			|| pd_pm.bq2597x.vbat_volt < pm_config.min_vbat_start_fc
			|| pd_pm.sw_from_fc
			|| pd_pm.sw_fc_init_fail) {
			bq_log("Start switch charge due to: pps_supported = %s,
				vbat_volt = %d, sw_from_fc = %d\n",
				adapter.pps_supported ? "true" : "false",
				pd_pm.bq2597x.vbat_volt, pd_pm.sw_from_fc);
			pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
		} else if (pd_pm.bq2597x.vbat_volt > pm_config.bat_volt_lp_lmt - 100) {
			bq_log("batt volt-%d too high, start switch charging\n",
				pd_pm.bq2597x.vbat_volt);
			pd_pm.sw_near_cv = true;
			pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
		} else {
			bq_log("battery volt-%d is ok, start flash charging\n",
				pd_pm.bq2597x.vbat_volt);
			pd_pm_move_state(PD_PM_STATE_FC_ENTRY);
		}
		break;

	case PD_PM_STATE_SW_ENTRY:
		/* make sure flash charge is disabled */
		if (pd_pm.fc_charge_enabled) {
			pd_pm_enable_fc(false);
			pd_pm_check_fc_enabled();
		}

		if (!pd_pm.fc_charge_enabled) {
			usbpd_select_pdo(1, 0, 0); /* vSafe5V) */
			pd_pm_move_state(PD_PM_STATE_SW_ENTRY_2);
		}
		break;

	case PD_PM_STATE_SW_ENTRY_2:
		pd_pm_enable_sw(true);
		pd_pm_check_sw_enabled();
		if (pd_pm.sw_charge_enabled)
			pd_pm_move_state(PD_PM_STATE_SW_LOOP);
	       break;

	case PD_PM_STATE_SW_LOOP:
		if (adapter.pps_supported && !pd_pm.sw_from_fc
			&& !pd_pm.sw_near_cv
			&& (pd_pm.bq2597x.vbat_volt > pm_config.min_vbat_start_fc)) {
			bq_log("battery volt: %d ok, start flash charging...\n",
				pd_pm.bq2597x.vbat_volt);
			pd_pm_move_state(PD_PM_STATE_FC_ENTRY);
			break;
		}
		break;

	case PD_PM_STATE_FC_ENTRY:
		/* make sure switch charge is disabled */
		if (pd_pm.sw_charge_enabled) {
			pd_pm_enable_sw(false);
			pd_pm_check_sw_enabled();
		}

		if (!pd_pm.sw_charge_enabled) {
			pd_pm_move_state(PD_PM_STATE_FC_ENTRY_2);
			tune_vbus_retry = 0;
		}
		break;

	case PD_PM_STATE_FC_ENTRY_2:
		/* tune adapter voltage to range:2xVBat + 200 +/- 50 */
		upper_volt = (pd_pm.bq2597x.vbat_volt * 2 + 250);
		lower_volt = (pd_pm.bq2597x.vbat_volt * 2 + 150);
		if (pd_pm.bq2597x.vbus_volt < lower_volt) {
			pd_pm.request_volt += 20;
			pd_pm_send_pps_request();
		} else if (pd_pm.bq2597x.vbus_volt > upper_volt) {
			pd_pm.request_volt -= 20;
			pd_pm_send_pps_request();
		} else {
			bq_log("Initial volt tune ok, retry %d times\n",
				tune_vbus_retry);
			pd_pm_move_state(PD_PM_STATE_FC_ENTRY_3);
			break;
		}
		if (tune_vbus_retry++ > 50) {
			bq_err("Initial volt tune failed,
				charge with switching charger\n");
			pd_pm.sw_fc_init_fail = true;
			pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
		}
		break;

	case PD_PM_STATE_FC_ENTRY_3:
		/* now it's ok to turn on flash charge */
		pd_pm_enable_fc(true);
		pd_pm_check_fc_enabled();
		if (pd_pm.fc_charge_enabled)
			pd_pm_move_state(PD_PM_STATE_FC_TUNE);
		pd_pm.ibus_lmt_change_timer = 0;
		pd_pm.fc_taper_timer = 0;
		break;

	case PD_PM_STATE_FC_TUNE:
		if (pd_pm.bq2597x.vbat_volt < pm_config.min_vbat_start_fc - 400) {
			/* volt drops too much when sw charge is off */
			pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
			break;
		}

		ret = pd_pm_fc_charge();
		if (ret == -1) {
			bq_log("Move to stop charging:%d\n", ret);
			pd_pm_move_state(PD_PM_STATE_STOP_CHARGE);
			break;
		} else if (ret == -2 || ret == 1) {
			bq_log("Move to switch charging:%d\n", ret);
			pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
			pd_pm.sw_from_fc = true;
			break;
		} else if (ret == 2) {
			bq_log("Move to switch charging, try to
				recover to flash charging:%d\n", ret);
			pd_pm_move_state(PD_PM_STATE_SW_ENTRY);
		} else {
			/* everything is ok, send new request*/
			pd_pm_send_pps_requst();
		}

		break;

	case PD_PM_STATE_STOP_CHARGE:
		if (pd_pm.fc_charge_enabled) {
			pd_pm_enable_fc(false);
			pd_pm_check_fc_enabled();
		}
		if (pd_pm.sw_charge_enabled) {
			pd_pm_enable_sw(false);
			pd_pm_check_sw_enabled();
		}
		break;
	}

}

static void pd_connected_callback(void)
{
	if (pd_pm.pd_connected)
		return;

	pd_pm.pd_connected = true;
	pd_pm_move_state(PD_PM_STATE_ENTRY);
}


static void pd_disconnected_callback(void)
{
	pd_pm.pd_connected = false;
	pd_pm_move_state(PD_PM_STATE_DISCONNECT);
}

static void pd_eval_src_caps_callback(u32 *pdos, u8 len)
{
	int max_volt, max_curr;
	u8 i;

	adapter.pps_max_volt = 0;
	adapter.pps_max_curr = 0;
	adapter.pps_supported = false;

	for (i = 0; i < len; i++) {
		adapter.pdos[i] = pdos[i];
		if (PD_SRC_PDO_TYPE(pdos[i]) != PD_SRC_PDO_TYPE_AUGMENETED
			|| PD_APDO_PPS(pdos[i]) != 0)
			continue;

		max_volt = PD_APDO_MAX_VOLT(pdos[i]) * 100;
		max_curr = PD_APDO_MAX_CURR(pdos[i]) * 50;

		/*
		 * prefer PDO with higher current capability
		 * if multiple PPS pdo exists
		 */
		if (max_volt >= pm_config.adapter_volt_required
			&& max_curr > adapter.pps_max_curr) {
			adapter.pps_max_volt = max_volt;
			adapter.pps_max_curr = max_curr;
			adapter.pps_supported = true;
			pd_pm.pps_pdo_selected = i + 1;
		}

		bq_dbg("PPS PDO found: %u, max_volt:%u, max_curr:%u",
				i + 1, max_volt, max_curr);
	}
}


static const struct pd_pm_cb pd_pm_cb = {
	.connected_cb = pd_connected_callback;
	.disconnected_cb = pd_disconnected_callback;
	.pps_status_cb = NULL;
	.eval_src_caps_cb = pd_eval_src_caps_callback;
};


static void pd_pm_workfunc(struct work_struct *work)
{

	pd_pm_sm();

	schedule_delayed_work(&pd_pm.pm_work, msecs_to_jiffies(100));
}


static int __init usb_pd_pm_init(void)
{

	usbpd_register_pm_cb(&pd_pm_cb);

	INIT_DELAYED_WORK(&pd_pm.pm_work, pd_pm_workfunc);

	schedule_delayed_work(&pd_pm.pm_work, msecs_to_jiffies(100));

	return 0;
}

static void __exit usb_pd_pm_exit(void)
{
	usbpd_unregister_pm_cb();
	cancel_delayed_work(&pd_pm.pm_work);
}

module_init(usb_pd_pm_init);
module_exit(usb_pd_pm_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("USB Power Delivery Policy Manager");
MODULE_LICENSE("GPL v2");
