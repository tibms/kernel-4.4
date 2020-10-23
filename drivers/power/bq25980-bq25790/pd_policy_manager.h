/*
 * usb_pd_policy_manager.h
 *
 *  Created on: Mar 27, 2017
 *      Author: a0220433
 */

#ifndef SRC_PDLIB_USB_PD_POLICY_MANAGER_H_
#define SRC_PDLIB_USB_PD_POLICY_MANAGER_H_
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/usb/usbpd.h>

enum pm_state {
    PD_PM_STATE_ENTRY,
    PD_PM_STATE_FC3_ENTRY,
    PD_PM_STATE_FC3_ENTRY_1,
    PD_PM_STATE_FC3_ENTRY_2,
    PD_PM_STATE_FC3_ENTRY_3,
    PD_PM_STATE_FC3_TUNE,
    PD_PM_STATE_FC3_EXIT,
};

enum apdo_type {
	PD_PM_APDO_NONE,
	PD_PM_APDO_BYPASS,
	PD_PM_APDO_CP,
};

enum fpdo_type {
	PD_PM_FPDO_NONE,
	PD_PM_FPDO_FIXED1,
	PD_PM_FPDO_FIXED2,
	PD_PM_FPDO_FIXED,
};


#define VBAT_REG_STATUS_SHIFT			0
#define IBAT_REG_STATUS_SHIFT			1

#define VBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)
#define IBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)

#define PDO_MAX_NUM			7

#define MAX_THERMAL_LEVEL			13
/* jeita related */
#define JEITA_WARM_THR			450
#define JEITA_COOL_NOT_ALLOW_CP_THR			100

/*
 * add hysteresis for warm threshold to avoid flash
 * charge and normal charge switch frequently at
 * the warm threshold
 */
#define JEITA_HYSTERESIS			20

struct sw_device {
	bool charge_enabled;
	bool charge_limited;
	bool charging;
};

struct cp_device {
	bool charge_enabled;

	int  health;
	/* alarm/fault status */
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool bus_ovp_fault;
	bool bus_ocp_fault;

	bool bat_ovp_alarm;
	bool bat_ocp_alarm;
	bool bus_ovp_alarm;
	bool bus_ocp_alarm;

	bool bat_therm_fault;

	bool therm_shutdown_flag;
	bool therm_shutdown_stat;

	bool vbat_reg;
	bool ibat_reg;

	int  vbat_volt;
	int  vbus_volt;
	int  ibat_curr;
	int  ibus_curr;
};

struct usbpd_pm {
	enum pm_state state;
	
	struct cp_device cp;
	struct cp_device cp_sec;

	struct sw_device sw;

	bool	cp_sec_stopped;

	bool	pd_active;
	int	adapter_apdo_type;
	int	adapter_fpdo_type;

	int	request_voltage;
	int	request_current;

	struct usbpd *pd;
	struct usbpd_pdo pdo[7];

	int	apdo_max_volt;
	int	apdo_min_volt;
	int	apdo_max_curr;
	int	selected_apdo;
	int	selected_fpdo;

	int	adapter_voltage;
	int	adapter_current;
	int	adapter_ptf;
	bool	adapter_omf;

	struct delayed_work pm_work;

	struct notifier_block nb;

	bool   psy_change_running;
	struct work_struct usb_psy_change_work;
	spinlock_t psy_change_lock;

	struct power_supply *cp_psy;
	struct power_supply *cp_bat_psy;
	struct power_supply *cp_sec_psy;
	struct power_supply *sw_psy;
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	/* jeita or thermal related */
	bool			jeita_triggered;
	bool			is_temp_out_fc3_range;
};

struct pdpm_config {
	int	bat_volt_lp_lmt; /*bat volt loop limit*/
	int	bat_curr_lp_lmt;
	int	bus_curr_lp_lmt;
	int	sw_keep_current;

	int	fc3_taper_current;
	int	fc3_steps;

	int	min_vbat_for_cp;
	int	min_ibat_for_cp;

	bool	cp_sec_enable;
	int		vbus_times;
	int		ibus_regu_hys;
	bool	is_qualified_adapter;


};

#endif /* SRC_PDLIB_USB_PD_POLICY_MANAGER_H_ */
