#define pr_fmt(fmt)	"[USBPD-PM]: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/usb/usbpd.h>
#include "pd_policy_manager.h"
#include "bq25980_charger.h"

/***** CUSTOMIZE FUNCTIONS *********/

//#define USE_HW_REGULATION

#define BATTERY_2S
//#define BATTERY_1S
#define IBAT_READABLE

//for regular switch charger
/* disable switching charger during flash charge*/
#define DISABLE_SWITCH_CHARGER

//for qualcomm switch charger
//#define LIMIT_SWITCH_CHARGER
#define SWITCH_CHARGER_NAME "bq25790-charger"


/***** CUSTOMIZE FUNCTIONS end *****/

/***** CUSTOMIZE BATTERY *********/

#ifdef BATTERY_1S

#define BAT_VOLT_MIN_LMT		3500
#define BAT_VOLT_LOOP_LMT		4480
#define STEP1_VOLTAGE	4250
#define STEP2_VOLTAGE	4450
#define STEP3_VOLTAGE	BAT_VOLT_LOOP_LMT
#define ADAPTER_CC_TO_CV_VOLTAGE STEP1_VOLTAGE
#define VBAT_REGU_HYS 7
#define ADAPTER_VOLT_UPPER 12000
#define ADAPTER_VOLT_LOWER 7000
#define ADAPTER_VOLT_UPPER_BYP 0
#define ADAPTER_VOLT_LOWER_BYP 0
#define ADAPTER_VOLT_FIXED 5000
#define ADAPTER_VOLT_FIXED1 0
#define ADAPTER_VOLT_FIXED2 0
#define VBAT_GAP_FOR_CP 100

#elif defined BATTERY_2S
	
#define BAT_VOLT_MIN_LMT		7000
#define BAT_VOLT_LOOP_LMT		8900
#define STEP1_VOLTAGE	8500
#define STEP2_VOLTAGE	8840
#define STEP3_VOLTAGE	BAT_VOLT_LOOP_LMT
#define ADAPTER_CC_TO_CV_VOLTAGE STEP1_VOLTAGE
#define VBAT_REGU_HYS 11
#define ADAPTER_VOLT_UPPER 20000
#define ADAPTER_VOLT_LOWER 12000
#define ADAPTER_VOLT_UPPER_BYP 10000
#define ADAPTER_VOLT_LOWER_BYP 6000
#define ADAPTER_VOLT_FIXED 9000
#define ADAPTER_VOLT_FIXED1 5000
#define ADAPTER_VOLT_FIXED2 12000
#define VBAT_GAP_FOR_CP 300

#endif

#define BAT_CURR_LOOP_LMT		6000
#define BAT_CURR_MIN_LMT		2000
#define STEP1_CURRENT	BAT_CURR_LOOP_LMT
#define STEP2_CURRENT	5000
#define STEP3_CURRENT	4000
#define ADAPTER_CURRENT_TOLE 300
#define IBAT_REGU_HYS 100
#define IBUS_REGU_HYS 50
#define IBUS_REGU_HYS_BYP 100

/***** CUSTOMIZE BATTERY end *****/

#define ADAPTER_CURR_MIN 2000

#define BUS_VOLT_INIT_UP		300	

#define PD_SRC_PDO_TYPE_FIXED		0
#define PD_SRC_PDO_TYPE_BATTERY		1
#define PD_SRC_PDO_TYPE_VARIABLE	2
#define PD_SRC_PDO_TYPE_AUGMENTED	3

#define PM_WORK_RUN_INTERVAL		100
#define TAPER_TIMEOUT	(5000 / PM_WORK_RUN_INTERVAL)
#define STEP_TIMEOUT	(500 / PM_WORK_RUN_INTERVAL)

static int g_fcc_val = 0;
static int g_fcv_val = 0;
static bool g_fix_pdo_requested = false;

enum {
	PM_ALGO_RET_OK,
	PM_ALGO_RET_THERM_FAULT,
	PM_ALGO_RET_OTHER_FAULT,
	PM_ALGO_RET_CHG_DISABLED,
	PM_ALGO_RET_TAPER_DONE,
};

static struct pdpm_config pm_config = {
	.bat_volt_lp_lmt		= BAT_VOLT_LOOP_LMT,
	.bat_curr_lp_lmt		= BAT_CURR_LOOP_LMT,
	.bus_curr_lp_lmt		= BAT_CURR_LOOP_LMT/2,
	.sw_keep_current	= 0,

	.fc3_taper_current		= BAT_CURR_MIN_LMT,
	.fc3_steps			= 1,

	.min_vbat_for_cp		= BAT_VOLT_MIN_LMT,
	.min_ibat_for_cp		= BAT_CURR_MIN_LMT,
};

static struct usbpd_pm *__pdpm;

static const unsigned char *pm_str[] = {
	"PD_PM_STATE_ENTRY",
	"PD_PM_STATE_FC3_ENTRY",
	"PD_PM_STATE_FC3_ENTRY_1",
	"PD_PM_STATE_FC3_ENTRY_2",
	"PD_PM_STATE_FC3_ENTRY_3",
	"PD_PM_STATE_FC3_TUNE",
	"PD_PM_STATE_FC3_EXIT",
};

static void usbpd_check_usb_psy(struct usbpd_pm *pdpm)
{
	if (!pdpm->usb_psy) { 
		pdpm->usb_psy = power_supply_get_by_name("usb");
		if (!pdpm->usb_psy)
			pr_err("usb psy not found!\n");
	}
}

static void usbpd_check_sw_psy(struct usbpd_pm *pdpm)
{
	if (!pdpm->sw_psy) {
		pdpm->sw_psy = power_supply_get_by_name(SWITCH_CHARGER_NAME);
		if (!pdpm->sw_psy)
			pr_err("batt psy not found!\n");
	}
}

static void usbpd_check_bms_psy(struct usbpd_pm *pdpm)
{
	if (!pdpm->bms_psy) {
		pdpm->bms_psy = power_supply_get_by_name("bms");
		if (!pdpm->bms_psy)
			pr_err("bms psy not found!\n");
	}
}

/* get thermal level from battery power supply property */
static int pd_get_batt_current_thermal_level(struct usbpd_pm *pdpm, int *level)
{
	int rc = 0;
	union power_supply_propval pval = {0,};

	usbpd_check_sw_psy(pdpm);

	if (!pdpm->sw_psy)
		return -ENODEV;

	//Get thermal level of your system now.
	//rc = power_supply_get_property(pdpm->sw_psy,
	//			POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &pval);
	if (rc < 0) {
		pr_err("Couldn't get thermal level. ret:%d\n", rc);
		return rc;
	}

	pr_debug("pval.intval: %d\n", pval.intval);

	*level = pval.intval;
	return rc;
}

/* determine whether to disable cp according to jeita status */
static bool pd_disable_cp_by_jeita_status(struct usbpd_pm *pdpm)
{
	//union power_supply_propval pval = {0,};
	int batt_temp = 0;
	int rc;

	usbpd_check_bms_psy(pdpm);
	if (!pdpm->bms_psy)
		return false;

	//rc = power_supply_get_property(pdpm->bms_psy,
	//			POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		pr_notice("Couldn't get batt temp prop:%d\n", rc);
		return false;
	}

	batt_temp = 200;//pval.intval;
	pr_debug("batt_temp: %d\n", batt_temp);

	if (batt_temp >= JEITA_WARM_THR && !pdpm->jeita_triggered) {
		pr_err("jeita upper limit reached, batt_temp:%d\n", batt_temp);
		pdpm->jeita_triggered = true;
		return true;
	} else if (batt_temp <= JEITA_COOL_NOT_ALLOW_CP_THR && !pdpm->jeita_triggered) {
		pr_err("jeita lower limit reached, batt_temp:%d\n", batt_temp);
		pdpm->jeita_triggered = true;
		return true;
	} else if ((batt_temp <= (JEITA_WARM_THR - JEITA_HYSTERESIS))
			&& (batt_temp >= (JEITA_COOL_NOT_ALLOW_CP_THR + JEITA_HYSTERESIS))
			&& pdpm->jeita_triggered) {
		pr_err("jeita returned to normal, batt_temp:%d\n", batt_temp);
		pdpm->jeita_triggered = false;
		return false;
	} else {
		return pdpm->jeita_triggered;
	}

}

static bool is_cool_charge(struct usbpd_pm *pdpm)
{
	union power_supply_propval pval = {0,};
	int batt_temp = 0;
	int rc;

	usbpd_check_bms_psy(pdpm);
	if (!pdpm->bms_psy)
		return false;

	rc = power_supply_get_property(pdpm->bms_psy,
				POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		pr_notice("Couldn't get batt temp prop:%d\n", rc);
		return false;
	}
	batt_temp = pval.intval;

	pr_debug("batt_temp: %d\n", batt_temp);
	if (batt_temp < 150)
		return true;
	return false;
}

static void usbpd_check_cp_psy(struct usbpd_pm *pdpm)
{

	if (!pdpm->cp_psy) {
		if (pm_config.cp_sec_enable)
			pdpm->cp_psy = power_supply_get_by_name("bq2597x-master");
		else
			pdpm->cp_psy = power_supply_get_by_name("bq25980-charger");
		if (!pdpm->cp_psy)
			pr_err("cp_psy not found\n");
	}

	if (!pdpm->cp_bat_psy) {
		pdpm->cp_bat_psy = power_supply_get_by_name("bq25980-battery");
		if (!pdpm->cp_bat_psy)
			pr_err("cp_bat_psy not found\n");
	}
}

static void usbpd_check_cp_sec_psy(struct usbpd_pm *pdpm)
{
	if (!pdpm->cp_sec_psy) {
		pdpm->cp_sec_psy = power_supply_get_by_name("bq2597x-slave");
		if (!pdpm->cp_sec_psy)
			pr_err("cp_sec_psy not found\n");
	}
}

static void usbpd_init_fc_val(struct usbpd_pm *pdpm)
{
	if (pdpm->cp.vbat_volt <= (STEP1_VOLTAGE - VBAT_REGU_HYS)) {
			g_fcc_val = STEP1_CURRENT;
			g_fcv_val = STEP1_VOLTAGE;
	}
	else if (pdpm->cp.vbat_volt <= (STEP2_VOLTAGE - VBAT_REGU_HYS)) {
			g_fcc_val = STEP2_CURRENT;
			g_fcv_val = STEP2_VOLTAGE;
	}
	else if (pdpm->cp.vbat_volt <= (STEP3_VOLTAGE - VBAT_REGU_HYS)) {
			g_fcc_val = STEP3_CURRENT;
			g_fcv_val = STEP3_VOLTAGE;
	}

	pr_notice("vbat_volt %d, g_fcc_val %d, g_fcv_val %d", pdpm->cp.vbat_volt, g_fcc_val, g_fcv_val);
	return;
}


static void usbpd_get_effective_fc_val(struct usbpd_pm *pdpm)
{
	static int s_fc_timer = 0;

	if ((pdpm->cp.vbat_volt >= (STEP1_VOLTAGE - VBAT_REGU_HYS))
			&& (pdpm->cp.vbat_volt < (STEP2_VOLTAGE - VBAT_REGU_HYS))) {
		if ((pdpm->cp.ibat_curr <= STEP2_CURRENT) && (g_fcc_val > STEP2_CURRENT)) {
			if (s_fc_timer++ >= STEP_TIMEOUT) {
				g_fcc_val = STEP2_CURRENT;
				g_fcv_val = STEP2_VOLTAGE;
				s_fc_timer = 0;
			}
		}
		else
			s_fc_timer = 0;
	}
	else if ((pdpm->cp.vbat_volt >= (STEP2_VOLTAGE - VBAT_REGU_HYS))
			&& (pdpm->cp.vbat_volt < (STEP3_VOLTAGE - VBAT_REGU_HYS))) {
		if ((pdpm->cp.ibat_curr <= STEP3_CURRENT) && (g_fcc_val > STEP3_CURRENT)) {
			if (s_fc_timer++ >= STEP_TIMEOUT) {
				g_fcc_val = STEP3_CURRENT;
				g_fcv_val = STEP3_VOLTAGE;
				s_fc_timer = 0;
			}
		}
		else
			s_fc_timer = 0;
	}
	else
		s_fc_timer = 0;

	pr_notice("vbat_volt %d, ibat_curr %d, g_fcc_val %d, g_fcv_val %d",
			pdpm->cp.vbat_volt, pdpm->cp.ibat_curr, g_fcc_val, g_fcv_val);
	return;
}

static void usbpd_pm_update_cp_status(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);

	if (!pdpm->cp_psy || !pdpm->cp_bat_psy)
		return;

	ret = power_supply_get_property(pdpm->cp_bat_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		pdpm->cp.vbat_volt = val.intval/1000;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		pdpm->cp.vbus_volt = val.intval/1000;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		pdpm->cp.ibus_curr = val.intval/1000;

#ifdef IBAT_READABLE
		ret = power_supply_get_property(pdpm->cp_bat_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &val);
		if (!ret)
			pdpm->cp.ibat_curr = val.intval/1000;
#else
		if (pdpm->adapter_apdo_type == PD_PM_PPS_BYPASS)
			pdpm->cp.ibat_curr = pdpm->cp.ibus_curr;
		else
			pdpm->cp.ibat_curr = pdpm->cp.ibus_curr<<1;
#endif

	ret = power_supply_get_property(pdpm->cp_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->cp.charge_enabled = !!val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_HEALTH, &val);
	if (!ret) {
		pdpm->cp.health = val.intval; 
	}

#ifdef USE_HW_REGULATION
	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_REG_STATUS, &val);
	if (!ret) {
		pdpm->cp.vbat_reg = !!(val.intval & VBAT_REG_STATUS_MASK);
		pdpm->cp.ibat_reg = !!(val.intval & IBAT_REG_STATUS_MASK);
	}
#endif
}

static void usbpd_pm_update_cp_sec_status(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_config.cp_sec_enable)
		return;

	usbpd_check_cp_sec_psy(pdpm);
	
	if (!pdpm->cp_sec_psy)
		return;

	ret = power_supply_get_property(pdpm->cp_sec_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		pdpm->cp_sec.ibus_curr = val.intval; 

	ret = power_supply_get_property(pdpm->cp_sec_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
	if (!ret)
		pdpm->cp_sec.charge_enabled = (val.intval==POWER_SUPPLY_STATUS_CHARGING)?true:false;
}

static int usbpd_pm_enable_cp_adc(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);

	if (!pdpm->cp_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(pdpm->cp_psy, 
			POWER_SUPPLY_PROP_TI_ADC, &val);
	
	return ret;
}

static int usbpd_pm_enable_cp_bypass(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);

	if (!pdpm->cp_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(pdpm->cp_psy, 
			POWER_SUPPLY_PROP_TI_BYPASS, &val);
	
	return ret;
}

static int usbpd_pm_enable_cp(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);

	if (!pdpm->cp_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(pdpm->cp_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	
	return ret;
}

static int usbpd_pm_enable_cp_sec(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_sec_psy(pdpm);
	
	if (!pdpm->cp_sec_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(pdpm->cp_sec_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	
	return ret;
}

static int usbpd_pm_check_cp_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);
	
	if (!pdpm->cp_psy)
		return -ENODEV;

	ret = power_supply_get_property(pdpm->cp_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->cp.charge_enabled = !!val.intval;

	pr_debug("pdpm->cp.charge_enabled:%d\n", pdpm->cp.charge_enabled);

	return ret;
}

static int usbpd_pm_check_cp_sec_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_sec_psy(pdpm);

	if (!pdpm->cp_sec_psy) 
		return -ENODEV;

	ret = power_supply_get_property(pdpm->cp_sec_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->cp_sec.charge_enabled = !!val.intval;
	
	return ret;
}
#ifdef DISABLE_SWITCH_CHARGER
static int usbpd_pm_enable_sw(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_sw_psy(pdpm);
	if (!pdpm->sw_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(pdpm->sw_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);

	return ret;
}

static int usbpd_pm_check_sw_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_sw_psy(pdpm);
	if (!pdpm->sw_psy)
		return -ENODEV;

	ret = power_supply_get_property(pdpm->sw_psy, 
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->sw.charge_enabled = !!val.intval;

	return ret;
}
#endif

#ifdef LIMIT_SWITCH_CHARGER
static int usbpd_pm_limit_sw(struct usbpd_pm *pdpm, int current_mA)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_sw_psy(pdpm);
	if (!pdpm->sw_psy)
		return -ENODEV;

	val.intval = current_mA*1000;
	ret = power_supply_set_property(pdpm->sw_psy, 
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &val);

	return ret;
}

static int usbpd_pm_check_sw_limited(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_sw_psy(pdpm);
	if (!pdpm->sw_psy)
		return -ENODEV;

	ret = power_supply_get_property(pdpm->sw_psy, 
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &val);
	if (!ret)
	{
		if (val.intval == 3000000)
			pdpm->sw.charge_limited = false;
		else
			pdpm->sw.charge_limited = true;
	}

	return ret;
}
#endif

int usbpd_pm_check_sw_charging(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_sw_psy(pdpm);
	if (!pdpm->sw_psy)
		return -ENODEV;

	ret = power_supply_get_property(pdpm->sw_psy, 
			POWER_SUPPLY_PROP_CHARGE_TYPE, &val);
	if (!ret)
		pdpm->sw.charging = (val.intval == POWER_SUPPLY_CHARGE_TYPE_FAST) ? true : false;

	return ret;
}

static int usbpd_request_voltage(struct usbpd_pm *pdpm, int pdo, int mv, int ma)
{
	pr_notice("pdo_pos:%d, request_voltage:%d, request_current:%d, ret:%d\n", pdo, mv, ma);
	return usbpd_select_pdo(pdpm->pd, pdo, mv * 1000, ma * 1000);
}

static void usbpd_pm_evaluate_src_caps(struct usbpd_pm *pdpm)
{
	int ret;
	int i;
	int apdo_type = 0;
	int fpdo_type = 0;

	if (!pdpm->pd) {
		pdpm->pd = usbpd_get_g_pd();
		if (!pdpm->pd) {
			pr_err("couldn't get usbpd device\n");
			return;
		}
	}

	ret = usbpd_fetch_pdo(pdpm->pd, pdpm->pdo);
	if (ret) {
		pr_err("Failed to fetch pdo info\n");
		return;
	}
	
	pdpm->adapter_apdo_type = PD_PM_APDO_NONE;
	pdpm->adapter_fpdo_type = PD_PM_FPDO_NONE;
	pdpm->selected_apdo = 0;
	pdpm->selected_fpdo = 0;

	for (i = 0; i < PDO_MAX_NUM; i++) {
		pr_notice("src PDOs: pos:%d pd_type:%d pps:%d max volt:%d min volt:%d current:%d",
				pdpm->pdo[i].pos,
				pdpm->pdo[i].type,
				pdpm->pdo[i].pps,
				pdpm->pdo[i].max_volt_mv,
				pdpm->pdo[i].min_volt_mv,
				pdpm->pdo[i].curr_ma);
		if (pdpm->pdo[i].type == PD_SRC_PDO_TYPE_AUGMENTED
			&& pdpm->pdo[i].pps && pdpm->pdo[i].pos) {
			if (pdpm->pdo[i].max_volt_mv >= ADAPTER_VOLT_UPPER
					&& pdpm->pdo[i].min_volt_mv <= ADAPTER_VOLT_LOWER
					&& pdpm->pdo[i].curr_ma >= ADAPTER_CURR_MIN) {
				apdo_type = PD_PM_APDO_CP;
			} else if (pdpm->pdo[i].max_volt_mv >= ADAPTER_VOLT_UPPER_BYP
					&& pdpm->pdo[i].min_volt_mv <= ADAPTER_VOLT_LOWER_BYP
					&& pdpm->pdo[i].curr_ma >= ADAPTER_CURR_MIN) {
				apdo_type = PD_PM_APDO_BYPASS;
			}
			if (apdo_type > pdpm->adapter_apdo_type) {
				pdpm->adapter_apdo_type = apdo_type;
				if (apdo_type == PD_PM_APDO_CP) {
					pdpm->apdo_max_volt = ADAPTER_VOLT_UPPER;
					pdpm->apdo_min_volt = ADAPTER_VOLT_LOWER;
				} else if (apdo_type == PD_PM_APDO_BYPASS){
					pdpm->apdo_max_volt = ADAPTER_VOLT_UPPER_BYP;
					pdpm->apdo_min_volt = ADAPTER_VOLT_LOWER_BYP;
				}

				pdpm->apdo_max_curr = pdpm->pdo[i].curr_ma;
				pdpm->selected_apdo = pdpm->pdo[i].pos;
			}
		} else if (pdpm->pdo[i].pos) {
			if (pdpm->pdo[i].curr_ma >= ADAPTER_CURR_MIN) {
				switch (pdpm->pdo[i].max_volt_mv) {
					case ADAPTER_VOLT_FIXED1:
						fpdo_type = PD_PM_FPDO_FIXED1;
						break;
					case ADAPTER_VOLT_FIXED2:
						fpdo_type = PD_PM_FPDO_FIXED1;
						break;
					case ADAPTER_VOLT_FIXED:
						fpdo_type = PD_PM_FPDO_FIXED;
						break;
					default:
						break;
				}
			}
			if (fpdo_type > pdpm->adapter_fpdo_type) {
				pdpm->adapter_fpdo_type = fpdo_type;
				pdpm->selected_fpdo = pdpm->pdo[i].pos;
			}
		}
	}
	
	if (pdpm->adapter_apdo_type > PD_PM_APDO_NONE) {
		pr_notice("PPS supported, preferred APDO type:%d pos:%d, max volt:%d, min volt:%d, current:%d",
				pdpm->adapter_apdo_type,
				pdpm->selected_apdo,
				pdpm->apdo_max_volt,
				pdpm->apdo_min_volt,
				pdpm->apdo_max_curr);
		pr_notice("preferred FPDO type:%d pos:%d",
				pdpm->adapter_fpdo_type,
				pdpm->selected_fpdo);
		
		if (pdpm->adapter_apdo_type == PD_PM_APDO_CP) {
			pm_config.vbus_times = 2;
			pm_config.ibus_regu_hys = IBUS_REGU_HYS;
			pm_config.bus_curr_lp_lmt = min(pm_config.bat_curr_lp_lmt/pm_config.vbus_times,
											pdpm->apdo_max_curr);
			usbpd_pm_enable_cp_bypass(pdpm, false);
		} else {
			pm_config.vbus_times = 1;
			pm_config.ibus_regu_hys = IBUS_REGU_HYS_BYP;
			pm_config.bus_curr_lp_lmt = min(pm_config.bat_curr_lp_lmt, pdpm->apdo_max_curr);
			usbpd_pm_enable_cp_bypass(pdpm, true);
		}
		g_fix_pdo_requested = false;
	}
	else {
		pr_notice("PPS not supported, select fixed pdo, preferred FPDO type:%d pos:%d",
				pdpm->adapter_fpdo_type,
				pdpm->selected_fpdo);
		if (pdpm->adapter_fpdo_type > PD_PM_FPDO_NONE) {
			ret = usbpd_request_voltage(pdpm, pdpm->selected_fpdo, 0, 0);
			if (ret)
				pr_err("Failed to request fixed pdo, use default pdo.");
		}
	}

	//check the adapter here.
	pm_config.is_qualified_adapter = true;
}

static void usbpd_update_pps_status(struct usbpd_pm *pdpm)
{
	int ret;
	u32 status = 0;

	ret = usbpd_get_pps_status(pdpm->pd, &status);

	if (!ret) {
		/*TODO: check byte order to insure data integrity*/
		if ((status & 0xFFFF) == 0xFF )
			pdpm->adapter_voltage = 0;
		else
			pdpm->adapter_voltage = (status & 0xFFFF )* 20;

		if (((status >> 16) & 0xFF) == 0xFF )
			pdpm->adapter_current = 0;
		else
			pdpm->adapter_current = ((status >> 16) & 0xFF ) * 50;
		
		pdpm->adapter_ptf = ((status >> 24) & 0x06) >> 1;
		pdpm->adapter_omf = !!((status >> 24) & 0x08);
		pr_debug("adapter_volt:%d, adapter_current:%d\n",
				pdpm->adapter_voltage, pdpm->adapter_current);
	}
}

static int usbpd_pm_fc3_charge_algo(struct usbpd_pm *pdpm)
{
	int steps;
	int step_vbat = 0;
	int step_ibus = 0;
	int step_ibat = 0;
#ifdef USE_HW_REGULATION
	int step_bat_reg = 0;
#endif
	int ibus_total = 0;
	int thermal_level = 0;
	int vbat_limit;
	int ibat_limit;
	static int ibus_limit;
	static int adapter_CV_timer = 0;
	static int fc3_taper_timer;

	usbpd_get_effective_fc_val(pdpm);

	vbat_limit = min(g_fcv_val, pm_config.bat_volt_lp_lmt);

	if (pm_config.is_qualified_adapter) {
		if (g_fcc_val == STEP1_CURRENT) {
			if (pdpm->cp.vbat_volt > (ADAPTER_CC_TO_CV_VOLTAGE - 2*VBAT_REGU_HYS)) {
				if (adapter_CV_timer++ >= STEP_TIMEOUT) {
					//adapter CV mode
					adapter_CV_timer = 0;
					ibus_limit = pm_config.bus_curr_lp_lmt - ADAPTER_CURRENT_TOLE;
				}
			} else if (pdpm->cp.vbat_volt < ADAPTER_CC_TO_CV_VOLTAGE - 200) {
				//adapter CC mode
				ibus_limit = pm_config.bus_curr_lp_lmt + ADAPTER_CURRENT_TOLE;
				adapter_CV_timer = 0;
			} else
				adapter_CV_timer = 0;

			if (ibus_limit == 0)
				ibus_limit = pm_config.bus_curr_lp_lmt + ADAPTER_CURRENT_TOLE;
			
			ibus_limit -= pm_config.sw_keep_current;
			ibat_limit = pm_config.bat_curr_lp_lmt + ADAPTER_CURRENT_TOLE*2;//not work, just in case
		}else {
			ibus_limit = pm_config.bus_curr_lp_lmt - ADAPTER_CURRENT_TOLE;
			ibat_limit = min(g_fcc_val, pm_config.bat_curr_lp_lmt);
			ibat_limit -= pm_config.sw_keep_current;
		}
	}
	else {
		ibat_limit = min(g_fcc_val, pm_config.bat_curr_lp_lmt);
		ibat_limit -= pm_config.sw_keep_current;
		ibus_limit = pm_config.bus_curr_lp_lmt - ADAPTER_CURRENT_TOLE;
	}
	
	/* battery voltage loop*/
	if (pdpm->cp.vbat_volt > vbat_limit)
		step_vbat = -pm_config.fc3_steps;
	else if (pdpm->cp.vbat_volt < vbat_limit - VBAT_REGU_HYS)
		step_vbat = pm_config.fc3_steps;

	/* battery charge current loop*/
	if (pdpm->cp.ibat_curr > ibat_limit)
		step_ibat = -pm_config.fc3_steps;
	else if (pdpm->cp.ibat_curr < ibat_limit - IBAT_REGU_HYS)
		step_ibat = pm_config.fc3_steps;

	/* bus current loop*/
	ibus_total = pdpm->cp.ibus_curr;
	if (pm_config.cp_sec_enable)
		ibus_total += pdpm->cp_sec.ibus_curr;

	if (ibus_total > ibus_limit)
		step_ibus = -pm_config.fc3_steps;
	else if (ibus_total < ibus_limit - pm_config.ibus_regu_hys)
		step_ibus = pm_config.fc3_steps;


	steps = min(min(step_vbat, step_ibus), step_ibat);

#ifdef USE_HW_REGULATION
	if (pdpm->cp.vbat_reg || pdpm->cp.ibat_reg)
		step_bat_reg = 5 * (-pm_config.fc3_steps);
	else
		step_bat_reg = pm_config.fc3_steps;

	steps = min(steps, step_bat_reg);
#endif

	pr_notice("steps:%d step_vbat:%d step_ibat:%d step_ibus:%d ibus_limit:%d ibat_limit:%d vbat:%d ibat:%d ibus:%d vbus:%d\n",
			steps, step_vbat, step_ibat, step_ibus, ibus_limit, ibat_limit,
			pdpm->cp.vbat_volt, pdpm->cp.ibat_curr, pdpm->cp.ibus_curr, pdpm->cp.vbus_volt);

	/* check if cp disabled due to other reason*/
	usbpd_pm_check_cp_enabled(pdpm);

	if (pm_config.cp_sec_enable)
		usbpd_pm_check_cp_sec_enabled(pdpm);

	pd_get_batt_current_thermal_level(pdpm, &thermal_level);

	pdpm->is_temp_out_fc3_range = pd_disable_cp_by_jeita_status(pdpm);

	if (pdpm->cp.health == POWER_SUPPLY_HEALTH_OVERHEAT
			|| pdpm->is_temp_out_fc3_range
			|| thermal_level >= MAX_THERMAL_LEVEL) {
		pr_notice("HEALTH:%d, is_temp_out:%d, thermal_level:%d\n",
				pdpm->cp.health, pdpm->is_temp_out_fc3_range, thermal_level);
		fc3_taper_timer = 0;
		adapter_CV_timer = 0;
		return PM_ALGO_RET_THERM_FAULT;
	} else if (pdpm->cp.health == POWER_SUPPLY_HEALTH_OVERVOLTAGE
			|| pdpm->cp.health == POWER_SUPPLY_HEALTH_OVERCURRENT) {
		pr_notice("OVP or OCP, HEALTH:%d", pdpm->cp.health);
		fc3_taper_timer = 0;
		adapter_CV_timer = 0;
		return PM_ALGO_RET_OTHER_FAULT; /* go to switch, and try to ramp up*/
	} else if (!pdpm->cp.charge_enabled 
			|| (pm_config.cp_sec_enable && !pdpm->cp_sec.charge_enabled && !pdpm->cp_sec_stopped)) {
		pr_notice("cp.charge_enabled:%d, cp_sec.charge_enabled:%d\n",
				pdpm->cp.charge_enabled, pdpm->cp_sec.charge_enabled);
		fc3_taper_timer = 0;
		adapter_CV_timer = 0;
		return PM_ALGO_RET_CHG_DISABLED;
	}

	/* charge pump taper charge */
	if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 50 &&
			(is_cool_charge(pdpm) ||
			pdpm->cp.ibat_curr < pm_config.fc3_taper_current)) {
		if (fc3_taper_timer++ >= TAPER_TIMEOUT) {
			pr_notice("charge pump taper charging done\n");
			fc3_taper_timer = 0;
			return PM_ALGO_RET_TAPER_DONE;
		}
	} else
		fc3_taper_timer = 0;
	   
	pdpm->request_voltage += steps * 20;

	if (pdpm->adapter_voltage > 0
			&& pdpm->request_voltage > pdpm->adapter_voltage + 1000)
		pdpm->request_voltage = pdpm->adapter_voltage + 1000;

	if (pdpm->request_voltage > pdpm->apdo_max_volt)
		pdpm->request_voltage = pdpm->apdo_max_volt;

	return PM_ALGO_RET_OK;
}

static void usbpd_pm_move_state(struct usbpd_pm *pdpm, enum pm_state state)
{
#if 1
	pr_debug("state change:%s -> %s\n", 
		pm_str[pdpm->state], pm_str[state]);
#endif
	pdpm->state = state;
}

static int usbpd_pm_sm(struct usbpd_pm *pdpm)
{
	int ret;
	int rc = 0;
	static int tune_vbus_retry;
	static bool stop_sw;
	static bool recover;
	int thermal_level = 0;
	int gap = 0;

	switch (pdpm->state) {
	case PD_PM_STATE_ENTRY:
		stop_sw = false;
		recover = false;

		pd_get_batt_current_thermal_level(pdpm, &thermal_level);
		pdpm->is_temp_out_fc3_range = pd_disable_cp_by_jeita_status(pdpm);

		if (thermal_level >= MAX_THERMAL_LEVEL
				|| pdpm->is_temp_out_fc3_range) {
			pr_notice("thermal too high or batt temp is out of fc3 range, waiting...\n");
		} else if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - VBAT_GAP_FOR_CP) {
			pr_notice("batt_volt is %d, exit and charge with switch charger\n", pdpm->cp.vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_EXIT);
		} else if (pdpm->cp.vbat_volt < pm_config.min_vbat_for_cp) {
			pr_notice("batt_volt is %d, charge with switch charger first.", pdpm->cp.vbat_volt);
			if (pdpm->adapter_fpdo_type > PD_PM_FPDO_NONE && !g_fix_pdo_requested) {
				ret = usbpd_request_voltage(pdpm, pdpm->selected_fpdo, 0, 0);
				if (ret) {
					pr_err("Failed to request fixed pdo, retry...");
				}
				else
					g_fix_pdo_requested = true;
			}
		} else {
			pr_notice("batt_volt-%d is ok, start flash charging\n",
					pdpm->cp.vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_ENTRY);
			tune_vbus_retry = 0;
		}
		break;

	case PD_PM_STATE_FC3_ENTRY:
		
#ifdef LIMIT_SWITCH_CHARGER
		if (!pdpm->sw.charge_limited) {
			usbpd_pm_limit_sw(pdpm, 2000);
			usbpd_pm_check_sw_limited(pdpm);
		}
		if (pdpm->sw.charge_limited)
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_ENTRY_1);
#else
		usbpd_pm_check_sw_charging(pdpm);
		if (pdpm->sw.charging) {
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_ENTRY_1);
			usbpd_pm_enable_sw(pdpm, false);
			tune_vbus_retry = 0;
		}
		else if (tune_vbus_retry++ > 20) {
			pr_err("Switch charge is not working, exit. Tried %d times.", tune_vbus_retry);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_EXIT);
		}
#endif

		break;

	case PD_PM_STATE_FC3_ENTRY_1:
		if (pm_config.cp_sec_enable)
			pdpm->request_voltage = pdpm->cp.vbat_volt * pm_config.vbus_times + BUS_VOLT_INIT_UP * 2;
		else
			pdpm->request_voltage = pdpm->cp.vbat_volt * pm_config.vbus_times + BUS_VOLT_INIT_UP;
			
		pdpm->request_current = pm_config.bus_curr_lp_lmt;

		ret = usbpd_request_voltage(pdpm, pdpm->selected_apdo, pdpm->request_voltage, pdpm->request_current);
		if (ret) {
			if (tune_vbus_retry++ >= 10) {
				pr_err("Failed to request pdo %d times, exit.",	tune_vbus_retry);
				usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_EXIT);
			}
			pr_err("Failed to request pdo %d times, retry...", tune_vbus_retry);
		} else {
			tune_vbus_retry = 0;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_ENTRY_2);
		}
		break;

	case PD_PM_STATE_FC3_ENTRY_2:
		pr_notice("entry2 vbus %d, vbat %d\n", pdpm->cp.vbus_volt, pdpm->cp.vbat_volt);

		if ((pdpm->cp.vbus_volt < (pdpm->cp.vbat_volt * pm_config.vbus_times + BUS_VOLT_INIT_UP - 50)) ||
				(pdpm->cp.vbus_volt > (pdpm->cp.vbat_volt * pm_config.vbus_times + BUS_VOLT_INIT_UP + 50))
				|| tune_vbus_retry == 0) {

			if (tune_vbus_retry++ >= 20) {
				pr_err("Failed to tune adapter volt into valid range, exit. Tried %d times.",
						tune_vbus_retry);
				usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_EXIT);
			}

			gap = pdpm->cp.vbat_volt * pm_config.vbus_times + BUS_VOLT_INIT_UP - pdpm->cp.vbus_volt;
			pdpm->request_voltage += gap;
			ret = usbpd_request_voltage(pdpm, pdpm->selected_apdo, pdpm->request_voltage, pdpm->request_current);
			if (ret) {
				pdpm->request_voltage -= gap;
				pr_err("Failed to request pdo.");
			}
		} else {
			pr_notice("adapter volt tune ok, retry %d times\n", tune_vbus_retry);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_ENTRY_3);
		}

		break;
	case PD_PM_STATE_FC3_ENTRY_3:
		
#ifdef LIMIT_SWITCH_CHARGER
		usbpd_pm_limit_sw(pdpm, pm_config.sw_keep_current);
		usbpd_pm_check_sw_limited(pdpm);
		if (!pdpm->sw.charge_limited) {
			pr_err("Limit switch charger fail, retry");
			break;
		}
#elif defined DISABLE_SWITCH_CHARGER
		usbpd_pm_check_sw_enabled(pdpm);
		if (pdpm->sw.charge_enabled) {
			usbpd_pm_enable_sw(pdpm, false);
			usbpd_pm_check_sw_enabled(pdpm);
		}
		if (pdpm->sw.charge_enabled) {
			pr_err("Disable switch charger fail, retry");
			break;
		}
#endif
		
		if (pm_config.cp_sec_enable && !pdpm->cp_sec.charge_enabled) {
			usbpd_pm_enable_cp_sec(pdpm, true);
			usbpd_pm_check_cp_sec_enabled(pdpm);
		}
		
		if (!pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm, true);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		if (pdpm->cp.charge_enabled &&
			((pm_config.cp_sec_enable && pdpm->cp_sec.charge_enabled)
				|| !pm_config.cp_sec_enable)) {
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_TUNE);
			usbpd_init_fc_val(pdpm);
		}
		else {
			pr_notice("Enable cp fail, retry.");
		}
		
		break;

	case PD_PM_STATE_FC3_TUNE:
		usbpd_update_pps_status(pdpm);

		ret = usbpd_pm_fc3_charge_algo(pdpm);
		if (ret == PM_ALGO_RET_THERM_FAULT) {
			pr_notice("Move to stop charging:%d\n", ret);
			stop_sw = true;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_OTHER_FAULT || ret == PM_ALGO_RET_TAPER_DONE) {
			pr_notice("Move to switch charging:%d\n", ret);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_CHG_DISABLED) {
			pr_notice("Move to switch charging, will try to recover flash charging:%d\n",
					ret);
			recover = true;
		        usbpd_pm_move_state(pdpm, PD_PM_STATE_FC3_EXIT);
			break;
		} else {//ret == PM_ALGO_RET_OK
			ret = usbpd_request_voltage(pdpm, pdpm->selected_apdo, pdpm->request_voltage, pdpm->request_current);
			if (ret) {
				pr_err("Failed to request pdo.");
			}
		}
		/*stop second charge pump if either of ibus is lower than 750ma during CV*/
		if (pm_config.cp_sec_enable && pdpm->cp_sec.charge_enabled 
				&& pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 50
				&& (pdpm->cp.ibus_curr < 750 || pdpm->cp_sec.ibus_curr < 750)) {
			pr_notice("second cp is disabled due to ibus < 750mA\n");
			usbpd_pm_enable_cp_sec(pdpm, false);
			usbpd_pm_check_cp_sec_enabled(pdpm);
			pdpm->cp_sec_stopped = true;
		}
		break;

	case PD_PM_STATE_FC3_EXIT:
		if (pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm, false);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		if (pm_config.cp_sec_enable && pdpm->cp_sec.charge_enabled) {
			usbpd_pm_enable_cp_sec(pdpm, false);
			usbpd_pm_check_cp_sec_enabled(pdpm);
		}

		/* set a voltage for switch charger*/
		if (pdpm->adapter_fpdo_type > PD_PM_FPDO_NONE) {
			ret = usbpd_request_voltage(pdpm, pdpm->selected_fpdo, 0, 0);
			if (ret)
				pr_err("Failed to request fix pdo for switch charger.");
		}

#ifdef LIMIT_SWITCH_CHARGER
		if (!stop_sw && pdpm->sw.charge_limited) {
			usbpd_pm_limit_sw(pdpm, 3000);
			usbpd_pm_check_sw_limited(pdpm);
		}
#elif defined DISABLE_SWITCH_CHARGER
		usbpd_pm_check_sw_enabled(pdpm);
		if (!stop_sw && !pdpm->sw.charge_enabled) {
			usbpd_pm_enable_sw(pdpm, true);
			usbpd_pm_check_sw_enabled(pdpm);
		}
#endif

		if (recover)
			usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
		else {
			rc = 1;
			usbpd_pm_enable_cp_adc(pdpm, false);
		}

	    break;
	}

	return rc;
}

static void usbpd_pm_workfunc(struct work_struct *work)
{
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
					pm_work.work);

	usbpd_pm_update_cp_status(pdpm);

	if (pm_config.cp_sec_enable)
		usbpd_pm_update_cp_sec_status(pdpm);

	if (!usbpd_pm_sm(pdpm) && pdpm->pd_active)
		schedule_delayed_work(&pdpm->pm_work,
				msecs_to_jiffies(PM_WORK_RUN_INTERVAL));
}

static void usbpd_pm_disconnect(struct usbpd_pm *pdpm)
{

	cancel_delayed_work(&pdpm->pm_work);
	usbpd_pm_enable_cp_adc(pdpm, false);

#ifdef LIMIT_SWITCH_CHARGER
	if(pdpm->sw.charge_limited)
	{
		usbpd_pm_limit_sw(pdpm, 3000);
		usbpd_pm_check_sw_limited(pdpm);
	}
#elif defined DISABLE_SWITCH_CHARGER
	usbpd_pm_check_sw_enabled(pdpm);
	if (!pdpm->sw.charge_enabled) {
		usbpd_pm_enable_sw(pdpm, true);
		usbpd_pm_check_sw_enabled(pdpm);
	}
#endif
}

static void usbpd_pd_contact(struct usbpd_pm *pdpm, bool connected)
{
	if (connected) {
		pdpm->jeita_triggered = false;
		pdpm->is_temp_out_fc3_range = false;
		pdpm->cp_sec_stopped = false;
		memset(&pdpm->pdo, 0, sizeof(pdpm->pdo));
		usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
		
		usbpd_pm_evaluate_src_caps(pdpm);
		if (pdpm->adapter_apdo_type > PD_PM_APDO_NONE) {
			usbpd_pm_enable_cp_adc(pdpm, true);
			msleep(100);
			schedule_delayed_work(&pdpm->pm_work, 0);
		}
	} else {
		usbpd_pm_disconnect(pdpm);
	}
}

static void usb_psy_change_work(struct work_struct *work)
{
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
					usb_psy_change_work);
	union power_supply_propval val = {0,};
	int ret = 0;

	ret = power_supply_get_property(pdpm->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_POWER_ROLE,&val);
	if (ret) {
		pr_err("Failed to read typec power role\n");
		goto out;
	}

	if (val.intval != POWER_SUPPLY_TYPEC_PR_SINK && 
			val.intval != POWER_SUPPLY_TYPEC_PR_DUAL)
		goto out;

	ret = power_supply_get_property(pdpm->usb_psy,
			POWER_SUPPLY_PROP_PD_ACTIVE, &val);
	if (ret) {
		pr_err("Failed to get usb pd active state\n");
		goto out;
	}

	if (!pdpm->pd_active && (val.intval == POWER_SUPPLY_PD_PPS_ACTIVE
			|| val.intval == POWER_SUPPLY_PD_PPS_ACTIVE)) {
		pdpm->pd_active = val.intval;
		usbpd_pd_contact(pdpm, true);
	}
	else if (pdpm->pd_active && !val.intval) {
		pdpm->pd_active = val.intval;
		usbpd_pd_contact(pdpm, false);
	}
out:
	pdpm->psy_change_running = false;
}	

static int usbpd_psy_notifier_cb(struct notifier_block *nb, 
			unsigned long event, void *data)
{
	struct usbpd_pm *pdpm = container_of(nb, struct usbpd_pm, nb);
	struct power_supply *psy = data;
	unsigned long flags;

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	usbpd_check_usb_psy(pdpm);

	if (!pdpm->usb_psy)
		return NOTIFY_OK;

	if (psy == pdpm->usb_psy) {
		spin_lock_irqsave(&pdpm->psy_change_lock, flags);
		if (!pdpm->psy_change_running) {
			pdpm->psy_change_running = true;
			schedule_work(&pdpm->usb_psy_change_work);
		}
		spin_unlock_irqrestore(&pdpm->psy_change_lock, flags);
	}

	return NOTIFY_OK;
}

static int __init usbpd_pm_init(void)
{
	struct usbpd_pm *pdpm;

	pdpm = kzalloc(sizeof(struct usbpd_pm), GFP_KERNEL);
	if (!pdpm)
		return -ENOMEM;

	__pdpm = pdpm;

	INIT_WORK(&pdpm->usb_psy_change_work, usb_psy_change_work);

	spin_lock_init(&pdpm->psy_change_lock);

	usbpd_check_cp_psy(pdpm);
	if (pm_config.cp_sec_enable)
		usbpd_check_cp_sec_psy(pdpm);
	usbpd_check_usb_psy(pdpm);
	
	INIT_DELAYED_WORK(&pdpm->pm_work, usbpd_pm_workfunc);
	
	pdpm->nb.notifier_call = usbpd_psy_notifier_cb;
	power_supply_reg_notifier(&pdpm->nb);

	return 0;
}

static void __exit usbpd_pm_exit(void)
{
	power_supply_unreg_notifier(&__pdpm->nb);
	cancel_delayed_work(&__pdpm->pm_work);
	cancel_work_sync(&__pdpm->usb_psy_change_work);
	usbpd_pm_enable_cp_adc(__pdpm, false);
}

module_init(usbpd_pm_init);
module_exit(usbpd_pm_exit);
