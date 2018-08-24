
#ifndef __BQ2429X_HEADER__
#define __BQ2429X_HEADER__

/* Register 00h */
#define BQ2429X_REG_00      		0x00
#define REG00_ENHIZ_MASK		    0x80
#define REG00_ENHIZ_SHIFT		    7
#define	REG00_HIZ_ENABLE			1
#define	REG00_HIZ_DISABLE			0

#define	REG00_VINDPM_MASK			0x78
#define	REG00_VINDPM_SHIFT			3
#define	REG00_VINDPM_BASE			3880
#define	REG00_VINDPM_LSB			80

#define REG00_IINLIM_MASK		    0x07
#define REG00_IINLIM_SHIFT			0
#define REG00_IINLIM_100MA			0
#define REG00_IINLIM_150MA			1
#define REG00_IINLIM_500MA			2
#define REG00_IINLIM_900MA			3
#define REG00_IINLIM_1000MA			4
#define REG00_IINLIM_1500MA			5
#define REG00_IINLIM_2000MA			6
#define REG00_IINLIM_3000MA			7



/* Register 01h */
#define BQ2429X_REG_01		    	0x01
#define	REG01_REG_RESET_MASK		0x80
#define	REG01_REG_RESET_SHIFT		7
#define	REG01_REG_RESET				1

#define REG01_WDT_RESET_MASK		0x40
#define REG01_WDT_RESET_SHIFT		6
#define REG01_WDT_RESET				1

#define REG01_OTG_CONFIG_MASK		0x20
#define REG01_OTG_CONFIG_SHIFT		5
#define REG01_OTG_DISABLE 			0
#define REG01_OTG_ENABLE 			1 

#define REG01_CHG_CONFIG_MASK     	0x1
#define REG01_CHG_CONFIG_SHIFT    	4
#define REG01_CHG_DISABLE        	0
#define REG01_CHG_ENABLE         	1

#define REG01_SYS_MINV_MASK       	0x0E
#define REG01_SYS_MINV_SHIFT      	1
#define	REG01_SYS_MINV_BASE 		3000
#define REG01_SYS_MINV_LSB			100

/* Register 0x02*/
#define BQ2429X_REG_02              0x02
#define REG02_ICHG_MASK           	0xFC
#define REG02_ICHG_SHIFT          	2
#define REG02_ICHG_BASE           	512
#define REG02_ICHG_LSB            	64

#define REG02_BCOLD_MASK			0x02
#define REG02_BCOLD_SHIFT 			1
#define REG02_BCOLD_M10C			0 
#define REG02_BCOLD_M20C			1 

#define REG02_FORCE_20PCT_MASK		0x01 
#define REG02_FORCE_20PCT_SHIFT 	0 
#define REG02_FORCE_20PCT_ENABLE	1 
#define REG02_FORCE_20PCT_DISABLE	0
 
/* Register 0x03*/
#define BQ2429X_REG_03              0x03
#define REG03_IPRECHG_MASK        	0xF0
#define REG03_IPRECHG_SHIFT       	4
#define REG03_IPRECHG_BASE        	128
#define REG03_IPRECHG_LSB         	128

#define REG03_ITERM_MASK          	0x0F
#define REG03_ITERM_SHIFT         	0
#define REG03_ITERM_BASE          	128
#define REG03_ITERM_LSB           	128


/* Register 0x04*/
#define BQ2429X_REG_04              0x04
#define REG04_VREG_MASK           	0xFC
#define REG04_VREG_SHIFT          	2
#define REG04_VREG_BASE           	3504
#define REG04_VREG_LSB            	16

#define REG04_BATLOWV_MASK         	0x02
#define REG04_BATLOWV_SHIFT        	1
#define REG04_BATLOWV_2800MV        0
#define REG04_BATLOWV_3000MV        1

#define REG04_VRECHG_MASK         	0x01
#define REG04_VRECHG_SHIFT        	0
#define REG04_VRECHG_100MV        	0
#define REG04_VRECHG_300MV        	1

/* Register 0x05*/
#define BQ2429X_REG_05             	0x05
#define REG05_EN_TERM_MASK        	0x80
#define REG05_EN_TERM_SHIFT       	7
#define REG05_TERM_ENABLE         	1
#define REG05_TERM_DISABLE        	0

#define REG05_WDT_MASK            	0x30
#define REG05_WDT_SHIFT           	4
#define REG05_WDT_DISABLE         	0
#define REG05_WDT_40S             	1
#define REG05_WDT_80S             	2
#define REG05_WDT_160S            	3
#define REG05_WDT_BASE            	0
#define REG05_WDT_LSB             	40

#define REG05_EN_TIMER_MASK       	0x08
#define REG05_EN_TIMER_SHIFT      	3
#define REG05_CHG_TIMER_ENABLE    	1
#define REG05_CHG_TIMER_DISABLE   	0

#define REG05_CHG_TIMER_MASK      	0x06
#define REG05_CHG_TIMER_SHIFT     	1
#define REG05_CHG_TIMER_5HOURS    	0
#define REG05_CHG_TIMER_8HOURS   	1
#define REG05_CHG_TIMER_12HOURS   	2
#define REG05_CHG_TIMER_20HOURS   	3

//#define REG05_JEITA_ISET_MASK     	0x01
//#define REG05_JEITA_ISET_SHIFT    	0
//#define REG05_JEITA_ISET_50PCT    	0
//#define REG05_JEITA_ISET_20PCT    	1


/* Register 0x06*/
#define BQ2429X_REG_06              0x06
#define	REG06_BOOSTV_MASK			0xF0
#define	REG06_BOOSTV_SHIFT			4
#define	REG06_BOOSTV_LSB			64
#define	REG06_BOOSTV_BASE			4550

#define REG06_BHOT_MASK				0x0C
#define REG06_BHOT_SHIFT			2
#define REG06_BHOT_55C				0 
#define REG06_BHOT_60C				1 
#define REG06_BHOT_65C				2 
#define REG06_BHOT_DISABLE			3 

#define	REG06_TREG_MASK				0x03
#define	REG06_TREG_SHIFT			0
#define	REG06_TREG_60C				0
#define	REG06_TREG_80C				1
#define	REG06_TREG_100C				2
#define	REG06_TREG_120C				3

/* Register 0x07*/
#define BQ2429X_REG_07              0x07
#define REG07_FORCE_DPDM_MASK     	0x80
#define REG07_FORCE_DPDM_SHIFT    	7
#define REG07_FORCE_DPDM          	1

#define REG07_TMR2X_EN_MASK       	0x40
#define REG07_TMR2X_EN_SHIFT      	6
#define REG07_TMR2X_ENABLE        	1
#define REG07_TMR2X_DISABLE       	0

#define REG07_BATFET_DIS_MASK     	0x20
#define REG07_BATFET_DIS_SHIFT    	5
#define REG07_BATFET_OFF          	1

#define	REG07_CHG_FLT_INT_MASK		0x02
#define	REG07_CHG_FLT_INT_SHIFT		1
#define	REG07_CHG_FLT_INT_DISABLE	0
#define	REG07_CHG_FLT_INT_ENABLE	1

#define	REG07_BAT_FLT_INT_MASK		0x01
#define	REG07_BAT_FLT_INT_SHIFT		0
#define	REG07_BAT_FLT_INT_DISABLE	0
#define	REG07_BAT_FLT_INT_ENABLE	1

/* Register 0x08*/
#define BQ2429X_REG_08              0x08
#define REG08_VBUS_STAT_MASK      	0xC0           
#define REG08_VBUS_STAT_SHIFT     	6
#define REG08_VBUS_TYPE_NONE	  	0
#define REG08_VBUS_TYPE_USB       	1
#define REG08_VBUS_TYPE_ADAPTER   	2
#define REG08_VBUS_TYPE_OTG       	3

#define REG08_CHRG_STAT_MASK      	0x30
#define REG08_CHRG_STAT_SHIFT     	4
#define REG08_CHRG_STAT_IDLE      	0
#define REG08_CHRG_STAT_PRECHG    	1
#define REG08_CHRG_STAT_FASTCHG   	2
#define REG08_CHRG_STAT_CHGDONE   	3

#define	REG08_INDPM_STAT_MASK		0x08
#define	REG08_INDPM_STAT_SHIFT		3
#define	REG08_INDPM_ACTIVE			1

#define REG08_PG_STAT_MASK        	0x04
#define REG08_PG_STAT_SHIFT       	2
#define REG08_POWER_GOOD          	1

#define REG08_THERM_STAT_MASK     	0x02
#define REG08_THERM_STAT_SHIFT    	1
#define REG08_IN_THERM_STAT			1

#define REG08_VSYS_STAT_MASK      0x01
#define REG08_VSYS_STAT_SHIFT     0
#define REG08_IN_VSYS_STAT        1


/* Register 0x09*/
#define BQ2429X_REG_09              0x09
#define REG09_FAULT_WDT_MASK      	0x80
#define REG09_FAULT_WDT_SHIFT     	7
#define REG09_FAULT_WDT           	1

#define REG09_FAULT_BOOST_MASK    	0x40
#define REG09_FAULT_BOOST_SHIFT   	6

#define REG09_FAULT_CHRG_MASK     	0x30
#define REG09_FAULT_CHRG_SHIFT    	4
#define REG09_FAULT_CHRG_NORMAL   	0
#define REG09_FAULT_CHRG_INPUT    	1
#define REG09_FAULT_CHRG_THERMAL  	2
#define REG09_FAULT_CHRG_TIMER    	3

#define REG09_FAULT_BAT_MASK      	0x08
#define REG09_FAULT_BAT_SHIFT     	3
#define	REG09_FAULT_BAT_OVP			1

#define REG09_FAULT_NTC_MASK      	0x07
#define REG09_FAULT_NTC_SHIFT     	0
#define	REG09_FAULT_NTC_NORMAL		0
#define REG09_FAULT_NTC_HOT 		1
#define REG09_FAULT_NTC_COLD		2 


/* Register 0x0A */
#define BQ2429X_REG_0A              0x0A

#define REG0A_PN_MASK             	0xE0
#define REG0A_PN_SHIFT            	5
#define REG0A_PN_BQ24295 			6 
#define REG0A_PN_BQ24296			1 
#define REG0A_PN_BQ24297			3 

#define REG0A_DEV_REV_MASK        	0x07
#define REG0A_DEV_REV_SHIFT       	0


#endif


