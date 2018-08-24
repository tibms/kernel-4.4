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

#ifndef _LINUX_BQ2429X_I2C_H
#define _LINUX_BQ2429X_I2C_H

#include <linux/power_supply.h>


enum bq2429x_input_current_limit {
	BQ2429X_ILIM_100mA 	= 100,
	BQ2429X_ILIM_150mA 	= 150,
	BQ2429X_ILIM_500mA	= 500,
	BQ2429X_ILIM_900mA	= 900,
	BQ2429X_ILIM_1000mA	= 1000,
	BQ2429X_ILIM_1500mA	= 1500,
	BQ2429X_ILIM_2000mA	= 2000,
	BQ2429X_ILIM_3000mA	= 3000,
};


struct bq2429x_charge_param {
	int vlim;
	int ilim;
	int ichg;
	int vreg;
};




struct bq2429x_platform_data {
	struct bq2429x_charge_param usb;
	struct bq2429x_charge_param ta;
	
	int boostv;
	int iprechg;
	int iterm;
	

	bool enable_term;	
};

#endif
