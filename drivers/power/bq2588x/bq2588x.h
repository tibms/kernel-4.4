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

#ifndef _LINUX_BQ2588X_I2C_H
#define _LINUX_BQ2588X_I2C_H

#include <linux/power_supply.h>


struct bq2588x_charge_param {
	int vlim;
	int ilim;
	int ichg;
	int vreg;
};

struct bq2588x_platform_data {
	struct bq2588x_charge_param usb;
	struct bq2588x_charge_param ta;
	int iprechg;
	int iterm;
	
	int otg_volt;
	int otg_current;
	
};

#endif
