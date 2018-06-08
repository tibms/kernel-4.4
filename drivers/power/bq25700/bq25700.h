/*
 * BQ2570x battery charging driver
 *
 * Copyright (C) 2017 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef _LINUX_BQ25700_H
#define _LINUX_BQ25700_H

#include <linux/power_supply.h>

struct bq25700_platform_data {

	int chg_mv;
	int chg_ma;
	int ivl_mv;
	int icl_ma;

	int boostv;
	int boosti;
};

#endif
