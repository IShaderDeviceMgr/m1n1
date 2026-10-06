/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef __WDT_H__
#define __WDT_H__

#include "types.h"

void wdt_disable(void);
void wdt_reboot(void);
void wdt_arm(u32 seconds);

#endif
