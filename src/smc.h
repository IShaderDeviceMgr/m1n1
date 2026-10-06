/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef SMC_H
#define SMC_H

#include "asc.h"
#include "rtkit.h"
#include "types.h"

typedef struct smc_dev smc_dev_t;

int smc_write_u32(smc_dev_t *smc, u32 key, u32 value);
int smc_read(smc_dev_t *smc, u32 key, void *buf, size_t size);
int smc_read_u32(smc_dev_t *smc, u32 key, u32 *value);
bool smc_set_ap_power(smc_dev_t *smc, u32 state);

smc_dev_t *smc_init(void);
void smc_shutdown(smc_dev_t *smc);

#endif
