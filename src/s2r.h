/* SPDX-License-Identifier: MIT */

#ifndef S2R_H
#define S2R_H

#include "types.h"

void s2r_configure(const char *val);
void s2r_test(void);

bool s2r_resume_pending(void);
u64 s2r_record_addr(void);
void s2r_resume(void) __attribute__((noreturn));

#endif
