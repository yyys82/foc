#ifndef FOC_APP_H
#define FOC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_core.h"

extern foc_core_t g_core;

void foc_app_init(void);
void foc_app_loop(void);
void foc_app_isr(void);

#ifdef __cplusplus
}
#endif

#endif
