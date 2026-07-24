#ifndef CLARKE_H
#define CLARKE_H

#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Clarke 变换: 三相 abc → αβ 静止坐标系 (幅度不变) */
void clarke_transform(const foc_abc_t *abc, foc_alphabeta_t *ab);

/* 逆 Clarke 变换: αβ → 三相 abc */
void clarke_transform_inv(const foc_alphabeta_t *ab, foc_abc_t *abc);

#ifdef __cplusplus
}
#endif

#endif
