#ifndef PARK_H
#define PARK_H

#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Park 变换: αβ 静止坐标系 → dq 旋转坐标系 (输入角度需含 sin/cos) */
void park_transform(const foc_alphabeta_t *ab, const foc_angle_t *angle, foc_dq_t *dq);

/* 逆 Park 变换: dq → αβ */
void park_transform_inv(const foc_dq_t *dq, const foc_angle_t *angle, foc_alphabeta_t *ab);

#ifdef __cplusplus
}
#endif

#endif
