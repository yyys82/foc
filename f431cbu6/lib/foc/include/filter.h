#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 一阶低通滤波器: y[n] = α·x[n] + (1-α)·y[n-1] */
typedef struct {
    float alpha;               /* 滤波系数 [0,1], 越小越平滑 */
    float y_prev;              /* 上一帧输出 */
    uint8_t initialized;       /* 首次更新时直接赋输入 */
} foc_lpf_t;

void lpf_init(foc_lpf_t *filt, float alpha);
void lpf_set_alpha(foc_lpf_t *filt, float alpha);
float lpf_update(foc_lpf_t *filt, float x);

/* 滑动平均滤波器 (固定 4 点窗口) */
typedef struct {
    float buf[4];
    uint32_t index;
} foc_movavg_t;

void movavg_init(foc_movavg_t *ma);
float movavg_update(foc_movavg_t *ma, float x, uint32_t n);

#ifdef __cplusplus
}
#endif

#endif
