#ifndef SVPWM_H
#define SVPWM_H

#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SVPWM 输出: 三相占空比 [0,1] + ADC 触发时刻 (双电阻采样用) */
typedef struct {
    float ta;                  /* A 相上桥占空比 */
    float tb;                  /* B 相上桥占空比 */
    float tc;                  /* C 相上桥占空比 */
    uint32_t sector;           /* 当前扇区 (1-6, 保留) */
    float t1;                  /* 扇区边界矢量作用时间 (保留) */
    float t2;                  /* 另一矢量作用时间 (保留) */
    float adc_trig_u;          /* U 相 ADC 触发点 (相对 PWM 周期) */
    float adc_trig_v;          /* V 相 ADC 触发点 */
} foc_svpwm_t;

/* 载波注入法 SVPWM: 输入 Vα/Vβ/母线电压 → 三路占空比 */
void svpwm_calc(const foc_alphabeta_t *v_ab, float vdc, foc_svpwm_t *svpwm);

/* 过调制处理: 比例压缩超出 limit 的分量 */
void svpwm_apply_ovm(foc_svpwm_t *svpwm, float limit);

#ifdef __cplusplus
}
#endif

#endif
