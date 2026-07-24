#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* 数学常量 */
#define FOC_PI            3.14159265358979f
#define FOC_PI_2          1.57079632679490f
#define FOC_2PI           6.28318530717959f
#define FOC_SQRT3_2       0.86602540378444f   /* √3/2 */
#define FOC_INV_SQRT3     0.57735026918963f   /* 1/√3 */

/* 控制循环时序 (默认 10kHz 电流环, 1kHz 速度环, 200Hz 位置环) */
#define FOC_CURRENT_LOOP_FREQ_HZ  10000.0f
#define FOC_SPEED_LOOP_DIV        10
#define FOC_POS_LOOP_DIV          5
#define FOC_DT_CURRENT            (1.0f / FOC_CURRENT_LOOP_FREQ_HZ)
#define FOC_DT_SPEED              (FOC_DT_CURRENT * FOC_SPEED_LOOP_DIV)
#define FOC_DT_POSITION           (FOC_DT_SPEED * FOC_POS_LOOP_DIV)

/* 默认运行限幅 */
#define FOC_CURRENT_LIMIT_DEFAULT     10.0f   /* 电流限幅 [A] */
#define FOC_SPEED_LIMIT_DEFAULT       300.0f  /* 速度限幅 [rad/s] */
#define FOC_VOLTAGE_LIMIT_DEFAULT     24.0f   /* 电压限幅 [V] */
#define FOC_VOLTAGE_LIMIT_RATIO       0.95f   /* SVPWM 最大调制比 */

/* 编码器默认配置 */
#define FOC_ENCODER_CPR_DEFAULT       4096    /* 编码器线数 (4 倍频后) */
#define FOC_ENCODER_FILTER_ALPHA      0.1f    /* 测速低通滤波系数 */

/* SVPWM 配置 */
#define FOC_SVPWM_OVM_ENABLE          1       /* 启用过调制 */
#define FOC_SVPWM_OVM_LIMIT           1.05f   /* 过调制限制 */
#define FOC_SVPWM_MIN_PULSE           0.005f  /* 最小占空比 0.5% (Vmin=84mV, Imin=0.7A) */
#define FOC_SVPWM_MAX_PULSE           0.95f   /* 最大占空比 */

/* CAN 总线波特率 */
#define FOC_CAN_BAUDRATE              500000

#ifdef __cplusplus
}
#endif

#endif
