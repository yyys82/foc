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

/* 控制循环时序 (实际 25kHz 电流环, 1kHz 速度环, 200Hz 位置环) */
#define FOC_CURRENT_LOOP_FREQ_HZ  25000.0f
#define FOC_SPEED_LOOP_DIV        25  /* 电流环:速度环 = 25:1，仅用于推导 FOC_DT_SPEED(=1/1000)，非循环计数器 */
#define FOC_POS_LOOP_DIV          5   /* 速度环:位置环 = 5:1，FOC_DT_POSITION=1/200 */
#define FOC_DT_CURRENT            (1.0f / FOC_CURRENT_LOOP_FREQ_HZ)
#define FOC_DT_SPEED              (FOC_DT_CURRENT * FOC_SPEED_LOOP_DIV)
#define FOC_DT_POSITION           (FOC_DT_SPEED * FOC_POS_LOOP_DIV)

/* 默认运行限幅 */
#define FOC_CURRENT_LIMIT_DEFAULT     2.5f    /* 电流命令限幅 [A]：≤3A 小电流应用，留 0.5A 裕量到 3.0A 过流故障点；如需更大连续电流须降增益/减 Rs */
#define FOC_SPEED_DEADBAND            0.0f    /* 速度环反馈死区：关闭(编码器驱动内已有滑动窗口滤波) */
#define FOC_POS_DEADBAND              0.003f  /* 位置环死区 [mech rad]：AS5600 1LSB≈0.0015rad，死区取2LSB */
#define FOC_SPEED_LIMIT_DEFAULT       300.0f  /* 速度限幅 [rad/s] */
#define FOC_VOLTAGE_LIMIT_DEFAULT     24.0f   /* 电压限幅 [V] */
#define FOC_VOLTAGE_LIMIT_RATIO       0.95f   /* SVPWM 最大调制比 */

/* 编码器默认配置 */
#define FOC_ENCODER_CPR_DEFAULT       4096    /* 编码器线数 (4 倍频后) */
#define FOC_ENCODER_FILTER_ALPHA      0.1f    /* 测速低通滤波系数 */

/* 电气角帧修正：elec = mech*pp - offset_rad + FOC_FRAME_CORR。
 * 根因已修（_set_duty 恢复正确 B/C 相序），此补丁清零。若日后仍偏可重扫帧。 */
#define FOC_FRAME_CORR                0.0f   /* (rad) */

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
