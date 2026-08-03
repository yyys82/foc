/*
 * STM32G4 TIM3 三相 PWM (25kHz, 中心对齐 ARR=3399)
 * CH1=PA6(U)  CH2=PA4(V)  CH3=PB0(W)
 */
#include "hal/hal_pwm.h"
#include "tim.h"
#include "hw_config.h"

/* ARR = PWM_PERIOD-1 = 3399 → f = HCLK/(2*(ARR+1)) = 25kHz 精确 */
#define TIM_PERIOD ((float)(PWM_PERIOD - 1U))

static volatile uint8_t _enabled;   /* enable() 可被线程/电流环 ISR 调用，须 volatile */

/* 上电初始化（foc_main_init 调用）：
 * 驱动器使能脚先拉低关断 → 启动 3 路 PWM（TIM3 计数器开始，TRGO 产生 ADC 注入触发）
 * → 再拉高使能，保证桥臂在输出有确定状态后才上电，避免中间态直通。 */
static void _init(void)
{
    HAL_GPIO_WritePin(DRV_EN_U_PORT, DRV_EN_U_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_EN_V_PORT, DRV_EN_V_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_EN_W_PORT, DRV_EN_W_PIN, GPIO_PIN_RESET);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    HAL_GPIO_WritePin(DRV_EN_U_PORT, DRV_EN_U_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRV_EN_V_PORT, DRV_EN_V_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRV_EN_W_PORT, DRV_EN_W_PIN, GPIO_PIN_SET);

    _enabled = 1;   /* 通道已启动；后续 enable(1) 幂等，enable(0) 可正常关断 */
}

static void _set_duty(float ta, float tb, float tc)
{
    /* 相序映射：ta→CH1(PA6/U)  tb→CH2(PA4/V)  tc→CH3(PB0/W)。
     * 原为 tb→CH3/tc→CH2（B/C 交换），扫帧测出 120° 帧误差、电流对齐却不产生转矩，
     * 换回正确相序使驱动与采样(PA3采V→ib、PB11采W→ic)一致。 */
    /* 防御性夹到 [0,1]：svpwm 已夹，此处兜底防 CCR 越界（写超 ARR 恒 100% 占空比） */
    if (ta < 0.0f) ta = 0.0f; else if (ta > 1.0f) ta = 1.0f;
    if (tb < 0.0f) tb = 0.0f; else if (tb > 1.0f) tb = 1.0f;
    if (tc < 0.0f) tc = 0.0f; else if (tc > 1.0f) tc = 1.0f;

    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (uint16_t)(ta * TIM_PERIOD));
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, (uint16_t)(tb * TIM_PERIOD));
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, (uint16_t)(tc * TIM_PERIOD));
}

static void _enable(uint8_t en)
{
    if (en && !_enabled)
    {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
        _enabled = 1;
    }
    else if (!en && _enabled)
    {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
        _enabled = 0;
    }
}

static float _get_vbus(void) { return MOTOR_NOMINAL_VOLTAGE; }

const hal_pwm_t g_hal_pwm = {
    .init     = _init,
    .set_duty = _set_duty,
    .enable   = _enable,
    .get_vbus = _get_vbus,
};
