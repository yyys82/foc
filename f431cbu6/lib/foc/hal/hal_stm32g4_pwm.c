/*
 * STM32G4 TIM3 三相 PWM (25kHz, 中心对齐 ARR=3399)
 * CH1=PA6(U)  CH2=PA4(V)  CH3=PB0(W)
 */
#include "hal/hal_pwm.h"
#include "tim.h"

#define TIM_PERIOD 3399.f

static uint8_t _enabled;

static void _set_duty(float ta, float tb, float tc)
{
    /* 相序映射（与 ADC 采样一致，勿改）：
     *   ta→CH1(PA6)  tb→CH3(PB0)  tc→CH2(PA4)
     *   ADC: PA3(IN4)采 PA4 分流→ib，PB11(IN14)采 PB0 分流→ic，ia=-(ib+ic) */
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (uint16_t)(ta * TIM_PERIOD));
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, (uint16_t)(tb * TIM_PERIOD));
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, (uint16_t)(tc * TIM_PERIOD));
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

static float _get_vbus(void) { return 16.8f; }

const hal_pwm_t g_hal_pwm = {
    .set_duty = _set_duty,
    .enable   = _enable,
    .get_vbus = _get_vbus,
};
