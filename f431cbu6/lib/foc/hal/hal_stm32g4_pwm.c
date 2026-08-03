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
    /* 相序映射：ta→CH1(PA6/U)  tb→CH2(PA4/V)  tc→CH3(PB0/W)。
     * 原为 tb→CH3/tc→CH2（B/C 交换），扫帧测出 120° 帧误差、电流对齐却不产生转矩，
     * 换回正确相序使驱动与采样(PA3采V→ib、PB11采W→ic)一致。 */
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

static float _get_vbus(void) { return 16.8f; }

const hal_pwm_t g_hal_pwm = {
    .set_duty = _set_duty,
    .enable   = _enable,
    .get_vbus = _get_vbus,
};
