/* FOC 应用层 — 组装 HAL、初始化核心、对齐/校准、启动 ISR
 * main.c 仅调用 foc_app_init() + foc_app_loop()
 * TIM6 中断回调调用 foc_app_isr()
 */

#include "foc_app.h"
#include "foc.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>

extern const hal_current_t  g_hal_current;
extern const hal_encoder_t  g_hal_encoder;
extern const hal_pwm_t      g_hal_pwm;

foc_core_t  g_core;
foc_comm_t  g_comm;
foc_can_t   g_can;

static foc_hal_t g_hal;

static const foc_motor_params_t g_motor = {
    .rs = 0.12f, .ld = 0.35e-3f, .lq = 0.35e-3f,
    .flux_linkage = 0.0072f, .pole_pairs = 14,
    .rated_current = 15.0f, .max_speed = 300.0f,
};

static void _uart_putchar(char c)
{
    while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) {}
    huart1.Instance->TDR = (uint8_t)c;
}

static uint8_t _uart_getchar(char *c)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        *c = (uint8_t)(huart1.Instance->RDR & 0xFF);
        return 1;
    }
    return 0;
}

int fputc(int ch, FILE *f)
{
    _uart_putchar((char)ch);
    return ch;
}

static void _do_alignment(void)
{
    foc_angle_t a0 = { .rad = 0, .sin = 0, .cos = 1 };
    foc_dq_t v0 = { .d = 3.0f, .q = 0 };
    foc_alphabeta_t ab0;
    foc_svpwm_t sv0;
    park_transform_inv(&v0, &a0, &ab0);
    svpwm_calc(&ab0, 16.8f, &sv0);
    g_hal_pwm.set_duty(sv0.ta, sv0.tb, sv0.tc);
    HAL_Delay(200);

    float raw_mech = g_hal_encoder.read_angle_rad();
    g_core.sense.encoder.offset_rad = raw_mech * g_motor.pole_pairs;
    printf("align: raw=%.3f off=%.3f\r\n", raw_mech, g_core.sense.encoder.offset_rad);

    g_hal_pwm.set_duty(0, 0, 0);
    HAL_Delay(100);
}

static void _do_calibration(void)
{
    foc_sense_current_calib(&g_core.sense.current, g_hal.current);
    printf("calib: u=%.4f v=%.4f w=%.4f\r\n",
           g_core.sense.current.offset.u,
           g_core.sense.current.offset.v,
           g_core.sense.current.offset.w);
}

void foc_app_init(void)
{
    g_hal.current = &g_hal_current;
    g_hal.encoder = &g_hal_encoder;
    g_hal.pwm     = &g_hal_pwm;

    foc_core_init(&g_core, &g_motor, &g_hal);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    if (g_hal.pwm->enable) g_hal.pwm->enable(1);

    _do_alignment();
    _do_calibration();

    foc_control_set_pid(&g_core.ctrl, FOC_CTRL_SPD, 0.18f, 0.5f, 0.0f);
    foc_control_set_pid(&g_core.ctrl, FOC_CTRL_IQ,  0.24f, 10.0f, 0.0f);

    foc_control_set_mode(&g_core.ctrl, FOC_MODE_SPEED);
    foc_control_set_target_spd(&g_core.ctrl, 100.0f);

    foc_core_enable(&g_core);
    g_core.state = FOC_STATE_RUN;

    comm_init(&g_comm, &g_core, _uart_putchar, _uart_getchar);

    TIM6->PSC = 169;
    TIM6->ARR = 99;
    TIM6->EGR = TIM_EGR_UG;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);
}

void foc_app_isr(void)
{
    foc_core_loop_current(&g_core);

    if (g_core.speed_div == 0) return;

    if (g_core.loop_count % g_core.speed_div == 0)
    {
        if (g_core.ctrl.mode == FOC_MODE_SPEED ||
            g_core.ctrl.mode == FOC_MODE_POSITION)
        {
            foc_core_loop_speed(&g_core);
        }
    }

    if (g_core.loop_count % (g_core.speed_div * g_core.pos_div) == 0)
    {
        if (g_core.ctrl.mode == FOC_MODE_POSITION)
        {
            foc_core_loop_position(&g_core);
        }
    }
}

void foc_app_loop(void)
{
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();

    if (now - last >= 50)
    {
        last = now;
        printf("%.1f,%.2f,%.2f\r\n",
               foc_core_get_speed(&g_core),
               foc_core_get_id(&g_core),
               foc_core_get_iq(&g_core));
    }

    comm_tick(&g_comm);
    __WFI();
}
