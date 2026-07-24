/* STM32G431 FOC — 应用层
 * TIM3  -> PWM (50kHz)
 * TIM6  -> 电流环 ISR (10kHz)
 * USART1 -> printf + 上位机
 * FDCAN1 -> CAN 控制
 */
#include "foc.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>
extern const hal_current_t g_hal_current;
extern const hal_encoder_t g_hal_encoder;
extern const hal_pwm_t     g_hal_pwm;
extern const hal_uart_t    g_hal_uart;
extern const hal_can_t     g_hal_can;
extern uint8_t g_openloop;

foc_core_t  g_core;
foc_comm_t  g_comm;
foc_can_t   g_can;
volatile uint32_t g_tim6_count;

static const foc_motor_params_t g_motor = {
    .rs = 0.12f, .ld = 0.35e-3f, .lq = 0.35e-3f,
    .flux_linkage = 0.0072f, .pole_pairs = 14,
    .rated_current = 15.0f, .max_speed = 300.0f,
};

int fputc(int ch, FILE *f)
{
    g_hal_uart.tx((char)ch);
    return ch;
}

static void _do_alignment(void)
{
    /* 旋转对齐: 用旋转磁场强制拉到确定位置 */
    float angle = 0;
    for (int i = 0; i < 200; i++)
    {
        foc_angle_t a = { .rad = angle, .sin = sinf(angle), .cos = cosf(angle) };
        foc_dq_t v = { .d = 4.0f, .q = 0 };
        foc_alphabeta_t ab;
        foc_svpwm_t sv;
        park_transform_inv(&v, &a, &ab);
        svpwm_calc(&ab, 16.8f, &sv);
        g_hal_pwm.set_duty(sv.ta, sv.tb, sv.tc);
        HAL_Delay(10);
        angle += 0.05f;
    }

    float raw = g_hal_encoder.get_angle();
    g_core.sense.encoder.offset_rad = raw * g_motor.pole_pairs;
    printf("align: raw=%.3f off=%.3f\r\n", raw, g_core.sense.encoder.offset_rad);

    g_hal_pwm.set_duty(0, 0, 0);
    HAL_Delay(100);
}


static void _do_calibration(void)
{
    foc_sense_current_calib(&g_core.sense.current, &g_hal_current);
    printf("calib: u=%.4f v=%.4f w=%.4f\r\n",
           g_core.sense.current.offset.u,
           g_core.sense.current.offset.v,
           g_core.sense.current.offset.w);
}

/* ISR: 10kHz 电流环, 分频执行速速/位置环 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;

    /* 速度环 (TIM6 1kHz) */
    if (g_core.ctrl.mode == FOC_MODE_SPEED ||
        g_core.ctrl.mode == FOC_MODE_POSITION)
    {
        foc_core_loop_speed(&g_core);
    }

    /* 位置环 200Hz (每 5 次速度环) */
    static uint32_t pos_cnt;
    pos_cnt++;
    if (pos_cnt >= 5)
    {
        pos_cnt = 0;
        if (g_core.ctrl.mode == FOC_MODE_POSITION)
        {
            foc_core_loop_position(&g_core);
        }
    }
}

void foc_main_init(void)
{
    g_openloop = 0;  /* 编码器模式，对齐修复后 */

    /* HAL 初始化 */
    g_hal_current.init();

    /* FOC 核心 */
    foc_core_init(&g_core, &g_motor,
                  &g_hal_current, &g_hal_encoder, &g_hal_pwm);

    /* PWM 使能 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    g_hal_pwm.enable(1);

    /* 禁用 ADC DMA 中断 */
    HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);

    /* 对齐 (ADC/I2C 中断均未开，确保编码器阻塞读不受干扰) */
    _do_alignment();

    /* ADC1/2 注入中断 + I2C2 中断 (对齐后同时开启) */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

    /* 清空编码器 DMA 管线，吃掉对齐时残留的脏数据 */
    for (int i = 0; i < 10; i++)
    {
        g_hal_encoder.get_angle();
        HAL_Delay(1);
    }

    /* 校准 */
    _do_calibration();

    /* 启动前重置全部 PI (清除残余积分) */
    foc_control_reset_all(&g_core.ctrl);

    /* PID */
    foc_control_set_pid(&g_core.ctrl, FOC_CTRL_SPD, 0.01f, 0.02f, 0.0f);
    foc_control_set_pid(&g_core.ctrl, FOC_CTRL_IQ,  0.8f, 0.9f, 0.0f);

    /* 速度模式 */
    foc_control_set_mode(&g_core.ctrl, FOC_MODE_SPEED);
    foc_control_set_target_spd(&g_core.ctrl, 20.0f);

    /* 扭矩模式 */
    foc_control_set_mode(&g_core.ctrl, FOC_MODE_TORQUE);
    foc_control_set_target_iq(&g_core.ctrl, 0.0f);

    /* 启动 */
    foc_core_enable(&g_core);
    g_core.state = FOC_STATE_RUN;

    /* 通信 */
    comm_init(&g_comm, &g_core, &g_hal_uart);
    can_protocol_init(&g_can, &g_core, &g_hal_can, 1);

    /* TIM6 1kHz 速度环 */
    TIM6->PSC = 169;
    TIM6->ARR = 999;
    TIM6->EGR = TIM_EGR_UG;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);

    /* DWT */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= 1;
    DWT->CYCCNT = 0;
}

void foc_main_loop(void)
{
    static uint32_t last;
    uint32_t now = HAL_GetTick();

    if (now - last >= 50)
    {
        last = now;
        float spd = foc_core_get_speed(&g_core);
        float tsp = g_core.ctrl.target_spd;
        float iq  = foc_core_get_iq(&g_core);
        float tiq = g_core.ctrl.target_iq;
        printf("%.2f,%.2f,%.2f,%.2f\r\n", spd, tsp, iq, tiq);
    }

    comm_tick(&g_comm);
    __WFI();
}
