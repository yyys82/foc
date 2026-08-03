/* STM32G431 FOC — 应用层
 * TIM3  -> PWM (25kHz, 中心对齐 ARR=3399)
 * TIM6  -> 速度环/位置环 ISR (1kHz / 200Hz)；电流环在 ADC 注入中断 (25kHz)
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
extern float   g_dbg_mech;
extern void    encoder_try_recovery(void);   /* hal_stm32g4_encoder.c：线程态 I2C 卡死恢复 */

foc_core_t  g_core;
foc_comm_t  g_comm;
foc_can_t   g_can;

static const foc_motor_params_t g_motor = {
    .rs = 0.12f, .ld = 0.35e-3f, .lq = 0.35e-3f,
    .flux_linkage = 0.0072f, .pole_pairs = 14,
    .rated_current = 2.0f, .max_speed = 300.0f,   /* 过流故障阈值 = rated*1.5 = 3.0A（贴 ADC 饱和上沿）；Rs=10mΩ+INA240A2(50V/V)=0.5V/A，ADC 约 ±3.0~3.3A 饱和 */
};

/* ===== 调试开关 ===== */
#define IQ_DBG_PRINT_MS 5       /* 打印周期 (ms)，200Hz @115200 */
#define SPD_DBG_TARGET  120.0f   /* 速度环目标 (rad/s)，调试用；调完改 0 或上位机给 */
#define POS_DBG_TARGET  10.0f   /* 位置环目标 (mech rad)，调试用 */

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
    /* 修复：旋转磁场结束在 elec=angle(=10.0rad ≡ 3.67rad≈210°)，转子磁通真实电角度=3.67；
     * 若按"转子停在 elec=0"存 offset(=raw*pp)，会引入 ~210° 固定电角度误差，
     * 使 Park 帧旋转 210° → iq_fb 反号(正反馈) + 电流灌错轴(无转矩/抖动)。
     * 修正：offset = raw*pp - angle，使 elec=0 对应磁通真实位置。 */
    g_core.sense.encoder.offset_rad = raw * g_motor.pole_pairs - angle;
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

/* TIM6 ISR: 速度环 1kHz / 位置环 200Hz（电流环在 ADC 注入中断，25kHz） */
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
    /* DWT 时基：须在编码器/电流初始化之前使能，保证时间戳基准一致
     * （原先放在函数末尾，导致编码器首次时间戳基于复位前的 CYCCNT，dt 算成 ~25s → 速度估算爆掉） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= 1;
    DWT->CYCCNT = 0;

    /* 编码器 I2C DMA 完成回调(DMA1_Channel1)是角度发布端，提到优先级 1：
     * 让电流环 ADC ISR(prio0) 能抢占它，发布端(~µs 级)不再整体顺延电流环 → 抖动归零。
     * (dma.c 里 MX_DMA_Init 默认设 0；此处覆盖，放 init 最前以早于对齐阶段首次启动 DMA) */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);

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
    foc_control_set_pid(&g_core.ctrl, FOC_CTRL_SPD, 0.03f, 0.02f, 0.0f);  /* 速度环：长测速窗(256)平滑反馈；低增益防低速测速噪声驱动振荡 */
    /* 电流环：L=0.35mH, R=0.12Ω, ωc=1000 → kp=0.35, ki=120。
     * 实测 0.35/120 跟踪最佳(0.2A→iq 0.20)；0.6/200 失稳跟踪变差。 */
    foc_control_set_pid(&g_core.ctrl, FOC_CTRL_ID,  0.35f, 120.0f, 0.0f);
    foc_control_set_pid(&g_core.ctrl, FOC_CTRL_IQ,  0.35f, 120.0f, 0.0f);
    foc_control_set_pid(&g_core.ctrl, FOC_CTRL_POS, 1.0f, 0.00f, 0.0f);  /* 位置环：纯P，低增益防保持振荡 */

    /* 复位后默认安全静止：IDLE + 断电，等待上位机 enable / mode / target 命令
     * 不调用 foc_core_enable → core->enable=0，电流环每个中断都会 set_duty(0,0,0) 清零，
     * 电机彻底无输出（零矢量）。需要转动时用上位机发 enable + mode spd + target。 */
//    foc_control_set_mode(&g_core.ctrl, FOC_MODE_SPEED);
//    foc_control_set_target_spd(&g_core.ctrl, 120.0f);

    /* 通信 */
    comm_init(&g_comm, &g_core, &g_hal_uart);
    g_hal_uart.init(115200);   /* 开启 USART1 RX 中断 + 接收环形缓冲，命令不再丢字节 */
    can_protocol_init(&g_can, &g_core, &g_hal_can, 1);

    /* TIM6 1kHz 速度环 */
    TIM6->PSC = 169;
    TIM6->ARR = 999;
    TIM6->EGR = TIM_EGR_UG;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);

    /* 上电状态横幅：直接走 UART(printf/uart->tx)，网页连接即可看到初始 STATUS。
     * 此时 core 已初始化为 IDLE + 断电（见上方默认安全静止段）。 */
    comm_banner(&g_comm);
}

void foc_main_loop(void)
{
    static uint32_t last;
    uint32_t now = HAL_GetTick();

//    /* 打印诊断：位置, 目标位置, 速度, 目标速度, iq实测, iq目标, id, vq */
//    if (now - last >= IQ_DBG_PRINT_MS)
//    {
//        last = now;
//        float pos  = g_dbg_mech;
//        float tpos = g_core.ctrl.target_pos;
//        float spd  = foc_core_get_speed(&g_core);
//        float tspd = g_core.ctrl.target_spd;
//        float iq   = foc_core_get_iq(&g_core);
//        float tiq  = g_core.ctrl.target_iq;
//        float id   = foc_core_get_id(&g_core);
//        float vq   = g_core.ctrl.vq;
//        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", pos, tpos, spd, tspd, iq, tiq, id, vq);
//    }

    encoder_try_recovery();   /* 线程态处理编码器 I2C 卡死恢复（电流环 ISR 只置标志，不阻塞） */
    comm_tick(&g_comm);
    __WFI();
}
