/*
 * STM32G4 INA240 双电阻电流 — 双 ADC 注入同步 + 中断读取
 *   TIM3 OC4REF → TRGO → ADC1(主)/ADC2(从) 注入同步转换 (INJECSIMULT) → 主机 JEOC 中断
 *   每 PWM 周期单入口，中断里直接读两个 JDR1，电流环就在本中断内执行（25kHz = PWM 频率）
 *   INA240A2: gain=50V/V, Rs=0.01Ω, Vref=1.65V → 0.5V/A，满量程约 ±3.0~3.3A
 */
#include "hal/hal_current.h"
#include "adc.h"
#include "hw_config.h"

/* i = raw*CURRENT_ADC_SCALE - CURRENT_ADC_OFFSET_A，编译期折叠为每相 1 乘 1 减
 * (原实现 raw→V→(V-1.65)/0.5 多步浮点，25kHz ISR 内冗余) */
#define CUR_SCALE_A_CNT  CURRENT_ADC_SCALE      /* ≈ 0.001611 A/count */
#define CUR_OFFSET_A     CURRENT_ADC_OFFSET_A   /* = 3.3 A */

static void _init(void)
{
    /* ADC 自校准：须在 ADC 使能前执行（HAL 内部会先关 ADC）。
     * G4 上电默认校准因子未加载，不校准会引入额外失调/增益误差，
     * 对靠减 1.65V 零点测双向小电流的方案影响最明显。 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    /* 独立模式：两 ADC 各自 T3_TRGO 触发注入，启动顺序无关。
     * (双注入同步 INJECSIMULT 下从机 ADC2 运行中不转换 → ic 恒定，已改回独立模式。) */
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);

    /* 只开主机(ADC1) JEOC 中断：两机同一 TRGO 触发、同刻完成，
     * 主机 JEOC 到来时从机 JDR 也已就绪，ISR 一次读两个 JDR1。
     * 从机 JEOCIE 保持关闭 → 每 PWM 周期单入口（勿再开 ADC2 JEOCIE/JEOSIE）。 */
    ADC2->IER &= ~ADC_IER_JEOCIE;
    ADC1->IER |= ADC_IER_JEOCIE;
}

static void _read(float *ia, float *ib, float *ic)
{
    /* 注入同步下两机同刻采样、同刻完成；在本(主机 JEOC)中断里读两个 JDR1 均为本拍新值。
     * 不变量：从机(ADC2)采样时间/序列长度 不得大于 主机(ADC1)，否则从机晚完成、
     * 主机 JEOC 会读到从机上一拍 JDR。当前两边同为 12.5cyc、JL=0，满足。 */
    float i_b = (float)(ADC1->JDR1 & 0xFFF) * CUR_SCALE_A_CNT - CUR_OFFSET_A;  /* ADC1=PA3 → V 相 → ib */
    float i_c = (float)(ADC2->JDR1 & 0xFFF) * CUR_SCALE_A_CNT - CUR_OFFSET_A;  /* ADC2=PB11 → W 相 → ic */

    *ia = -(i_b) - (i_c);
    *ib = i_b;
    *ic = i_c;
}

static void _delay_us(uint32_t us)
{
    /* DWT 周期计数精确延时（替代原 NOP 空转 + 魔法常数 170）：
     * 比循环更准、不受 CPU 流水/优化影响；中断抢占只会拉长延时，校准平均无碍。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= 1;
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SYS_CLOCK_HZ / 1000000U);
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {}
}

const hal_current_t g_hal_current = {
    .init     = _init,
    .read     = _read,
    .delay_us = _delay_us,
};
