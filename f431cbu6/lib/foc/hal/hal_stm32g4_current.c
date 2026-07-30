/*
 * STM32G4 INA240 双电阻电流 — 注入 TRGO 硬件触发 + 中断读取
 *   TIM3 OC4REF → TRGO → ADC1/2 Injected → JEOC中断
 *   中断里直接读 JDR1，电流环就在本中断内执行（25kHz，每2次ADC触发）
 *   INA240: gain=50V/V, Rs=0.01Ω, Vref=1.65V
 */
#include "hal/hal_current.h"
#include "adc.h"

static void _init(void)
{
    /* ADC 自校准：须在 ADC 使能前执行（HAL 内部会先关 ADC）。
     * G4 上电默认校准因子未加载，不校准会引入额外失调/增益误差，
     * 对靠减 1.65V 零点测双向小电流的方案影响最明显。 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);

    /* 只开 ADC2 的注入完成中断。ADC1/ADC2 由同一 TRGO 同步触发、采样时间相同，
     * ADC2 的 JEOC 到来时 ADC1 必然也已完成 → 每个 PWM 周期恰好进一次
     * ADC1_2_IRQHandler，在 ISR 里一次性读两个 JDR1。避免了两个 JEOC 先后置位
     * 导致共享中断双入口、需要 &1 分频的脆弱时序。电流环触发率 = PWM 频率。 */
    ADC2->IER |= ADC_IER_JEOCIE;
}

static void _read(float *ia, float *ib, float *ic)
{
    uint32_t v1 = (uint16_t)(ADC1->JDR1 & 0xFFFF);
    uint32_t v2 = (uint16_t)(ADC2->JDR1 & 0xFFFF);

    float va = (float)v1 * 3.3f / 4096.0f;
    float vb = (float)v2 * 3.3f / 4096.0f;

    float i_b = (va - 1.65f) / (50.0f * 0.01f);
    float i_c = (vb - 1.65f) / (50.0f * 0.01f);
    *ia = -(i_b) - (i_c);
    *ib = i_b;
    *ic = i_c;
}

static void _delay_us(uint32_t us)
{
    uint32_t ticks = us * 170;
    for (uint32_t i = 0; i < ticks; i++) __NOP();
}

const hal_current_t g_hal_current = {
    .init     = _init,
    .read     = _read,
    .delay_us = _delay_us,
};
