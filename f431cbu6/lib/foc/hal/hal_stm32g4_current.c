/*
 * STM32G4 INA240 双电阻电流 — 双 ADC 注入同步 + 中断读取
 *   TIM3 OC4REF → TRGO → ADC1(主)/ADC2(从) 注入同步转换 (INJECSIMULT) → 主机 JEOC 中断
 *   每 PWM 周期单入口，中断里直接读两个 JDR1，电流环就在本中断内执行（25kHz = PWM 频率）
 *   INA240A2: gain=50V/V, Rs=0.01Ω, Vref=1.65V → 0.5V/A，满量程约 ±3.0~3.3A
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

    /* 双 ADC 注入同步 (ADC_DUALMODE_INJECSIMULT)：
     * 启动顺序先从机(ADC2)后主机(ADC1)。从机在同步模式下只使能、不启动，
     * 跟随主机；主机 InjectedStart 把 JADSTART arm 到下一个 T3_TRGO 触发。
     * 规则组不用（vbus 为常量），故不再 HAL_ADC_Start。 */
    HAL_ADCEx_InjectedStart(&hadc2);   /* 从机：使能，跟随主机 */
    HAL_ADCEx_InjectedStart(&hadc1);   /* 主机：arm 注入转换 */

    /* 只开主机(ADC1)的注入完成中断：同步模式下两机同时完成，主机 JEOC 到来时
     * 从机 JDR1 必然已就绪，ISR 里一次读两个 JDR1。从机 JEOCIE 保持关闭 →
     * 每个 PWM 周期单入口（切勿再开 ADC2 JEOCIE 或 JEOSIE，否则双入口倍频）。 */
    ADC2->IER &= ~ADC_IER_JEOCIE;      /* 显式保险 */
    ADC1->IER |= ADC_IER_JEOCIE;
}

static void _read(float *ia, float *ib, float *ic)
{
    /* 注入同步下两机同刻采样、同刻完成；在本(主机 JEOC)中断里读两个 JDR1 均为本拍新值。
     * 不变量：从机(ADC2)采样时间/序列长度 不得大于 主机(ADC1)，否则从机晚完成、
     * 主机 JEOC 会读到从机上一拍 JDR。当前两边同为 12.5cyc、JL=0，满足。 */
    uint32_t v1 = (uint16_t)(ADC1->JDR1 & 0xFFFF);   /* ADC1=PA3，采 V 相 → ib */
    uint32_t v2 = (uint16_t)(ADC2->JDR1 & 0xFFFF);   /* ADC2=PB11，采 W 相 → ic */

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
