/*
 * STM32G4 INA240 双电阻电流 — 注入 TRGO 硬件触发 + 中断读取
 *   TIM3 OC4REF → TRGO → ADC1/2 Injected → JEOC中断
 *   中断里读 JDR1 存全局变量，TIM6 ISR 跑电流环
 *   INA240: gain=50V/V, Rs=0.01Ω, Vref=1.65V
 */
#include "hal/hal_current.h"
#include "adc.h"

volatile uint32_t g_adc1_jdr;
volatile uint32_t g_adc2_jdr;
volatile uint8_t  g_adc_fresh;

static void _init(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);

    ADC1->IER |= ADC_IER_JEOCIE;
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
