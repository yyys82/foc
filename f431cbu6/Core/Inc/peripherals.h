/**
 * @file peripherals.h
 * @brief Hardware peripheral initialization for STM32G431 FOC board
 *        System clock, TIM3 PWM, ADC1/2 injected, I2C2
 */

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "stm32g4xx_hal.h"
#include "hw_config.h"

/* Handles (extern, defined in peripherals.c) */
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/**
 * System clock: HSI16 → PLL → 170MHz
 */
void SystemClock_Config(void);

/**
 * TIM3: center-aligned PWM, 3 channels + CH4 for ADC trigger
 * 25kHz center-aligned, ARR = 3400
 */
void PWM_Init(void);

/**
 * ADC1 + ADC2: injected mode, triggered by TIM3 TRGO
 * ADC1_IN4 (PA3) = Phase U current
 * ADC2_IN14 (PB11) = Phase W current
 */
void ADC_Init(void);

/**
 * I2C2: PB13=SCL, PB14=SDA, 400kHz fast mode
 * For AS5600 encoder
 */
void I2C_Init(void);

/**
 * GPIO: driver enable pins PA5, PA7, PB1
 */
void GPIO_Init(void);

/**
 * Start PWM output and ADC injected conversions
 */
void Peripherals_Start(void);

#endif /* PERIPHERALS_H */
