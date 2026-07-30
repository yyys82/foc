/**
 * @file hw_config.h
 * @brief Hardware configuration for STM32G431CBU6 FOC board
 *        Adapted from moteus r4.11 hardware to G431 custom board
 *
 * Hardware:
 *   MCU:       STM32G431CBU6 (170MHz, 128KB Flash, 32KB RAM)
 *   PWM:       TIM3 CH1(PA6), CH2(PA4), CH3(PB0), center-aligned 25kHz
 *   Current:   ADC1_IN4(PA3) + ADC2_IN14(PB11), injected, TIM3 TRGO trigger
 *   Sense:     10mΩ shunt + INA240A2 (gain=20)
 *   Driver EN: PA5 (U), PA7 (V), PB1 (W) - individual GPIO enables
 *   Encoder:   AS5600 via I2C2 (PB10=SCL, PB11=SDA) - NOTE: PB11 shared!
 *              Actually I2C2: PB10=SCL, PB11=SDA conflicts with ADC2_IN14(PB11)
 *              => Use I2C2 alternate: PA15=SCL? No. Use I2C1: PA13=SCL, PA14=SDA
 *              => Or remap. Check schematic. Using I2C2: PB13=SCL, PB14=SDA (alt4)
 *   CAN:       FDCAN1 PA12=TX, PB8=RX, Classic CAN 1Mbps
 *   Vbus:      None (fixed 16.8V assumed)
 *   Supply:    16.8V
 */

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "stm32g4xx_hal.h"

/*============================================================================
 * System Clock: HSI16 → PLL ×85/4 → 170MHz
 *============================================================================*/
#define SYS_CLOCK_HZ            170000000U
#define APB1_CLOCK_HZ           170000000U
#define APB2_CLOCK_HZ           170000000U

/*============================================================================
 * PWM: TIM3 center-aligned, 3 complementary channels
 *============================================================================*/
#define PWM_TIM                 TIM3
#define PWM_TIM_CLK_ENABLE()    __HAL_RCC_TIM3_CLK_ENABLE()
#define PWM_FREQUENCY_HZ        25000U
#define PWM_PERIOD              (SYS_CLOCK_HZ / (2U * PWM_FREQUENCY_HZ))  /* 3400 */

/* PWM GPIO: PA6=TIM3_CH1, PA4=TIM3_CH2, PB0=TIM3_CH3 */
#define PWM_GPIO_PORT_A         GPIOA
#define PWM_GPIO_PORT_B         GPIOB
#define PWM_PIN_U               GPIO_PIN_6    /* PA6 TIM3_CH1 */
#define PWM_PIN_V               GPIO_PIN_4    /* PA4 TIM3_CH2 */
#define PWM_PIN_W               GPIO_PIN_0    /* PB0 TIM3_CH3 */
#define PWM_GPIO_AF             GPIO_AF2_TIM3

/*============================================================================
 * ADC: Injected mode, triggered by TIM3 TRGO
 *   ADC1_IN4 = PA3  → Phase U current
 *   ADC2_IN14 = PB11 → Phase W current
 *============================================================================*/
#define ADC_U                   ADC1
#define ADC_U_CHANNEL           ADC_CHANNEL_4
#define ADC_U_GPIO_PORT         GPIOA
#define ADC_U_PIN               GPIO_PIN_3

#define ADC_W                   ADC2
#define ADC_W_CHANNEL           ADC_CHANNEL_14
#define ADC_W_GPIO_PORT         GPIOB
#define ADC_W_PIN               GPIO_PIN_11

/* Current sensing: 10mΩ shunt + INA240A2 gain=20
 * V_adc = I_phase * 0.010 * 20 = I_phase * 0.2
 * I_phase = V_adc / 0.2 = ADC_raw * 3.3 / 4096 / 0.2
 * I_phase = ADC_raw * 0.004028 A/count
 */
#define CURRENT_SENSE_GAIN      20.0f
#define SHUNT_RESISTANCE        0.010f
#define ADC_VREF                3.3f
#define ADC_RESOLUTION          4096.0f
#define CURRENT_ADC_SCALE       (ADC_VREF / (ADC_RESOLUTION * SHUNT_RESISTANCE * CURRENT_SENSE_GAIN))
/* = 3.3 / (4096 * 0.01 * 20) = 3.3 / 819.2 = 0.004028 A/count */

/* ADC offset (midpoint) for bidirectional current measurement */
#define ADC_OFFSET_NOMINAL      2048

/*============================================================================
 * Gate Driver Enable Pins
 *   PA5 = U enable, PA7 = V enable, PB1 = W enable
 *============================================================================*/
#define DRV_EN_U_PORT           GPIOA
#define DRV_EN_U_PIN            GPIO_PIN_5
#define DRV_EN_V_PORT           GPIOA
#define DRV_EN_V_PIN            GPIO_PIN_7
#define DRV_EN_W_PORT           GPIOB
#define DRV_EN_W_PIN            GPIO_PIN_1

/*============================================================================
 * Encoder: AS5600 via I2C
 *   I2C2: PB13=SCL, PB14=SDA (AF4)
 *   NOTE: Verify from schematic! Common AS5600 wiring uses I2C1.
 *   AS5600 12-bit absolute, 0-360° mapped to 0-4095
 *============================================================================*/
#define ENCODER_I2C             I2C2
#define ENCODER_I2C_CLK_ENABLE() __HAL_RCC_I2C2_CLK_ENABLE()
#define ENCODER_SCL_PORT        GPIOB
#define ENCODER_SCL_PIN         GPIO_PIN_13
#define ENCODER_SDA_PORT        GPIOB
#define ENCODER_SDA_PIN         GPIO_PIN_14
#define ENCODER_GPIO_AF         GPIO_AF4_I2C2
#define ENCODER_I2C_ADDR        (0x36U << 1)  /* AS5600 7-bit addr = 0x36 */
#define ENCODER_RESOLUTION      4096U         /* 12-bit */

/*============================================================================
 * FDCAN1: PA12=TX, PB8=RX, Classic CAN mode, 1Mbps
 *============================================================================*/
#define FDCAN_PERIPH            FDCAN1
#define FDCAN_CLK_ENABLE()      __HAL_RCC_FDCAN_CLK_ENABLE()
#define FDCAN_TX_PORT           GPIOA
#define FDCAN_TX_PIN            GPIO_PIN_12
#define FDCAN_RX_PORT           GPIOB
#define FDCAN_RX_PIN            GPIO_PIN_8
#define FDCAN_GPIO_AF           GPIO_AF9_FDCAN1

/*============================================================================
 * Motor Parameters (defaults, configurable via moteus registers)
 *============================================================================*/
#define MOTOR_POLE_PAIRS        7U            /* Typical for outrunner */
#define MOTOR_MAX_CURRENT_A     20.0f         /* Peak current limit */
#define MOTOR_NOMINAL_VOLTAGE   16.8f         /* Supply voltage (no Vbus sense) */
#define MOTOR_PHASE_RESISTANCE  0.1f          /* Ohms (estimate) */
#define MOTOR_PHASE_INDUCTANCE  0.0001f       /* Henry (estimate) */

/*============================================================================
 * Control Loop Frequencies
 *============================================================================*/
#define CURRENT_LOOP_FREQ_HZ    25000U        /* = PWM frequency */
#define VELOCITY_LOOP_FREQ_HZ   1000U
#define POSITION_LOOP_FREQ_HZ   1000U         /* moteus runs position at same rate */

/*============================================================================
 * moteus Protocol Constants
 *============================================================================*/
#define MOTEUS_CAN_ID           1U            /* Default device CAN ID */
#define MOTEUS_SOURCE_ID        0U            /* Host source ID */
#define MOTEUS_MODEL_NUMBER     0x00000001U   /* Custom model number */
#define MOTEUS_FW_VERSION       0x0100U       /* v1.0 */
#define MOTEUS_REG_MAP_VERSION  1U

#endif /* HW_CONFIG_H */
