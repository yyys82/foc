/**
 * @file system_stm32g4xx.c
 * @brief Minimal SystemInit for STM32G431 (GCC)
 *        Sets vector table offset; clock config done in main.c via HAL
 */

#include "stm32g4xx.h"

/* Vector table in Flash */
uint32_t SystemCoreClock = 16000000U;  /* HSI16 default, updated after PLL */

const uint8_t AHBPrescTable[16] = {0,0,0,0,0,0,0,0,1,2,3,4,6,7,8,9};
const uint8_t APBPrescTable[8]  = {0,0,0,0,1,2,3,4};

void SystemInit(void)
{
  /* Set VTOR to Flash base */
  SCB->VTOR = FLASH_BASE;

  /* FPU is enabled in startup .s, nothing else needed here */
}
