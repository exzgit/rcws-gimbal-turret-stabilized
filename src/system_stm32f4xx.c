#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

uint32_t SystemCoreClock = 16000000;

void SystemInit(void)
{
    /* enable fpu */
    (*(volatile uint32_t *)0xE000ED88) |= (0xF << 20);
}

/* Provide a minimal SystemCoreClockUpdate implementation that queries HAL */
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = HAL_RCC_GetHCLKFreq();
}
