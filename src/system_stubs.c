#include "stm32f4xx_hal.h"

/* Provide prescaler tables used by HAL RCC helpers */
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

/* Configure system clock to 84 MHz using HSI -> PLL (sane default for STM32F4) */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; /* 16/16*336/4 = 84 MHz */
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK
															|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	/* Ensure SystemCoreClock is updated after changing clock config */
	SystemCoreClockUpdate();
}

/* Configure GPIO pins used for TIM1 CH1/CH2 (PA8/PA9) as AF push-pull */
void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Enable GPIOA clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/* Enable GPIOB clock for debug pin */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* PA8 -> TIM1_CH1, PA9 -> TIM1_CH2 (AF1) */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* PB3 as debug output (toggle to verify firmware running) */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* Configure TIM1 for PWM: 1 us tick, period 20000 -> 20 ms (50 Hz) */
void MX_TIM1_Init(void)
{
	extern TIM_HandleTypeDef htim1; /* use global handle declared in main.c */
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* Enable TIM1 clock */
	__HAL_RCC_TIM1_CLK_ENABLE();

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = (uint32_t)(SystemCoreClock / 1000000U) - 1U; /* 1 MHz timer -> 1 us */
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 20000U - 1U; /* 20 ms */
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	HAL_TIM_PWM_Init(&htim1);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1500; /* neutral 1500 us */
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

	/* Do not start PWM or enable MOE here — main.c starts PWM and enables MOE
	   using the global `htim1` so the same handle is used across the codebase. */
}
