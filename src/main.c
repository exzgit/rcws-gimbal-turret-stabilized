/* Minimal test firmware
 * - Configures clocks, GPIO and TIM1 (implemented in system_stubs.c)
 * - Starts TIM1 CH1 PWM on PA8
 * - Sends low throttle (1000us) briefly for arming, then a slow throttle
 *   (~1600us) for 10 seconds to test brushless motor spin.
 */

#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim1;

/* Prototypes provided in system_stubs.c */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);

static void set_motor_speed_us(uint16_t us)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us);
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_TIM1_Init();

	/* Start PWM and enable main output */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_MOE_ENABLE(&htim1);

	/* Ensure low throttle initially */
	set_motor_speed_us(1000);
	HAL_Delay(200);

	/* ESC calibration sequence: max -> min, then ramp to test speed */
	set_motor_speed_us(2000); /* max */
	HAL_Delay(2000);

	set_motor_speed_us(1000); /* min */
	HAL_Delay(1000);

	/* Gentle ramp from 1000 -> 1600 over ~6 seconds */
	for (uint16_t v = 1000; v <= 1600; v += 5) {
		set_motor_speed_us(v);
		HAL_Delay(6);
	}

	/* Hold test throttle for 10 seconds */
	HAL_Delay(10000);

	/* Return to low throttle */
	set_motor_speed_us(1000);

	while (1) { HAL_Delay(1000); }
}

