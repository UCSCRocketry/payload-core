#include "servo.h"
#include "stm32f4xx_hal_tim.h"
#include "log.h"

//! This function may modify htim in dev
int servo_init(struct servo_device *dev)
{
	uint32_t clk_freq = HAL_RCC_GetPCLK2Freq();
	TIM_HandleTypeDef *htim = dev->htim;
	TIM_OC_InitTypeDef sConfigOC = { 0 };


	float target_div = (float) clk_freq * (float) dev->period_us / 1000000.0f;
	// Iterate to find the optimal prescaler while maximizing period
	uint32_t period = 65535;
	uint32_t prescaler = 1;
	while (period * prescaler < target_div)
	{
		prescaler++;
	}
	while (period * prescaler > target_div)
	{
		period--;
	}

	// Configure timer
	htim->Init.Prescaler = prescaler - 1;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->Init.Period = period;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.RepetitionCounter = 0;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(htim) != HAL_OK)
	{
		return -1;
	}

	// Setup initial rotation @ center of possible PWM widths
	// For symmetric setups, this is 0 degrees, uncorrected.
	uint32_t pwm_center_us = (dev->pwm_max_us + dev->pwm_min_us) / 2;
	uint32_t pulse
	        = (int) ((float) pwm_center_us / ((float) dev->period_us / (float) htim->Init.Period));
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, dev->tim_channel) != HAL_OK)
	{
		return -1;
	}

	return 0;
}

int servo_start(struct servo_device *dev)
{
	TIM_HandleTypeDef *htim = dev->htim;
	return HAL_TIM_PWM_Start(htim, dev->tim_channel);
}

int servo_stop(struct servo_device *dev)
{
	TIM_HandleTypeDef *htim = dev->htim;
	return HAL_TIM_PWM_Stop(htim, dev->tim_channel);
}

int servo_set(struct servo_device *dev, const float pos_deg)
{
	float pos = pos_deg;
	float deg_min = dev->deg_min;
	float deg_max = dev->deg_max;

	// Clamp to valid range
	if (pos_deg < deg_min)
	{
		pos = deg_min;
	}
	if (pos_deg > deg_max)
	{
		pos = deg_max;
	}

	// Map angle linearly to PWM pulse width in us
	float t = (pos - deg_min) / (deg_max - deg_min);
	float pwm_us = dev->pwm_min_us + t * (float) (dev->pwm_max_us - dev->pwm_min_us);

	// Change timer compare register
	uint32_t period = dev->htim->Init.Period;
	uint32_t pulse = (uint32_t) (pwm_us * (float) period / (float) dev->period_us);
	__HAL_TIM_SET_COMPARE(dev->htim, dev->tim_channel, pulse);

	return 0;
}

int servo_read(struct servo_device *dev, float *servo_pos)
{
	/* Configure channel */
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = dev->adc_channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(dev->hadc, &sConfig) != HAL_OK)
	{
		return -1;
	}

	/* Convert the Channel */
	HAL_ADC_Start(dev->hadc);
	HAL_ADC_PollForConversion(dev->hadc, 100);
	uint16_t adcval = HAL_ADC_GetValue(dev->hadc);
	HAL_ADC_Stop(dev->hadc);
	// max val 3835
	// min val 182
	*servo_pos = 180.0 * (((float) adcval - 182.0) / (3835.0 - 182.0));

	return 0;
}