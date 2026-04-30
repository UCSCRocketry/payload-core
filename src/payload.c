/**
 * @file payload.c
 * @brief Payload state machine orchestrator.
 */

#include "payload.h"
#include "sensor_io.h"
#include "prebuf.h"
#include "dump.h"
#include "spif.h"
#include "../lib/bmp388/bmp388.h"
#include "sensor.h"
#include "main.h"
#include "log.h"
#include "stm32f4xx_hal_gpio.h"
#include <string.h>
#include <stdbool.h>
#include "stm32f4xx_hal_tim.h"

_Static_assert(sizeof(struct payload_page) == 256, "payload_page must be exactly 256 bytes");

/* External Variables -----------------------------------------------*/
extern SPI_HandleTypeDef hspi1; // SPI peripheral for SPIF
extern SPI_HandleTypeDef hspi3; // SPI peripheral for Altimeter/BMP
extern SPI_HandleTypeDef hspi4; // SPI peripheral for IMU

extern TIM_HandleTypeDef htim1; // Timer 1 peripheral (PWM)
extern TIM_HandleTypeDef htim2; // Timer 2 peripheral (Logging)
extern TIM_HandleTypeDef htim3; // Timer 3 peripheral (LED)
extern TIM_HandleTypeDef htim4; // Timer 4 peripheral (Servo)

extern SPIF_HandleTypeDef hspif; // SPI flash device

extern enum led_state_e led_state;

/* Private Variables -----------------------------------------------*/
static bool is_initialized = false;

static volatile enum payload_state_e payload_state = PAYLOAD_STATE_INIT;

struct prebuf pb = { 0 };
static float baseline_pressure = 0.0f;
static uint32_t current_page_idx = 0;
static struct payload_page recording_page;
static uint32_t page_sample_idx = 0;
static float peak_altitude = 0.0f;
static uint32_t land_hold_count = 0;

// Sets up the payload system
void payload_run(void)
{
	// Press button to dump
	if (button_pressed())
	{
		for (uint8_t i = 0; i < 10; i++)
		{
			if (!button_pressed())
			{
				LOG_INF("Startup button held - dumping flash to SD card...");
				led_state = LED_BLINK_FAST;
				if (!dump_and_format_flash(&hspif))
				{
					LOG_INF("Dump OK");
				}
				else
				{
					LOG_ERR("Dump failed - flash NOT erased");
				}
				led_state = LED_OFF;
				goto cancel_erase;
			}
			
			// Delay for 1 seconds - 10 seconds total
			HAL_Delay(1000);
			LOG_INF("Held for %u/10 seconds till erase (RELEASE to dump flash)", i + 1);
		}

		LOG_INF("Erasing SPIF...");
		if (!SPIF_EraseChip(&hspif))
		{
			LOG_ERR("SPIF Erase: flash chip erase failed");
		}
		LOG_INF("SPIF Erase: flash erased. Done.");

	}
	cancel_erase:
	

	// Wait 5 seconds as a buffer between dump sensitive and arm sensitive button
	LOG_INF("Waiting 5 s...");
	HAL_Delay(5000);

	// Press button to arm
	while (!button_pressed())
	{
		HAL_Delay(10);
	}
	LOG_INF("Armed.");
	led_state = LED_BLINK_SLOW;

	// Initialize sensors
	if (sensor_io_init(&hspi3, &hspi4) != 0)
	{
		Error_Handler();
	}
	is_initialized = true;

	// Take baseline pressure
	LOG_DBG("Sampling baseline pressure (%d readings)...", PAYLOAD_BASELINE_SAMPLES);
	baseline_pressure = sensor_io_press_baseline(PAYLOAD_BASELINE_SAMPLES);
	LOG_DBG("Baseline: %ld.%03ld kPa", (int32_t) baseline_pressure,
	        (int32_t) ((baseline_pressure - (int32_t) baseline_pressure) * 1000.0f));

	LOG_INF("Pre-launch buffer active. Threshold: %d m", (int) PAYLOAD_LAUNCH_ALT_THRESHOLD_M);

	payload_state = PAYLOAD_STATE_PRELAUNCH;
	return;
}

void payload_handle_LED(void)
{
	switch (led_state)
	{
	case LED_ON:
		HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_RESET);
		break;
	case LED_BLINK_SLOW:
		__HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFF);
		HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);
		break;
	case LED_BLINK_FAST:
		__HAL_TIM_SET_AUTORELOAD(&htim3, 0x5555);
		HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);
		break;
	default: // LED_OFF
		HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_SET);
		break;
	}
	return;
}

void payload_handle_prelog(void)
{
	struct payload_sample s = { 0 };
	if (sensor_io_sample(&s) == 0)
	{
		prebuf_push(&pb, &s);
		struct sensor_value cur = { .val1 = s.pressure_v1, .val2 = s.pressure_v2 };
		float alt = bmp388_calc_altitude(baseline_pressure, sensor_value_to_float(&cur));

#ifdef __PAYLOAD_TESTING__
		int launched = (alt >= PAYLOAD_LAUNCH_ALT_THRESHOLD_M
		                || (HAL_GPIO_ReadPin(PAYLOAD_BTN_GPIO_PORT, PAYLOAD_BTN_GPIO_PIN)
		                    == GPIO_PIN_RESET));
#else
		int launched = (alt >= PAYLOAD_LAUNCH_ALT_THRESHOLD_M);
#endif
		if (launched)
		{
			LOG_INF("Launch detected! Alt ~%d m", (int) alt);
			current_page_idx = prebuf_flush(&pb, &hspif);

			led_state = LED_ON;
			LOG_DBG("Recording: starting at page %lu / %lu", current_page_idx, hspif.PageCnt);
			memset(&recording_page, 0xFF, sizeof(recording_page));
			page_sample_idx = 0;
			payload_state = PAYLOAD_STATE_ASCEND;
		}
	}
	return;
}

void payload_handle_log(void)
{

	if (current_page_idx < hspif.PageCnt)
	{
		if(current_page_idx % 25 == 0) LOG_INF("Recording page %lu", current_page_idx);

		struct payload_sample s = { 0 };
		if (sensor_io_sample(&s) == 0)
		{
			recording_page.samples[page_sample_idx++] = s;

			if (page_sample_idx == PAYLOAD_SAMPLES_PER_PAGE)
			{
				if (!SPIF_WritePage(&hspif, current_page_idx, (uint8_t *) &recording_page,
				                    sizeof(recording_page), 0))
				{
					LOG_ERR("Flash write error at page %lu", current_page_idx);
				}
				current_page_idx++;
				page_sample_idx = 0;
				memset(&recording_page, 0xFF, sizeof(recording_page));
			}

            struct sensor_value cur = { .val1 = s.pressure_v1, .val2 = s.pressure_v2 };
            float alt = bmp388_calc_altitude(baseline_pressure, sensor_value_to_float(&cur));

            if(current_page_idx % 10 == 0) {
                LOG_INF("Altitude: %ld", (int32_t) alt);
            }

            if (payload_state == PAYLOAD_STATE_ASCEND) {
                if (alt > peak_altitude) {
                    peak_altitude = alt;
                } else if (alt < peak_altitude - PAYLOAD_APOGEE_MARGIN_M) {
                    LOG_INF("Apogee detected! Peak alt ~%d m. Transitioning to DESCEND.", (int)peak_altitude);
                    payload_state = PAYLOAD_STATE_DESCEND;
                }
            } else if (payload_state == PAYLOAD_STATE_DESCEND) {
                if (alt <= PAYLOAD_LAND_ALT_THRESHOLD_M) {
                    land_hold_count++;
                } else {
                    land_hold_count = 0;
                }

                if (land_hold_count >= PAYLOAD_LAND_HOLD_SAMPLES) {
                    LOG_INF("Landing detected!");
                    
                    if (page_sample_idx > 0) {
                        if (!SPIF_WritePage(&hspif, current_page_idx, (uint8_t *)&recording_page, sizeof(recording_page), 0)) {
                            LOG_ERR("Flash write error at page %lu", current_page_idx);
                        }
                        
                        current_page_idx++;
                    }
                    
                    payload_terminate_recording();
                }
            }
		}
	}
	else
	{
		payload_terminate_recording();
	}

	return;
}

void payload_terminate_recording(void)
{
    LOG_INF("Recording terminated. Wrote %lu pages. Entering low-power stop mode.", current_page_idx);
    led_state = LED_OFF;
    HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_SET);
    payload_state = PAYLOAD_STATE_DONE;

    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

void payload_handle_servo(void)
{
	return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		payload_handle_LED();
	}
	else if (htim == &htim2 && is_initialized)
	{
		if (payload_state == PAYLOAD_STATE_PRELAUNCH)
		{
			payload_handle_prelog();
		}
		else if (payload_state == PAYLOAD_STATE_ASCEND || payload_state == PAYLOAD_STATE_DESCEND)
		{
			payload_handle_log();
		}
	}
	else if (htim == &htim4)
	{
		payload_handle_servo();
	}
}
