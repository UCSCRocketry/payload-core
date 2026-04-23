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

extern SPI_HandleTypeDef hspi1; // SPI peripheral for SPIF
extern SPI_HandleTypeDef hspi3; // SPI peripheral for Altimeter/BMP
extern SPI_HandleTypeDef hspi4; // SPI peripheral for IMU

extern TIM_HandleTypeDef htim1; // Timer 1 peripheral
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static SPIF_HandleTypeDef hspif; // SPI flash device

extern enum led_state_e led_state;

static bool is_initialized = false;

static volatile enum {
    PAYLOAD_STATE_INIT = 0,
    PAYLOAD_STATE_PRELAUNCH,
    PAYLOAD_STATE_RECORDING,
    PAYLOAD_STATE_DONE
} payload_state = PAYLOAD_STATE_INIT;

struct prebuf pb = { 0 };
static float baseline_pressure = 0.0f;
static uint32_t current_page_idx = 0;
static struct payload_page recording_page;
static uint32_t page_sample_idx = 0;

void payload_run(void)
{
	// Initialize flash
	if (!SPIF_Init(&hspif, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin))
	{
		LOG_ERR("Flash init failed");
		Error_Handler();
	}
	LOG_INF("Flash: %lu pages (%lu KiB)", hspif.PageCnt, hspif.PageCnt / 4);

	// Press button to dump
	if (button_pressed())
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
	}

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3)
    {
		switch (led_state)
		{
			case LED_ON:
				HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_RESET);
				break;
			case LED_BLINK_SLOW:
				__HAL_TIM_SET_AUTORELOAD(htim, 0xFFFF);
				HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);
				break;
			case LED_BLINK_FAST:
				__HAL_TIM_SET_AUTORELOAD(htim, 0x5555);
				HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);
				break;
			default: // LED_OFF
				HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_SET);
				break;
		}
        
    } else if (htim == &htim2) {
		if (is_initialized) {
            if (payload_state == PAYLOAD_STATE_PRELAUNCH) {
				LOG_INF("Prelaunch state");

                struct payload_sample s = { 0 };
                if (sensor_io_sample(&s) == 0) {
                    prebuf_push(&pb, &s);
                    struct sensor_value cur = { .val1 = s.pressure_v1, .val2 = s.pressure_v2 };
                    float alt = bmp388_calc_altitude(baseline_pressure, sensor_value_to_float(&cur));

#ifdef __PAYLOAD_TESTING__
                    int launched = (alt >= PAYLOAD_LAUNCH_ALT_THRESHOLD_M || (HAL_GPIO_ReadPin(PAYLOAD_BTN_GPIO_PORT, PAYLOAD_BTN_GPIO_PIN) == GPIO_PIN_RESET));
#else
                    int launched = (alt >= PAYLOAD_LAUNCH_ALT_THRESHOLD_M);
#endif
                    if (launched) {
                        LOG_INF("Launch detected! Alt ~%d m", (int) alt);
                        current_page_idx = prebuf_flush(&pb, &hspif);

                        led_state = LED_ON;
                        LOG_DBG("Recording: starting at page %lu / %lu", current_page_idx, hspif.PageCnt);
                        memset(&recording_page, 0xFF, sizeof(recording_page));
                        page_sample_idx = 0;
                        payload_state = PAYLOAD_STATE_RECORDING;
                    }
                }
            } else if (payload_state == PAYLOAD_STATE_RECORDING) {
				LOG_INF("Recording state");
				
                if (current_page_idx < hspif.PageCnt) {
					LOG_INF("Recording page %lu", current_page_idx);
					
                    struct payload_sample s = { 0 };
                    if (sensor_io_sample(&s) == 0) {
                        recording_page.samples[page_sample_idx++] = s;

                        if (page_sample_idx == PAYLOAD_SAMPLES_PER_PAGE) {
                            if (!SPIF_WritePage(&hspif, current_page_idx, (uint8_t *)&recording_page, sizeof(recording_page), 0)) {
                                LOG_ERR("Flash write error at page %lu", current_page_idx);
                            }
                            current_page_idx++;
                            page_sample_idx = 0;
                            memset(&recording_page, 0xFF, sizeof(recording_page));
                        }
                    }
                } else {
                    LOG_INF("Recording terminated. Wrote %lu pages. Entering low-power stop mode.", hspif.PageCnt);
                    led_state = LED_OFF;
                    HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_SET);
                    payload_state = PAYLOAD_STATE_DONE;

                    HAL_SuspendTick();
                    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
                }
            }
		}
	}
}
