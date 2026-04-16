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

	// Take baseline pressure
	LOG_DBG("Sampling baseline pressure (%d readings)...", PAYLOAD_BASELINE_SAMPLES);
	float baseline = sensor_io_press_baseline(PAYLOAD_BASELINE_SAMPLES);
	LOG_DBG("Baseline: %ld.%03ld kPa", (int32_t) baseline,
	        (int32_t) ((baseline - (int32_t) baseline) * 1000.0f));

	LOG_INF("Pre-launch buffer active. Threshold: %d m", (int) PAYLOAD_LAUNCH_ALT_THRESHOLD_M);

	struct prebuf pb = { 0 };

    // Logging to prebuffer until launch detected
	while (1)
	{
		struct payload_sample s = { 0 };
		if (!sensor_io_sample(&s))
		{
			prebuf_push(&pb, &s);

			struct sensor_value cur = { .val1 = s.pressure_v1, .val2 = s.pressure_v2 };
			float alt = bmp388_calc_altitude(baseline, sensor_value_to_float(&cur));

#ifdef __PAYLOAD_TESTING__
			int launched = (alt >= PAYLOAD_LAUNCH_ALT_THRESHOLD_M || button_pressed());
#else
			int launched = (alt >= PAYLOAD_LAUNCH_ALT_THRESHOLD_M);
#endif
			if (launched)
			{
				LOG_INF("Launch detected! Alt ~%d m", (int) alt);
				break;
			}
		}
		HAL_Delay(PAYLOAD_PREBUF_POLL_PERIOD_MS);
	}

	uint32_t page_idx = prebuf_flush(&pb, &hspif);

#ifndef LOG_NODEBUG
    for (int i = 0; i < PREBUF_DEPTH; i++)
    {
        struct sensor_value cur = { .val1 = pb.prebuf[i].pressure_v1, .val2 = pb.prebuf[i].pressure_v2 };
        struct sensor_value accelx = {.val1 = pb.prebuf[i].accel_x_v1, .val2 = pb.prebuf[i].accel_x_v2};
        struct sensor_value accely = {.val1 = pb.prebuf[i].accel_y_v1, .val2 = pb.prebuf[i].accel_y_v2};
        struct sensor_value accelz = {.val1 = pb.prebuf[i].accel_z_v1, .val2 = pb.prebuf[i].accel_z_v2};
        struct sensor_value gyrox = {.val1 = pb.prebuf[i].gyro_x_v1, .val2 = pb.prebuf[i].gyro_x_v2};
        struct sensor_value gyroy = {.val1 = pb.prebuf[i].gyro_y_v1, .val2 = pb.prebuf[i].gyro_y_v2};
        struct sensor_value gyroz = {.val1 = pb.prebuf[i].gyro_z_v1, .val2 = pb.prebuf[i].gyro_z_v2};
        LOG_INF("Prebuf log: [%lu], (%f), (%f, %f, %f), (%f, %f, %f)", pb.prebuf[i].timestamp_ms,
            sensor_value_to_float(&cur), sensor_value_to_float(&accelx), sensor_value_to_float(&accely), sensor_value_to_float(&accelz),
            sensor_value_to_float(&gyrox), sensor_value_to_float(&gyroy), sensor_value_to_float(&gyroz));
    }
#endif

    // Regular Recording Loop with ground landing detection
    led_state = LED_ON;
	LOG_DBG("Recording: page %lu / %lu", page_idx, hspif.PageCnt);
	{
		struct payload_page page;
        struct payload_sample s = { 0 };
		uint32_t page_sample_idx = 0;

		// Flight phase state machine — starts ASCENDING since launch was just detected
		enum flight_phase_e phase = FLIGHT_ASCENDING;
		float peak_alt = PAYLOAD_LAUNCH_ALT_THRESHOLD_M; // at least this high at launch
		uint32_t land_hold_count = 0;

		memset(&page, 0xFF, sizeof(page));

		while (page_idx < hspif.PageCnt)
		{	
			if (sensor_io_sample(&s) == 0)
			{
				page.samples[page_sample_idx++] = s;

				if (page_sample_idx == PAYLOAD_SAMPLES_PER_PAGE)
				{
					if (!SPIF_WritePage(&hspif, page_idx, (uint8_t *) &page, sizeof(page), 0))
					{
						LOG_ERR("Flash write error at page %lu", page_idx);
					}
					page_idx++;
					page_sample_idx = 0;
					memset(&page, 0xFF, sizeof(page));
				}

				// --- Ground landing detection state machine ---
				struct sensor_value cur_press = { .val1 = s.pressure_v1, .val2 = s.pressure_v2 };
				float alt = bmp388_calc_altitude(baseline, sensor_value_to_float(&cur_press));

				switch (phase)
				{
				case FLIGHT_ASCENDING:
					if (alt > peak_alt)
					{
						peak_alt = alt;
					}
					// Only detect apogee if we actually ascended past the launch threshold.
					// This prevents false landing detection when launch is triggered by
					// button press in __PAYLOAD_TESTING__ mode while on the bench.
					if (peak_alt > PAYLOAD_LAUNCH_ALT_THRESHOLD_M
					    && alt < peak_alt - PAYLOAD_APOGEE_MARGIN_M)
					{
						phase = FLIGHT_DESCENDING;
						LOG_INF("Apogee detected. Peak ~%d m, current ~%d m", (int) peak_alt, (int) alt);
					}
					break;

				case FLIGHT_DESCENDING:
					if (alt <= PAYLOAD_LAND_ALT_THRESHOLD_M)
					{
						land_hold_count++;
					}
					else
					{
						land_hold_count = 0;
					}
					if (land_hold_count >= PAYLOAD_LAND_HOLD_SAMPLES)
					{
						phase = FLIGHT_LANDED;
						LOG_INF("Landing detected! Alt ~%d m, held for %lu samples", (int) alt, land_hold_count);
					}
					break;

				default:
					break;
				}

				if (phase == FLIGHT_LANDED)
				{
					// Flush any partial page before stopping
					if (page_sample_idx > 0)
					{
						SPIF_WritePage(&hspif, page_idx, (uint8_t *) &page, sizeof(page), 0);
						page_idx++;
					}
					break;
				}
			}
			HAL_Delay(PAYLOAD_MAIN_POLL_PERIOD_MS);
		}
	}

	LOG_INF("Recording terminated. Wrote %lu pages. Entering low-power stop mode.", page_idx);
	led_state = LED_OFF;
    HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_SET);
	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	while (1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
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
        
    }
}
