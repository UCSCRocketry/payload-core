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
#include <string.h>
#include <stdbool.h>

_Static_assert(sizeof(struct payload_page) == 256, "payload_page must be exactly 256 bytes");

extern SPI_HandleTypeDef hspi1; // For SPIF
extern SPI_HandleTypeDef hspi3; // For BMP
extern SPI_HandleTypeDef hspi4; // For IMU

static SPIF_HandleTypeDef spif;

void payload_run(void)
{
	// Initialize flash
	if (!SPIF_Init(&spif, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin))
	{
		LOG_ERR("Flash init failed");
		Error_Handler();
	}
	LOG_INF("Flash: %lu pages (%lu KiB)", spif.PageCnt, spif.PageCnt / 4);

	// Button is dump sensitive
	if (button_pressed())
	{
		LOG_INF("Startup button held - dumping flash to SD card...");
		if (!dump_and_format_flash(&spif))
		{
			LOG_INF("Dump OK");
		}
		else
		{
			LOG_ERR("Dump failed - flash NOT erased");
		}
	}

	// Wait 10 seconds as a buffer between dump sensitive and arm sensitive button
	LOG_INF("Waiting 10 s...");
	HAL_Delay(10000);

	// Button is arm sensitive
	while (!button_pressed())
	{
		HAL_Delay(50);
	}
	LOG_INF("Armed.");

	// Initialize sensors
	if (sensor_io_init(&hspi3, &hspi4) != 0)
	{
		Error_Handler();
	}

	// Take baseline pressure
	LOG_INF("Sampling baseline pressure (%d readings)...", PAYLOAD_BASELINE_SAMPLES);
	float baseline = sensor_io_press_baseline(PAYLOAD_BASELINE_SAMPLES);
	LOG_INF("Baseline: %ld.%03ld kPa", (int32_t) baseline,
	        (int32_t) ((baseline - (int32_t) baseline) * 1000.0f));

	LOG_INF("Pre-launch buffer active. Threshold: %d m", (int) PAYLOAD_LAUNCH_ALT_THRESHOLD_M);

	struct prebuf pb = { 0 };

	while (1)
	{
		struct payload_sample s = { 0 };
		if (sensor_io_sample(&s) == 0)
		{
			prebuf_push(&pb, &s);

			struct sensor_value cur = { .val1 = s.pressure_v1, .val2 = s.pressure_v2 };
			float alt = bmp388_calc_altitude(baseline, sensor_value_to_float(&cur));
			if (alt >= PAYLOAD_LAUNCH_ALT_THRESHOLD_M || button_pressed())
			{
				LOG_INF("Launch detected! Alt ~%d m", (int) alt);
				break;
			}
		}
		HAL_Delay(100);
	}

	uint32_t page_idx = prebuf_flush(&pb, &spif);

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

	LOG_INF("Recording: page %lu / %lu", page_idx, spif.PageCnt);
	{
		struct payload_page page;
        struct payload_sample s = { 0 };
		uint32_t page_sample_idx = 0;

		memset(&page, 0xFF, sizeof(page));

		while (page_idx < spif.PageCnt)
		{	
			if (sensor_io_sample(&s) == 0)
			{
				page.samples[page_sample_idx++] = s;

				if (page_sample_idx == PAYLOAD_SAMPLES_PER_PAGE)
				{
					if (!SPIF_WritePage(&spif, page_idx, (uint8_t *) &page, sizeof(page), 0))
					{
						LOG_ERR("Flash write error at page %lu", page_idx);
					}
					page_idx++;
					page_sample_idx = 0;
					memset(&page, 0xFF, sizeof(page));
				}
			}
			HAL_Delay(10);
		}
	}

	LOG_INF("Flash full (%lu pages). Entering low-power stop mode.", spif.PageCnt);
	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	while (1);
}
