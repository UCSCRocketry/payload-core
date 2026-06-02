/**
 * @file payload.c
 * @brief Payload state machine orchestrator.
 */

#include "payload.h"
#include "kalman.h"
#include "sensor_io.h"
#include "dump.h"
#include "spif.h"
#include "../lib/bmp388/bmp388.h"
#include "../lib/pid/pid.h"
#include "sensor.h"
#include "main.h"
#include "log.h"
#include "servo.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include <string.h>
#include <stdbool.h>

_Static_assert(sizeof(struct payload_page) == 256, "payload_page must be exactly 256 bytes");

/* External Variables -----------------------------------------------*/
extern SPI_HandleTypeDef hspi1; // SPI flash
extern SPI_HandleTypeDef hspi3; // BMP388 altimeter
extern SPI_HandleTypeDef hspi4; // LSM9DS1 IMU

// extern TIM_HandleTypeDef htim2; // Unused
extern TIM_HandleTypeDef htim3; // LED timer
extern TIM_HandleTypeDef htim4; // 100 Hz control timer

extern SPIF_HandleTypeDef hspif;

extern struct servo_device servo_dev1;
extern struct servo_device servo_dev2;

extern enum led_state_e led_state;

/* Private Variables -----------------------------------------------*/
static bool is_initialized = false;
static volatile bool run_pending = false;

static volatile enum payload_state_e payload_state = PAYLOAD_STATE_INIT;
static struct payload_avionics_state avionics_state = { 0 };
static struct payload_sensor_sample sensor_sample = { 0 };

static float baseline_pressure = 0.0f;
static uint32_t current_page_idx = 0;
static struct payload_page recording_page;
static uint32_t page_sample_idx = 0;
static float peak_altitude = 0.0f;
static uint32_t land_hold_count = 0;

static struct pid_controller roll_pid = {
	.kp = PAYLOAD_PID_KP,
	.ki = PAYLOAD_PID_KI,
	.kd = PAYLOAD_PID_KD,
	.setpoint = PAYLOAD_PID_SETPOINT,
	.p_min = PAYLOAD_PID_P_MIN,
	.p_max = PAYLOAD_PID_P_MAX,
	.i_min = PAYLOAD_PID_I_MIN,
	.i_max = PAYLOAD_PID_I_MAX,
	.d_min = PAYLOAD_PID_D_MIN,
	.d_max = PAYLOAD_PID_D_MAX,
	.out_min = PAYLOAD_PID_OUT_MIN,
	.out_max = PAYLOAD_PID_OUT_MAX,
};

/* Private Function Prototypes --------------------------------------*/
static void handle_startup_button(void);
static void payload_detect_launch(void);
static void payload_record_avionics(void);
static void payload_handle_controls(void);
static void payload_terminate_recording(void);
static void payload_run(void);

/* Implementation --------------------------------------------------*/

/**
 * @brief Handles the pre-flight startup button:
 *        release within 10 s → dump flash; hold 10 s → erase flash.
 */
static void handle_startup_button(void)
{
	if (!button_pressed())
	{
		return;
	}

	for (uint8_t i = 0; i < 10; i++)
	{
		if (!button_pressed())
		{
			LOG_INF("Startup button held - dumping flash to SD card...");
			led_state = LED_BLINK_FAST;
			if (!dump_flash(&hspif))
			{
				LOG_INF("Dump OK");
			}
			else
			{
				LOG_ERR("Dump failed - flash NOT erased");
			}
			led_state = LED_OFF;
			return;
		}

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

/**
 * @brief Initialises sensors, takes the baseline pressure, and waits for the arm button.
 */
void payload_setup(void)
{
	handle_startup_button();

	while (!button_pressed())
	{
		HAL_Delay(10);
	}
	LOG_INF("Armed.");
	led_state = LED_BLINK_SLOW;

	if (sensor_io_init(&hspi3, &hspi4) != 0)
	{
		Error_Handler();
	}

	LOG_DBG("Sampling baseline pressure (%d readings)...", PAYLOAD_BASELINE_SAMPLES);
	baseline_pressure = sensor_io_press_baseline(PAYLOAD_BASELINE_SAMPLES);
	LOG_DBG("Baseline: %ld.%03ld kPa", (int32_t) baseline_pressure,
	        (int32_t) ((baseline_pressure - (int32_t) baseline_pressure) * 1000.0f));

	LOG_INF("Pre-launch buffer active. Threshold: %d m", (int) PAYLOAD_LAUNCH_ALT_THRESHOLD_M);

	is_initialized = true;
	payload_state = PAYLOAD_STATE_PRELAUNCH;
}

/**
 * @brief Samples sensors and transitions to ASCEND if launch altitude is exceeded.
 */
static void payload_detect_launch(void)
{
	struct payload_sensor_sample s = { 0 };
	if (sensor_io_sample(&s) != 0)
	{
		return;
	}

	struct sensor_value cur = { .val1 = s.pressure_v1, .val2 = s.pressure_v2 };
	float alt = bmp388_calc_altitude(baseline_pressure, sensor_value_to_float(&cur))
	            * PAYLOAD_FEET_TO_METERS_CONV;

#ifdef __PAYLOAD_TESTING__
	int launched = (alt >= PAYLOAD_LAUNCH_ALT_THRESHOLD_M
	                || (HAL_GPIO_ReadPin(PAYLOAD_BTN_GPIO_PORT, PAYLOAD_BTN_GPIO_PIN)
	                    == GPIO_PIN_RESET));
#else
	int launched = (alt >= PAYLOAD_LAUNCH_ALT_THRESHOLD_M);
#endif

	if (!launched)
	{
		return;
	}

	LOG_INF("Launch detected! Alt ~%d m", (int) alt);
	led_state = LED_ON;

	LOG_DBG("Recording: starting at page %lu / %lu", current_page_idx, hspif.PageCnt);
	current_page_idx = 0;
	page_sample_idx = 0;
	memset(&recording_page, 0xFF, sizeof(recording_page));
	pid_reset(&roll_pid);
	payload_state = PAYLOAD_STATE_ASCEND;
}

/**
 * @brief Writes the current sample to flash and handles apogee / landing detection.
 */
static void payload_record_avionics(void)
{
	struct payload_sensor_sample s = sensor_sample;
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
	float alt = bmp388_calc_altitude(baseline_pressure, sensor_value_to_float(&cur))
	            * PAYLOAD_FEET_TO_METERS_CONV;

	if (payload_state == PAYLOAD_STATE_ASCEND)
	{
		if (alt > peak_altitude)
		{
			peak_altitude = alt;
		}
		else if (alt < peak_altitude - PAYLOAD_APOGEE_MARGIN_M)
		{
			LOG_INF("Apogee detected! Peak alt ~%d m. Transitioning to DESCEND.",
			        (int) peak_altitude);

			if (servo_start(&servo_dev1))
			{
				LOG_ERR("Error starting servo device 1 (TIM1-CH1).");
			}
			if (servo_start(&servo_dev2))
			{
				LOG_ERR("Error starting servo device 2 (TIM1-CH2).");
			}

			payload_state = PAYLOAD_STATE_DESCEND;
		}
	}
	else // PAYLOAD_STATE_DESCEND
	{
		land_hold_count = (alt <= PAYLOAD_LAND_ALT_THRESHOLD_M) ? (land_hold_count + 1) : 0;

		if (land_hold_count >= PAYLOAD_LAND_HOLD_SAMPLES)
		{
			LOG_INF("Landing detected!");

			if (page_sample_idx > 0)
			{
				if (!SPIF_WritePage(&hspif, current_page_idx, (uint8_t *) &recording_page,
				                    sizeof(recording_page), 0))
				{
					LOG_ERR("Flash write error at page %lu", current_page_idx);
				}
				current_page_idx++;
			}

			payload_terminate_recording();
		}
	}
}

/**
 * @brief Runs the PID controller and drives the fins to the commanded angle.
 */
static void payload_handle_controls(void)
{
	LOG_DBG("roll rate: %f", avionics_state.v_ang);
	float fin_rad = pid_update(&roll_pid, avionics_state.v_ang);
	avionics_state.fin_angle_rad = fin_rad;

	float fin_deg = fin_rad * (180.0f / 3.14159265f);
	servo_set(&servo_dev1, fin_deg);
	servo_set(&servo_dev2, fin_deg);
}

/**
 * @brief Flushes remaining data to flash and enters low-power stop mode.
 */
static void payload_terminate_recording(void)
{
	LOG_INF("Recording terminated. Wrote %lu pages. Entering low-power stop mode.",
	        current_page_idx);
	led_state = LED_OFF;
	HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_SET);
	payload_state = PAYLOAD_STATE_DONE;

	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

/**
 * @brief Main 100 Hz control cycle: sample sensors, run Kalman, control fins, log.
 */
static void payload_run(void)
{
	if (sensor_io_sample(&sensor_sample))
	{
		Error_Handler();
	}

	payload_kalman(&avionics_state, &sensor_sample, baseline_pressure);

	// Write Kalman outputs back into the sample for logging
	sensor_sample.velocity_z_v1 = (int32_t) avionics_state.v;
	sensor_sample.velocity_z_v2
	        = (int32_t) ((avionics_state.v - (int32_t) avionics_state.v) * 1000000.0f);
	sensor_sample.ang_a_z_v1 = (int32_t) avionics_state.a_ang;
	sensor_sample.ang_a_z_v2
	        = (int32_t) ((avionics_state.a_ang - (int32_t) avionics_state.a_ang) * 1000000.0f);
	sensor_sample.ang_v_z_v1 = (int32_t) avionics_state.v_ang;
	sensor_sample.ang_v_z_v2
	        = (int32_t) ((avionics_state.v_ang - (int32_t) avionics_state.v_ang) * 1000000.0f);

	if (payload_state == PAYLOAD_STATE_ASCEND || payload_state == PAYLOAD_STATE_DESCEND)
	{
		payload_handle_controls();

		float fin_pos;
		if (servo_read(&servo_dev1, &fin_pos) == 0)
		{
			float fin_rad = fin_pos * (3.14159265f / 180.0f);
			sensor_sample.fin1_pos_v1 = (int32_t) fin_rad;
			sensor_sample.fin1_pos_v2 = (int32_t) ((fin_rad - (int32_t) fin_rad) * 1000000.0f);
		}
		if (servo_read(&servo_dev2, &fin_pos) == 0)
		{
			float fin_rad = fin_pos * (3.14159265f / 180.0f);
			sensor_sample.fin2_pos_v1 = (int32_t) fin_rad;
			sensor_sample.fin2_pos_v2 = (int32_t) ((fin_rad - (int32_t) fin_rad) * 1000000.0f);
		}
	}

	if (payload_state == PAYLOAD_STATE_PRELAUNCH)
	{
		payload_detect_launch();
	}
	else if (payload_state == PAYLOAD_STATE_ASCEND || payload_state == PAYLOAD_STATE_DESCEND)
	{
		if (current_page_idx < hspif.PageCnt)
		{
			payload_record_avionics();
		}
		else
		{
			payload_terminate_recording();
		}
	}
}

/**
 * @brief Called from the main loop to execute any pending 100 Hz cycles.
 *
 * Deferred from the TIM4 interrupt to avoid calling SysTick-dependent functions
 * from interrupt context.
 */
void payload_poll(void)
{
	if (run_pending)
	{
		run_pending = false;
		payload_run();
	}
}

/**
 * @brief TIM period-elapsed interrupt callback.
 */
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
	}
	else if (htim == &htim4 && is_initialized)
	{
		run_pending = true;
	}
}
