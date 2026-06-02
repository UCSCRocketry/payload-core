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
#include "../lib/pid/pid.h"
#include "sensor.h"
#include "main.h"
#include "log.h"
#include "servo.h"
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

extern struct servo_device servo_dev1; // Servo 1
extern struct servo_device servo_dev2; // Servo 2

extern enum led_state_e led_state;

/* Private Variables -----------------------------------------------*/
static bool is_initialized = false;
static volatile bool payload_run_pending = false;

static volatile enum payload_state_e payload_state = PAYLOAD_STATE_INIT;
static struct payload_avionics_state avionics_state = { 0 };
static struct payload_sensor_sample sensor_sample = { 0 };

struct prebuf pb = { 0 };
static float baseline_pressure = 0.0f;
static uint32_t current_page_idx = 0;
static struct payload_page recording_page;
static uint32_t page_sample_idx = 0;
static float peak_altitude = 0.0f;
static uint32_t land_hold_count = 0;

static struct pid_controller roll_pid = {
	.kp      = PAYLOAD_PID_KP,
	.ki      = PAYLOAD_PID_KI,
	.kd      = PAYLOAD_PID_KD,
	.setpoint = PAYLOAD_PID_SETPOINT,
	.p_min   = PAYLOAD_PID_P_MIN,   .p_max   = PAYLOAD_PID_P_MAX,
	.i_min   = PAYLOAD_PID_I_MIN,   .i_max   = PAYLOAD_PID_I_MAX,
	.d_min   = PAYLOAD_PID_D_MIN,   .d_max   = PAYLOAD_PID_D_MAX,
	.out_min = PAYLOAD_PID_OUT_MIN, .out_max = PAYLOAD_PID_OUT_MAX,
};

/* Private Functions -----------------------------------------------*/
static void payload_terminate_recording(void);


/**
 * @brief Sets up the payload system.
 *
 * Handles the initial regular function/erase flash/dump flash
 * decision at the payload start. Initializes the sensors and 
 * takes baseline pressure.
 *
 * @return void
 */
void payload_setup(void)
{
	// Press button to dump or erase
	if (button_pressed())
	{
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
	baseline_pressure = sensor_io_press_baseline(PAYLOAD_BASELINE_SAMPLES);
	LOG_DBG("Baseline: %ld.%03ld kPa", (int32_t) baseline_pressure,
	        (int32_t) ((baseline_pressure - (int32_t) baseline_pressure) * 1000.0f));

	LOG_INF("Pre-launch buffer active. Threshold: %d m", (int) PAYLOAD_LAUNCH_ALT_THRESHOLD_M);

	is_initialized = true;
	payload_state = PAYLOAD_STATE_PRELAUNCH;
	return;
}

/**
 * @brief Handles the landing detection during preflight
 *
 * @return void
 */
void payload_detect_launch(void)
{
	struct payload_sensor_sample s = { 0 };
	if (sensor_io_sample(&s) == 0)
	{
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
			led_state = LED_ON;

			LOG_DBG("Recording: starting at page %lu / %lu", current_page_idx, hspif.PageCnt);

			current_page_idx = 0;
			page_sample_idx = 0;
			memset(&recording_page, 0xFF, sizeof(recording_page));
			pid_reset(&roll_pid);
			payload_state = PAYLOAD_STATE_ASCEND;
		}
	}
	return;
}

/**
 * @brief Handles the logging function during flight.
 *
 * @return void
 */
void payload_record_avionics(void)
{
	// Copy sample from the vehicle data
	struct payload_sensor_sample s = sensor_sample;
	recording_page.samples[page_sample_idx++] = s;

	// If the SPIF page is full, flush the data, otherwise move on
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

	// Calculate altitude from pressure data
	struct sensor_value cur = { .val1 = s.pressure_v1, .val2 = s.pressure_v2 };
	float alt = bmp388_calc_altitude(baseline_pressure, sensor_value_to_float(&cur));

	// If ascending, detect descent
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
	else if (payload_state == PAYLOAD_STATE_DESCEND) // If descending, detect landing
	{
		// Keep a count of how long payload is under landing altitude threshold.
		land_hold_count = (alt <= PAYLOAD_LAND_ALT_THRESHOLD_M) ? (land_hold_count + 1) :
						  (0);

		// If payload senses it has been under landing alt threshold for long
		// enough a time, it will go into landing mode and terminate recording.
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


	return;
}

/**
 * @brief Terminates the recording and suspends the system.
 *
 * @return void
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
 * @brief Handles the fin actuation.
 *
 * @return void
 */
void payload_handle_controls(void)
{
	LOG_INF("roll rate: %f", avionics_state.v_ang);
	float fin_rad = pid_update(&roll_pid, avionics_state.v_ang);
	avionics_state.fin_angle_rad = fin_rad;

	float fin_deg = fin_rad * (180.0f / 3.14159265f);
	servo_set(&servo_dev1, fin_deg);
	servo_set(&servo_dev2, fin_deg);
	return;
}

/**
 * @brief Performs Kalman filter and updates the state.
 *
 * Adapted from https://github.com/Newaysfactory/RocketRollControlSystem/
 *
 * @param vehicle_state The current vehicle state.
 * @param input_samples The samples to calculate the next vehicle state
 * @return void
 */
// Kalman filter: 
void payload_kalman(struct payload_avionics_state *vehicle_state, struct payload_sensor_sample *input_samples)
{
	struct sensor_value gyroz_val = { 0 };
	struct sensor_value accelz_val = { 0 };
	struct sensor_value press_val = { 0 };
	float alt;

    float baro_in_meters;
	float acc_in_ms2;
	float p_in_rad_s;
	float x_pred;
	float v_pred;
	float a_pred;
	float v_ang_pred;
	float a_ang_pred;

	// Get values and altitude
	gyroz_val.val1 = input_samples->ang_v_z_v1;
	gyroz_val.val2 = input_samples->ang_v_z_v2;
	accelz_val.val1 = input_samples->accel_z_v1;
	accelz_val.val2 = input_samples->accel_z_v2;
	press_val.val1 = input_samples->pressure_v1;
	press_val.val2 = input_samples->pressure_v2;

	alt = bmp388_calc_altitude(baseline_pressure, sensor_value_to_float(&press_val));

    //--- VERTICAL TRAJECTORY SECTION ---

    baro_in_meters = PAYLOAD_FEET_TO_METERS_CONV * alt;
    acc_in_ms2 = sensor_value_to_float(&accelz_val);

    //Kalman prediction step
    x_pred = vehicle_state->x + (vehicle_state->v * PAYLOAD_PHI_VERTICAL_12_S) + (vehicle_state->a * PAYLOAD_PHI_VERTICAL_13_S);
    v_pred = vehicle_state->v + (vehicle_state->a * PAYLOAD_PHI_VERTICAL_23_S);
    a_pred = vehicle_state->a;

    //Kalman correction step (two measurements: barometer and accelerometer)
    vehicle_state->x = x_pred + (PAYLOAD_K11_VERTICAL * (baro_in_meters - x_pred)) + (PAYLOAD_K12_VERTICAL * (acc_in_ms2 - a_pred));
    vehicle_state->v = v_pred + (PAYLOAD_K21_VERTICAL * (baro_in_meters - x_pred)) + (PAYLOAD_K22_VERTICAL * (acc_in_ms2 - a_pred));
    vehicle_state->a = a_pred + (PAYLOAD_K31_VERTICAL * (baro_in_meters - x_pred)) + (PAYLOAD_K32_VERTICAL * (acc_in_ms2 - a_pred));

    //--- ROLL SECTION ---

    p_in_rad_s = sensor_value_to_float(&gyroz_val) - 0.0;

    //Kalman prediction step
    v_ang_pred = vehicle_state->v_ang + vehicle_state->a_ang * PAYLOAD_PHI_ROLL_11_S;
    a_ang_pred = vehicle_state->a_ang;

    //Kalman correction step (single measurement: gyro Z-axis)
    vehicle_state->v_ang = v_ang_pred + PAYLOAD_K1_ROLL * (p_in_rad_s - v_ang_pred) + 0.046;
	vehicle_state->v_ang = (float) ((int32_t) (vehicle_state->v_ang * 100)) / 100.0f;
    vehicle_state->a_ang = a_ang_pred + PAYLOAD_K2_ROLL * (p_in_rad_s - v_ang_pred);

	return;
}

/**
 * @brief Runs the main functions of the payload.
 *
 * Designed to be called at 100 Hz.
 *
 * @return void
 */
static void payload_run(void)
{
	// Get raw data from the sensors
	if (sensor_io_sample(&sensor_sample))
	{
		Error_Handler();
	}

	// Use sensor data to update the vehicle state
	payload_kalman(&avionics_state, &sensor_sample);
	
	// Write Kalman outputs back into vehicle_sample
	sensor_sample.velocity_z_v1 = (int32_t) avionics_state.v;
	sensor_sample.velocity_z_v2 = (int32_t) ((avionics_state.v - (int32_t) avionics_state.v) * 1000000.0f);
	sensor_sample.ang_a_z_v1 = (int32_t) avionics_state.a_ang;
	sensor_sample.ang_a_z_v2 = (int32_t) ((avionics_state.a_ang - (int32_t) avionics_state.a_ang) * 1000000.0f);
	sensor_sample.ang_v_z_v1 = (int32_t) avionics_state.v_ang;
	sensor_sample.ang_v_z_v2 = (int32_t) ((avionics_state.v_ang - (int32_t) avionics_state.v_ang) * 1000000.0f);

	// Handle fin control
	if (payload_state == PAYLOAD_STATE_ASCEND || payload_state == PAYLOAD_STATE_DESCEND)
	{
		payload_handle_controls();
		// Read actual fin positions (ADC feedback) into vehicle_sample
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

	// Do Logging
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
 * @brief Called from the main loop to execute the 100 Hz control cycle.
 *
 * Avoids calling SysTick dependent functions within TIM interrupt context.
 *
 * @return void
 */
void payload_poll(void)
{
	if (payload_run_pending)
	{
		payload_run_pending = false;
		payload_run();
	}
}

/**
 * @brief TIM period elapsed interrupt callback.
 *
 * @param htim The timer that called the interrupt.
 * @return void
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
	else if (htim == &htim2 && is_initialized)
	{
		
	}
	else if (htim == &htim4 && is_initialized) // Every 100 Hz
	{
		payload_run_pending = true;
	}
}
