/**
 * @file payload.h
 * @brief Payload state machine types and public API.
 */

#ifndef __PAYLOAD_H__
#define __PAYLOAD_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "payload_config.h"

/**
 * @brief Raw sensor sample stored in flash.
 *
 * Each field is split into integer (v1) and fractional (v2) parts, following
 * the Zephyr sensor_value convention, so no floating-point is stored on flash.
 */
struct payload_sensor_sample
{
	uint64_t timestamp_ms; // Ch 0: HAL_GetTick() in ms
	int32_t pressure_v1; // Ch 1: pressure int  (kPa)
	int32_t pressure_v2; // Ch 1: pressure frac
	int32_t fin1_pos_v1; // Ch 2: fin 1 position int  (rad)
	int32_t fin1_pos_v2; // Ch 2: fin 1 position frac
	int32_t fin2_pos_v1; // Ch 3: fin 2 position int  (rad)
	int32_t fin2_pos_v2; // Ch 3: fin 2 position frac
	int32_t accel_y_v1; // Ch 4: accel Y int  (m/s²)
	int32_t accel_y_v2; // Ch 4: accel Y frac
	int32_t velocity_y_v1; // Ch 5: velocity Y int  (m/s)
	int32_t velocity_y_v2; // Ch 5: velocity Y frac
	int32_t ang_a_y_v1; // Ch 6: angular accel Y int  (rad/s²)
	int32_t ang_a_y_v2; // Ch 6: angular accel Y frac
	int32_t ang_v_y_v1; // Ch 7: angular velocity Y int  (rad/s)
	int32_t ang_v_y_v2; // Ch 7: angular velocity Y frac
} __attribute__((packed));

/**
 * @brief Flash page consisting of four packed samples.
 */
struct payload_page
{
	struct payload_sensor_sample samples[PAYLOAD_SAMPLES_PER_PAGE];
} __attribute__((packed));

/**
 * @brief Estimated vehicle state produced by the Kalman observer.
 */
struct payload_avionics_state
{
	float x; // Vertical position    (m)
	float v; // Vertical velocity    (m/s)
	float a; // Vertical acceleration (m/s²)
	float p_ang; // Roll angle           (rad) — reserved, unused
	float v_ang; // Roll rate            (rad/s)
	float a_ang; // Roll angular accel   (rad/s²)
	float fin_angle_rad; // Commanded fin deflection (rad)
};

/**
 * @brief User-LED display state.
 */
enum led_state_e
{
	LED_ON,
	LED_BLINK_SLOW,
	LED_BLINK_FAST,
	LED_OFF,
};

/**
 * @brief Payload state machine states.
 */
enum payload_state_e
{
	PAYLOAD_STATE_INIT = 0,
	PAYLOAD_STATE_PRELAUNCH,
	PAYLOAD_STATE_ASCEND,
	PAYLOAD_STATE_DESCEND,
	PAYLOAD_STATE_DONE,
};

/**
 * @brief Debounced button read (active LOW).
 * @return Non-zero if button is pressed.
 */
static inline int button_pressed(void)
{
	if (HAL_GPIO_ReadPin(PAYLOAD_BTN_GPIO_PORT, PAYLOAD_BTN_GPIO_PIN) == GPIO_PIN_RESET)
	{
		HAL_Delay(20);
		return (HAL_GPIO_ReadPin(PAYLOAD_BTN_GPIO_PORT, PAYLOAD_BTN_GPIO_PIN) == GPIO_PIN_RESET);
	}
	return 0;
}

/**
 * @brief Initialise sensors, take a baseline pressure, and wait for the arm button.
 */
void payload_setup(void);

/**
 * @brief Call from the main loop to execute any pending 100 Hz control cycles.
 */
void payload_poll(void);

#endif // __PAYLOAD_H__
