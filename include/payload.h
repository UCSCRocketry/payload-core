/**
 * @file payload.h
 * @brief Payload recording system types and entry point.
 */

#ifndef __PAYLOAD_H__
#define __PAYLOAD_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"

// If defined, some test-specific functionality will be enabled
#define __PAYLOAD_TESTING__

// Sensor polling period during prelaunch buffer capture
#define PAYLOAD_PREBUF_POLL_PERIOD_MS 10U

// Sensor polling period during flight
#define PAYLOAD_MAIN_POLL_PERIOD_MS 10U

// User button port/pin on Blackpill;
#define PAYLOAD_BTN_GPIO_PORT GPIOA
#define PAYLOAD_BTN_GPIO_PIN  GPIO_PIN_0

// Record / prebuf commit trigger altitude.
#define PAYLOAD_LAUNCH_ALT_THRESHOLD_M 100.0f

// Number of BMP388 readings averaged to establish the launch-site baseline.
#define PAYLOAD_BASELINE_SAMPLES 16

// Prelaunch buffer size
#define PAYLOAD_PREBUF_DEPTH 64

// Number of payload samples (64B) per page (256B)
#define PAYLOAD_SAMPLES_PER_PAGE 4

// After launch detection, payload records for this amount of time
#define PAYLOAD_MAX_RECORD_LEN_S 1200

// Maximum number of samples recorded after launch detection
#define PAYLOAD_MAX_SAMPLES_NUM (PAYLOAD_MAX_RECORD_LEN_S * (1000 / PAYLOAD_MAIN_POLL_PERIOD_MS))

// --- Ground landing detection parameters ---

// Altitude must drop this far below peak to declare descent (prevents apogee oscillation false
// triggers)
#define PAYLOAD_APOGEE_MARGIN_M 50.0f

// Altitude (m) at or below which the payload is considered "near ground"
#define PAYLOAD_LAND_ALT_THRESHOLD_M 200.0f

// Duration (seconds) that altitude must stay below land threshold to confirm landing
#define PAYLOAD_LAND_HOLD_S 30

// Number of consecutive samples below land threshold required (derived from hold time and poll
// period)
#define PAYLOAD_LAND_HOLD_SAMPLES (PAYLOAD_LAND_HOLD_S * (1000 / PAYLOAD_MAIN_POLL_PERIOD_MS))

// --- Kalman Filter Parameters ---
//! Current values for M1/A1 combination.
//! These values must be changed depending on the sensor used.
//Kalman gains for vertical trajectory
#define PAYLOAD_K11_VERTICAL 		0.0221
#define PAYLOAD_K21_VERTICAL 		0.0145
#define PAYLOAD_K31_VERTICAL 		0.0001

//Kalman gains for vertical trajectory (accelerometer channel)
#define PAYLOAD_K12_VERTICAL 		0.0011
#define PAYLOAD_K22_VERTICAL 		0.0091
#define PAYLOAD_K32_VERTICAL 		0.0584

//Kalman gains for roll motion
#define PAYLOAD_K1_ROLL 			0.2655
#define PAYLOAD_K2_ROLL 			0.7224

//State transition matrix elements for vertical motion
//! These are in ms, not seconds
#define PAYLOAD_PHI_VERTICAL_12   	(PAYLOAD_MAIN_POLL_PERIOD_MS)                 
#define PAYLOAD_PHI_VERTICAL_13   	((float) PAYLOAD_MAIN_POLL_PERIOD_MS * ((float) PAYLOAD_MAIN_POLL_PERIOD_MS / 2))
#define PAYLOAD_PHI_VERTICAL_23   	(PAYLOAD_MAIN_POLL_PERIOD_MS)
//! These are in seconds
#define PAYLOAD_PHI_VERTICAL_12_S	((float) PAYLOAD_PHI_VERTICAL_12 / 1000.0f)
#define PAYLOAD_PHI_VERTICAL_13_S	((float) PAYLOAD_PHI_VERTICAL_13 / 1000.0f)
#define PAYLOAD_PHI_VERTICAL_23_S	((float) PAYLOAD_PHI_VERTICAL_23 / 1000.0f)

//State transition matrix element for roll motion
//! This is in ms, not seconds
#define PAYLOAD_PHI_ROLL_11       	(PAYLOAD_MAIN_POLL_PERIOD_MS)
//! These are in seconds
#define PAYLOAD_PHI_ROLL_11_S     	((float) PAYLOAD_PHI_ROLL_11 / 1000.0f)

// --- PID Controller Parameters (from RocketRollControlSystem HardwareProfile.h) ---
#define PAYLOAD_PID_SETPOINT   0.0f     // Target roll rate (rad/s)
#define PAYLOAD_PID_KP         0.07f    // Proportional gain
#define PAYLOAD_PID_KI         0.016f   // Integral gain
#define PAYLOAD_PID_KD         0.0f     // Derivative gain
#define PAYLOAD_PID_P_MAX      1.0f
#define PAYLOAD_PID_P_MIN     -1.0f
#define PAYLOAD_PID_I_MAX      10000.0f
#define PAYLOAD_PID_I_MIN     -10000.0f
#define PAYLOAD_PID_D_MAX      0.1f
#define PAYLOAD_PID_D_MIN     -0.1f
#define PAYLOAD_PID_OUT_MAX    1.000f
#define PAYLOAD_PID_OUT_MIN   -1.000f

// --- Other Constants ---
#define PAYLOAD_FEET_TO_METERS_CONV 0.3048

/**
 * @brief Payload logging sample
 */
struct payload_sensor_sample
{
	uint64_t timestamp_ms; // Ch 0: HAL_GetTick() in ms
	int32_t pressure_v1; // Ch 1: raw pressure int (kPa)
	int32_t pressure_v2; // Ch 1: raw pressure frac
	int32_t fin1_pos_v1; // Ch 2: position of fin1 int (rad)
	int32_t fin1_pos_v2; // Ch 2: position of fin1 frac
	int32_t fin2_pos_v1; // Ch 3: position of fin2 int (rad)
	int32_t fin2_pos_v2; // Ch 3: position of fin2 frac
	int32_t accel_z_v1; // Ch 4: accel Z int (m/s^2)
	int32_t accel_z_v2; // Ch 4: accel Z frac
	int32_t velocity_z_v1; // Ch 5: velocity Z int (m/s)
	int32_t velocity_z_v2; // Ch 5: velocity Z frac
	int32_t ang_a_z_v1; // Ch 6: ang accel Z int (rad/s^2)
	int32_t ang_a_z_v2; // Ch 6: ang accel Z frac
	int32_t ang_v_z_v1; // Ch 7: ang velocity Z int (rad/s)
	int32_t ang_v_z_v2; // Ch 7: ang velocity Z frac
} __attribute__((packed));

/**
 * @brief Debounced button press (ACTIVE LOW)
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
 * @brief Payload flash page consisting of four samples
 */
struct payload_page
{
	struct payload_sensor_sample samples[PAYLOAD_SAMPLES_PER_PAGE];
} __attribute__((packed));

/**
 * @brief User LED state enum
 */
enum led_state_e
{
	LED_ON,
	LED_BLINK_SLOW,
	LED_BLINK_FAST,
	LED_OFF,
};

// Payload Core State Machine State enum
enum payload_state_e
{
	PAYLOAD_STATE_INIT = 0,
	PAYLOAD_STATE_PRELAUNCH,
	PAYLOAD_STATE_ASCEND,
	PAYLOAD_STATE_DESCEND,
	PAYLOAD_STATE_DONE,
};

// Estimated vehicle state (output of Kalman observer)
struct payload_avionics_state
{                 
    float x;             	//Vertical position (m)
    float v;             	//Vertical velocity (m/s)
    float a;             	//Vertical acceleration (m/s^2)
    float p_ang;         	//Roll angle (rad) - reserved, currently unused
    float v_ang;         	//Roll rate (rad/s)
    float a_ang;         	//Roll angular acceleration (rad/s^2)
    float fin_angle_rad; 	//Commanded fin deflection angle (rad)
};

/**
 * @brief Run the payload state machine.
 */
void payload_setup(void);

/**
 * @brief Call from the main loop to execute pending 100 Hz control cycles.
 */
void payload_poll(void);

#endif // __PAYLOAD_H__
