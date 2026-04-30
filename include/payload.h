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
#define PAYLOAD_PREBUF_POLL_PERIOD_MS	20U

// Sensor polling period during flight
#define PAYLOAD_MAIN_POLL_PERIOD_MS		20U

// User button port/pin on Blackpill;
#define PAYLOAD_BTN_GPIO_PORT   GPIOA
#define PAYLOAD_BTN_GPIO_PIN    GPIO_PIN_0

// Record / prebuf commit trigger altitude.
#define PAYLOAD_LAUNCH_ALT_THRESHOLD_M      100.0f

// Number of BMP388 readings averaged to establish the launch-site baseline.
#define PAYLOAD_BASELINE_SAMPLES    16

// Prelaunch buffer size
#define PAYLOAD_PREBUF_DEPTH    	64

// Number of payload samples (64B) per page (256B)
#define PAYLOAD_SAMPLES_PER_PAGE    4

// After launch detection, payload records for this amount of time
#define PAYLOAD_MAX_RECORD_LEN_S	1200

// Maximum number of samples recorded after launch detection
#define PAYLOAD_MAX_SAMPLES_NUM		PAYLOAD_MAX_RECORD_LEN_S * (1000 / PAYLOAD_MAIN_POLL_PERIOD_MS)

// --- Ground landing detection parameters ---

// Altitude must drop this far below peak to declare descent (prevents apogee oscillation false triggers)
#define PAYLOAD_APOGEE_MARGIN_M		30.0f

// Altitude (m) at or below which the payload is considered "near ground"
#define PAYLOAD_LAND_ALT_THRESHOLD_M	30.0f

// Duration (seconds) that altitude must stay below land threshold to confirm landing
#define PAYLOAD_LAND_HOLD_S		10

// Number of consecutive samples below land threshold required (derived from hold time and poll period)
#define PAYLOAD_LAND_HOLD_SAMPLES	(PAYLOAD_LAND_HOLD_S * (1000 / PAYLOAD_MAIN_POLL_PERIOD_MS))

/**
 * @brief Payload logging sample
 */
struct payload_sample
{
	uint64_t timestamp_ms; // Ch 0: HAL_GetTick() in ms
	int32_t pressure_v1; // Ch 1: pressure int (kPa)
	int32_t pressure_v2; // Ch 1: pressure frac
	int32_t accel_x_v1; // Ch 2: accel X int (m/s^2)
	int32_t accel_x_v2; // Ch 2: accel X frac
	int32_t accel_y_v1; // Ch 3: accel Y int
	int32_t accel_y_v2; // Ch 3: accel Y frac
	int32_t accel_z_v1; // Ch 4: accel Z int
	int32_t accel_z_v2; // Ch 4: accel Z frac
	int32_t gyro_x_v1; // Ch 5: gyro X int (rad/s)
	int32_t gyro_x_v2; // Ch 5: gyro X frac
	int32_t gyro_y_v1; // Ch 6: gyro Y int
	int32_t gyro_y_v2; // Ch 6: gyro Y frac
	int32_t gyro_z_v1; // Ch 7: gyro Z int
	int32_t gyro_z_v2; // Ch 7: gyro Z frac
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
	struct payload_sample samples[PAYLOAD_SAMPLES_PER_PAGE];
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
    PAYLOAD_STATE_RECORDING,
    PAYLOAD_STATE_DONE,
};

/**
 * @brief Run the payload state machine.
 */
void payload_run(void);

#endif // __PAYLOAD_H__
