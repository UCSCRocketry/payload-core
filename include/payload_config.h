/**
 * @file payload_config.h
 * @brief Compile-time configuration for the payload system.
 */

#ifndef __PAYLOAD_CONFIG_H__
#define __PAYLOAD_CONFIG_H__

// If defined, some test-specific functionality will be enabled
#define __PAYLOAD_TESTING__

// Sensor polling period during prelaunch buffer capture
#define PAYLOAD_PREBUF_POLL_PERIOD_MS 10U

// Sensor polling period during flight
#define PAYLOAD_MAIN_POLL_PERIOD_MS 10U

// User button port/pin on Blackpill
#define PAYLOAD_BTN_GPIO_PORT GPIOA
#define PAYLOAD_BTN_GPIO_PIN  GPIO_PIN_0

// Altitude at which launch is declared
#define PAYLOAD_LAUNCH_ALT_THRESHOLD_M 100.0f

// Number of BMP388 readings averaged to establish the launch-site baseline
#define PAYLOAD_BASELINE_SAMPLES 16

// Prelaunch buffer depth (samples)
#define PAYLOAD_PREBUF_DEPTH 64

// Number of payload samples (64 B) per flash page (256 B)
#define PAYLOAD_SAMPLES_PER_PAGE 4

// After launch detection, payload records for this amount of time
#define PAYLOAD_MAX_RECORD_LEN_S 1200

// Maximum number of samples recorded after launch detection
#define PAYLOAD_MAX_SAMPLES_NUM (PAYLOAD_MAX_RECORD_LEN_S * (1000 / PAYLOAD_MAIN_POLL_PERIOD_MS))

// --- Apogee / landing detection ---

// Altitude must drop this far below peak to declare descent
#define PAYLOAD_APOGEE_MARGIN_M 50.0f

// Altitude (m) at or below which the payload is considered near the ground
#define PAYLOAD_LAND_ALT_THRESHOLD_M 200.0f

// Duration (s) that altitude must stay below the land threshold to confirm landing
#define PAYLOAD_LAND_HOLD_S 30

// Consecutive samples below land threshold required to confirm landing
#define PAYLOAD_LAND_HOLD_SAMPLES (PAYLOAD_LAND_HOLD_S * (1000 / PAYLOAD_MAIN_POLL_PERIOD_MS))

// --- Kalman filter gains ---
// Current values for M1/A1 sensor combination.
// Must be re-tuned when sensors change.

// Vertical trajectory — barometer channel
#define PAYLOAD_K11_VERTICAL 0.0221
#define PAYLOAD_K21_VERTICAL 0.0145
#define PAYLOAD_K31_VERTICAL 0.0001

// Vertical trajectory — accelerometer channel
#define PAYLOAD_K12_VERTICAL 0.0011
#define PAYLOAD_K22_VERTICAL 0.0091
#define PAYLOAD_K32_VERTICAL 0.0584

// Roll motion
#define PAYLOAD_K1_ROLL 0.2655
#define PAYLOAD_K2_ROLL 0.7224

// State transition matrix — vertical motion (ms then seconds)
#define PAYLOAD_PHI_VERTICAL_12 (PAYLOAD_MAIN_POLL_PERIOD_MS)
#define PAYLOAD_PHI_VERTICAL_13                                                                    \
	((float) PAYLOAD_MAIN_POLL_PERIOD_MS * ((float) PAYLOAD_MAIN_POLL_PERIOD_MS / 2))
#define PAYLOAD_PHI_VERTICAL_23   (PAYLOAD_MAIN_POLL_PERIOD_MS)
#define PAYLOAD_PHI_VERTICAL_12_S ((float) PAYLOAD_PHI_VERTICAL_12 / 1000.0f)
#define PAYLOAD_PHI_VERTICAL_13_S ((float) PAYLOAD_PHI_VERTICAL_13 / 1000.0f)
#define PAYLOAD_PHI_VERTICAL_23_S ((float) PAYLOAD_PHI_VERTICAL_23 / 1000.0f)

// State transition matrix — roll motion (ms then seconds)
#define PAYLOAD_PHI_ROLL_11   (PAYLOAD_MAIN_POLL_PERIOD_MS)
#define PAYLOAD_PHI_ROLL_11_S ((float) PAYLOAD_PHI_ROLL_11 / 1000.0f)

// --- PID controller (roll stabilisation) ---
#define PAYLOAD_PID_SETPOINT 0.0f // Target roll rate (rad/s)
#define PAYLOAD_PID_KP       0.07f // Proportional gain
#define PAYLOAD_PID_KI       0.016f // Integral gain
#define PAYLOAD_PID_KD       0.0f // Derivative gain
#define PAYLOAD_PID_P_MAX    1.0f
#define PAYLOAD_PID_P_MIN    -1.0f
#define PAYLOAD_PID_I_MAX    10000.0f
#define PAYLOAD_PID_I_MIN    -10000.0f
#define PAYLOAD_PID_D_MAX    0.1f
#define PAYLOAD_PID_D_MIN    -0.1f
#define PAYLOAD_PID_OUT_MAX  1.000f
#define PAYLOAD_PID_OUT_MIN  -1.000f

// Unit conversion
#define PAYLOAD_FEET_TO_METERS_CONV 0.3048

#endif // __PAYLOAD_CONFIG_H__
