/**
 * @file kalman.c
 * @brief Kalman filter for vertical trajectory and roll estimation.
 *
 * Adapted from https://github.com/Newaysfactory/RocketRollControlSystem/
 */

#include "kalman.h"
#include "../lib/bmp388/bmp388.h"
#include "../lib/common/sensor.h"

/**
 * @brief Performs the Kalman filter update and writes results into vehicle_state.
 *
 * @param vehicle_state  Current estimated state (updated in place).
 * @param input_samples  Raw sensor readings for this tick.
 * @param baseline_pressure Launch-site pressure used for altitude calculation (kPa).
 */
void payload_kalman(struct payload_avionics_state *vehicle_state,
                    struct payload_sensor_sample *input_samples, float baseline_pressure)
{
	struct sensor_value gyroy_val = { 0 };
	struct sensor_value accely_val = { 0 };
	struct sensor_value press_val = { 0 };

	gyroy_val.val1 = input_samples->ang_v_y_v1;
	gyroy_val.val2 = input_samples->ang_v_y_v2;
	accely_val.val1 = input_samples->accel_y_v1;
	accely_val.val2 = input_samples->accel_y_v2;
	press_val.val1 = input_samples->pressure_v1;
	press_val.val2 = input_samples->pressure_v2;

	float alt = bmp388_calc_altitude(baseline_pressure, sensor_value_to_float(&press_val));
	float baro_in_meters = PAYLOAD_FEET_TO_METERS_CONV * alt;
	float acc_in_ms2 = sensor_value_to_float(&accely_val);

	// --- Vertical trajectory ---

	float x_pred = vehicle_state->x + (vehicle_state->v * PAYLOAD_PHI_VERTICAL_12_S)
	               + (vehicle_state->a * PAYLOAD_PHI_VERTICAL_13_S);
	float v_pred = vehicle_state->v + (vehicle_state->a * PAYLOAD_PHI_VERTICAL_23_S);
	float a_pred = vehicle_state->a;

	vehicle_state->x = x_pred + (PAYLOAD_K11_VERTICAL * (baro_in_meters - x_pred))
	                   + (PAYLOAD_K12_VERTICAL * (acc_in_ms2 - a_pred));
	vehicle_state->v = v_pred + (PAYLOAD_K21_VERTICAL * (baro_in_meters - x_pred))
	                   + (PAYLOAD_K22_VERTICAL * (acc_in_ms2 - a_pred));
	vehicle_state->a = a_pred + (PAYLOAD_K31_VERTICAL * (baro_in_meters - x_pred))
	                   + (PAYLOAD_K32_VERTICAL * (acc_in_ms2 - a_pred));

	// --- Roll ---

	float p_in_rad_s = sensor_value_to_float(&gyroy_val);

	float v_ang_pred = vehicle_state->v_ang + vehicle_state->a_ang * PAYLOAD_PHI_ROLL_11_S;
	float a_ang_pred = vehicle_state->a_ang;

	vehicle_state->v_ang = v_ang_pred + PAYLOAD_K1_ROLL * (p_in_rad_s - v_ang_pred) + 0.046;
	vehicle_state->v_ang = (float) ((int32_t) (vehicle_state->v_ang * 100)) / 100.0f;
	vehicle_state->a_ang = a_ang_pred + PAYLOAD_K2_ROLL * (p_in_rad_s - v_ang_pred);
}
