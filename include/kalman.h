/**
 * @file kalman.h
 * @brief Kalman filter for vertical trajectory and roll estimation.
 */

#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "payload.h"

void payload_kalman(struct payload_avionics_state *vehicle_state,
                    struct payload_sensor_sample *input_samples, float baseline_pressure);

#endif // __KALMAN_H__
