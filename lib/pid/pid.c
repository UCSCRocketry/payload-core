/**
 * @file pid.c
 * @brief PID roll-rate controller.
 *
 * Ported from RocketRollControlSystem/Electronics/Firmware/CorpoProgramma.c
 * Controller() by Newaysfactory.  Struct-based rewrite; logic preserved.
 */

#include "pid.h"
#include "log.h"

/**
 * @brief Run one PID step and return the commanded output.
 *
 * Anti-windup: integral only accumulates when the previous output was not
 * saturated (same strategy as the original Controller()).
 *
 * @param pid         Controller state and gains.
 * @param measurement Current plant output (e.g. roll rate in rad/s).
 * @return Commanded output (e.g. fin deflection in radians).
 */
float pid_update(struct pid_controller *pid, float measurement)
{
    float error = pid->setpoint - measurement;
    LOG_INF("Error: %f", error);

    float p = pid->kp * error;
    if (p < pid->p_min) p = pid->p_min;
    if (p > pid->p_max) p = pid->p_max;

    if (pid->ki > 0.0f)
    {
        /* Anti-windup: only accumulate when previous output was not saturated */
        if ((pid->prev_out > pid->out_min) && (pid->prev_out < pid->out_max))
        {
            pid->integral += pid->ki * error;
        }
        if (pid->integral < pid->i_min) pid->integral = pid->i_min;
        if (pid->integral > pid->i_max) pid->integral = pid->i_max;
    }
    else
    {
        pid->integral = 0.0f;
    }

    float d = 0.0f;
    if (pid->kd > 0.0f)
    {
        d = pid->kd * (error - pid->prev_error);
        if (d < pid->d_min) d = pid->d_min;
        if (d > pid->d_max) d = pid->d_max;
    }
    pid->prev_error = error;

    float out = p + pid->integral + d;
    if (out < pid->out_min) out = pid->out_min;
    if (out > pid->out_max) out = pid->out_max;

    pid->prev_out = out;
    return out;
}

/**
 * @brief Reset integrator and derivative state (call at launch to prevent
 *        windup from pre-launch gyro drift).
 */
void pid_reset(struct pid_controller *pid)
{
    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;
    pid->prev_out   = 0.0f;
}
