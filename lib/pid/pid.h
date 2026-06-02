#ifndef __PID_H__
#define __PID_H__

struct pid_controller
{
    float kp;
    float ki;
    float kd;
    float setpoint;
    float p_min,   p_max;
    float i_min,   i_max;
    float d_min,   d_max;
    float out_min, out_max;
    /* state */
    float prev_error;
    float integral;
    float prev_out;
};

float pid_update(struct pid_controller *pid, float measurement);
void  pid_reset(struct pid_controller *pid);

#endif /* __PID_H__ */
