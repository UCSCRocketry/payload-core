
#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

struct servo_device
{
    // Timer to use
    //! Different servo settings on different Timer peripherals
    //! may cause problems
    //
    //! Not tested on any timer other than TIM1 (Adv Ctl)
    TIM_HandleTypeDef *htim;
    ADC_HandleTypeDef *hadc;

    // For channel N, TIM_CHANNEL_N
    uint32_t    tim_channel; 
    uint32_t    adc_channel;

    uint32_t    period_us; // Signal period
    uint32_t    pwm_min_us; // Minimum pulse width
    uint32_t    pwm_max_us; // Maximum pulse width

    // Rotation degree corresponding to min PWM
    float deg_min;
    // Rotation degree corresponding to max PWM
    float deg_max;

    // Degree offset
    float deg_offset;
};

int servo_init(struct servo_device *dev);
int servo_start(struct servo_device *dev);
int servo_stop(struct servo_device *dev);
int servo_set(struct servo_device *dev, const float pos_deg);
int servo_read(struct servo_device *dev, float *servo_pos);

#endif // __SERVO_H__