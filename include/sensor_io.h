/**
 * @file sensor_io.h
 * @brief BMP388 and LSM9DS1 init and sampling.
 */

#ifndef __SENSOR_IO__
#define __SENSOR_IO__

#include "payload.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

int sensor_io_init(SPI_HandleTypeDef *hspi_bmp, SPI_HandleTypeDef *hspi_imu);

int sensor_io_sample(struct payload_sample *s);

float sensor_io_press_baseline(int num_samples);

#endif /* __SENSORS_H__ */
