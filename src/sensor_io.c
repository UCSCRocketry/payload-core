/**
 * @file sensor_io.c
 * @brief BMP388 and LSM9DS1 init and sampling.
 */

#include "sensor_io.h"
#include "../lib/bmp388/bmp388.h"
#include "../lib/lsm9ds1/lsm9ds1.h"
#include "../lib/lsm9ds1/lsm9ds1_reg.h"
#include "../lib/common/sensor.h"
#include "../lib/common/log.h"

static struct bmp388_config bmp_cfg;
static struct bmp388_data bmp_dat;
static struct bmp388_device bmp_dev = { .config = &bmp_cfg, .data = &bmp_dat };

static struct lsm9ds1_config imu_cfg;
static struct lsm9ds1_data imu_dat;
static struct lsm9ds1_device imu_dev = { .config = &imu_cfg, .data = &imu_dat };

/**
 * @brief Initialise all sensors.
 * @param hspi_bmp Pointer to SPI handle for BMP388
 * @param hspi_imu Pointer to SPI handle for LSM9DS1
 * @return 0 on success, else failure.
 */
int sensor_io_init(SPI_HandleTypeDef *hspi_bmp, SPI_HandleTypeDef *hspi_imu)
{
    // Init BMP388
	bmp_cfg.hspi = hspi_bmp;
	if (bmp388_init(&bmp_dev) != 0)
	{
		LOG_ERR("Sensor IO: BMP388 init failed");
		return -1;
	}
	LOG_INF("Sensor IO: BMP388 OK");

    // Init LSM9DS1
	lsm9ds1_ctx_init_imu(&imu_cfg.ctx, hspi_imu);
	imu_cfg.accel_range = LSM9DS1_4g;
	imu_cfg.gyro_range = LSM9DS1_500dps;
	imu_cfg.imu_odr = LSM9DS1_IMU_119Hz;
	if (lsm9ds1_init(&imu_dev) != 0)
	{
		LOG_ERR("Sensor IO: LSM9DS1 init failed");
		return -1;
	}
	LOG_INF("Sensor IO: LSM9DS1 OK");

	return 0;
}

/**
 * @brief Read all sensors into one payload_sample.
 * @param s Pointer to sample.
 * @return 0 on success, else failure.
 */
int sensor_io_sample(struct payload_sample *s)
{
	struct sensor_value accel[3] = { 0 };
	struct sensor_value gyro[3] = { 0 };
	struct sensor_value press = { 0 };

	s->timestamp_ms = (uint64_t) HAL_GetTick();

	

	if (bmp388_sample_fetch(&bmp_dev)
	    || bmp388_channel_get(&bmp_dev, SENSOR_CHAN_PRESS, &press))
	{
		LOG_WRN("Sensor IO: BMP388 sample failed");
		return -1;
	}

	if (lsm9ds1_sample_fetch_accel(&imu_dev)
	    || lsm9ds1_accel_channel_get(SENSOR_CHAN_ACCEL_XYZ, accel, &imu_dat))
	{
		LOG_WRN("Sensor IO: LSM9DS1 accel sample failed");
		return -1;
	}

	if (lsm9ds1_sample_fetch_gyro(&imu_dev)
	    || lsm9ds1_gyro_channel_get(SENSOR_CHAN_GYRO_XYZ, gyro, &imu_dat))
	{
		LOG_WRN("Sensor IO: LSM9DS1 gyro sample failed");
		return -1;
	}

	s->pressure_v1 = press.val1;
	s->pressure_v2 = press.val2;
	s->accel_x_v1 = accel[0].val1;
	s->accel_x_v2 = accel[0].val2;
	s->accel_y_v1 = accel[1].val1;
	s->accel_y_v2 = accel[1].val2;
	s->accel_z_v1 = accel[2].val1;
	s->accel_z_v2 = accel[2].val2;
	s->gyro_x_v1 = gyro[0].val1;
	s->gyro_x_v2 = gyro[0].val2;
	s->gyro_y_v1 = gyro[1].val1;
	s->gyro_y_v2 = gyro[1].val2;
	s->gyro_z_v1 = gyro[2].val1;
	s->gyro_z_v2 = gyro[2].val2;

	return 0;
}

/**
 * @brief Average num_samples pressure readings for a launch-site baseline.
 * @param num_samples Number of samples 
 * @return Baseline pressure in kPa.
 */
float sensor_io_press_baseline(int num_samples)
{
	float sum = 0.0f;
	struct sensor_value v = { 0 };

	for (int i = 0; i < num_samples; i++)
	{
		if (bmp388_sample_fetch(&bmp_dev) == 0
		    && bmp388_channel_get(&bmp_dev, SENSOR_CHAN_PRESS, &v) == 0)
		{
			sum += sensor_value_to_float(&v);
		}
		HAL_Delay(10);
	}

	return sum / num_samples;
}
