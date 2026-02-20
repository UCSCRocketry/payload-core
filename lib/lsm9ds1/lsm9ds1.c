/**
 ******************************************************************************
 * @file           : lsm9ds1.c
 * @author         : Kyle Chen
 * @brief          : Application facing driver for LSM9DS1 IMU and Magnetometer
 * 
 *  This file was adapted from the corresponding driver file in the Zephyr
 *  Project.
 *
 ******************************************************************************
 */

#include "lsm9ds1.h"

/**
 * @brief IMU register read callback for stmdev_ctx_t
 * @param handle Opaque handle (SPI_HandleTypeDef *)
 * @param reg    Register address
 * @param data   Buffer to store read data
 * @param len    Number of bytes to read
 * @return 0 on success, -1 on error
 */
static int32_t lsm9ds1_ctx_read_imu(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) handle;
	if (hspi == NULL)
	{
		return -1;
	}
	return (lsm9ds1_spi_mem_read(hspi, reg, data, len) == HAL_OK) ? 0 : -1;
}

/**
 * @brief IMU register write callback for stmdev_ctx_t
 * @param handle Opaque handle (SPI_HandleTypeDef *)
 * @param reg    Register address
 * @param data   Data to write
 * @param len    Number of bytes to write
 * @return 0 on success, -1 on error
 */
static int32_t lsm9ds1_ctx_write_imu(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) handle;
	if (hspi == NULL)
	{
		return -1;
	}
	return (lsm9ds1_spi_mem_write(hspi, reg, (uint8_t *) data, len) == HAL_OK) ? 0 : -1;
}

/**
 * @brief Magnetometer register read callback for stmdev_ctx_t
 * @param handle Opaque handle (SPI_HandleTypeDef *)
 * @param reg    Register address
 * @param data   Buffer to store read data
 * @param len    Number of bytes to read
 * @return 0 on success, -1 on error
 */
static int32_t lsm9ds1_ctx_read_mag(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) handle;
	if (hspi == NULL)
	{
		return -1;
	}
	return (lsm9ds1_spi_mag_read(hspi, reg, data, len) == HAL_OK) ? 0 : -1;
}

/**
 * @brief Magnetometer register write callback for stmdev_ctx_t
 * @param handle Opaque handle (SPI_HandleTypeDef *)
 * @param reg    Register address
 * @param data   Data to write
 * @param len    Number of bytes to write
 * @return 0 on success, -1 on error
 */
static int32_t lsm9ds1_ctx_write_mag(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) handle;
	if (hspi == NULL)
	{
		return -1;
	}
	return (lsm9ds1_spi_mag_write(hspi, reg, (uint8_t *) data, len) == HAL_OK) ? 0 : -1;
}

/**
 * @brief Delay callback for stmdev_ctx_t (milliseconds)
 * @param millisec Delay in milliseconds
 */
static void lsm9ds1_ctx_mdelay(uint32_t millisec)
{
	HAL_Delay(millisec);
}

/**
 * @brief Initialize IMU (XL/G) register read/write context with SPI handle
 * @param ctx   Register context to initialize
 * @param hspi  SPI handle (stored in ctx->handle)
 */
void lsm9ds1_ctx_init_imu(stmdev_ctx_t *ctx, SPI_HandleTypeDef *hspi)
{
	if (ctx == NULL)
	{
		return;
	}
	ctx->read_reg = lsm9ds1_ctx_read_imu;
	ctx->write_reg = lsm9ds1_ctx_write_imu;
	ctx->mdelay = lsm9ds1_ctx_mdelay;
	ctx->handle = (void *) hspi;
	ctx->priv_data = NULL;
}

/**
 * @brief Initialize magnetometer register read/write context with SPI handle
 * @param ctx   Register context to initialize
 * @param hspi  SPI handle (stored in ctx->handle)
 */
void lsm9ds1_ctx_init_mag(stmdev_ctx_t *ctx, SPI_HandleTypeDef *hspi)
{
	if (ctx == NULL)
	{
		return;
	}
	ctx->read_reg = lsm9ds1_ctx_read_mag;
	ctx->write_reg = lsm9ds1_ctx_write_mag;
	ctx->mdelay = lsm9ds1_ctx_mdelay;
	ctx->handle = (void *) hspi;
	ctx->priv_data = NULL;
}

/* Sensitivity of the accelerometer, indexed by the raw full scale value. Unit is µg/ LSB */
static const uint16_t lsm9ds1_accel_fs_sens[] = { 61, 732, 122, 244 };

/*
 * Sensitivity of the gyroscope, indexed by the raw full scale value.
 * The value here is just a factor applied to GAIN_UNIT_G, as the sensitivity is
 * proportional to the full scale size.
 * The index 2 is never used : the value `0` is just a placeholder.
 */
static const uint16_t lsm9ds1_gyro_fs_sens[] = { 1, 2, 0, 8 };

/*
 * Values of the different sampling frequencies of the accelerometer, indexed by the raw odr
 * value that the sensor understands.
 */
static const uint16_t lsm9ds1_odr_map[] = { 0, 10, 50, 119, 238, 476, 952 };

/*
 * Value of the different sampling frequencies of the gyroscope, indexed by the raw odr value
 * that the sensor understands.
 */
static const uint16_t lsm9ds1_gyro_odr_map[] = { 0, 15, 59, 119, 238, 476, 952 };

/* Sensitivity of the magnetometer, indexed by the raw full scale value. Unit is µGauss / LSB */
static const uint16_t lsm9ds1_mag_fs_sens[] = {140, 290, 430, 580};

/*
 * Values of the different sampling frequencies of the magnetometer, indexed by the raw odr value,
 * that the sensor can understand. Unit : Hz.
 * Warning : the real values of the sampling frequencies are often not an integer.
 * For instance, the "0" on this array is not 0 Hz but 0.625 Hz
 */
static const uint16_t lsm9ds1_mag_odr_map[] = {0, 1, 2, 5, 10, 20, 40, 80};

/*
 * Values of the sampling frequencies of the magnetometer while in "fast odr" mode, indexed by the
 * raw odr value. Unit : Hz.
 */
static const uint16_t lsm9ds1_mag_fast_odr_map[] = {1000, 560, 300, 155};

/**
 * @brief Reboot the LSM9DS1 IMU (software reset)
 * @param dev Device handle
 * @return 0 on success, negative errno on failure
 */
static int lsm9ds1_reboot(const struct lsm9ds1_device *dev)
{
	const struct lsm9ds1_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *) &cfg->ctx;
	lsm9ds1_ctrl_reg8_t ctrl8_reg;
	int ret;

	ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG8, (uint8_t *) &ctrl8_reg, 1);
	if (ret < 0)
	{
		return ret;
	}

	ctrl8_reg.boot = 1;

	ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG8, (uint8_t *) &ctrl8_reg, 1);
	if (ret < 0)
	{
		return ret;
	}

	lsm9ds1_ctx_mdelay(50);

	return 0;
}

/**
 * @brief Reboot the LSM9DS1 Magnetometer (software reset)
 * @param dev Device handle
 * @return 0 on success, negative errno on failure
 */
static int lsm9ds1_mag_reboot(const struct lsm9ds1_mag_device *dev)
{
	const struct lsm9ds1_mag_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lsm9ds1_ctrl_reg2_m_t ctrl_reg2;
	int ret;

	ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG2_M, (uint8_t *)&ctrl_reg2, 1);
	if (ret < 0) {
		return ret;
	}

	ctrl_reg2.reboot = 1;

	ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG2_M, (uint8_t *)&ctrl_reg2, 1);
	if (ret < 0) {
		return ret;
	}

	lsm9ds1_ctx_mdelay(50);

	return 0;
}

/**
 * @brief Map accelerometer full-scale range (g) to register value
 * @param range Requested range in g (2, 4, 8, or 16)
 * @return Register full-scale value, or -EINVAL if unsupported
 */
static int lsm9ds1_accel_range_to_fs_val(int32_t range)
{
	switch (range)
	{
	case 2:
		return LSM9DS1_2g;
	case 4:
		return LSM9DS1_4g;
	case 8:
		return LSM9DS1_8g;
	case 16:
		return LSM9DS1_16g;
	default:
		return -EINVAL;
	}
}

/**
 * @brief Map gyroscope full-scale range (dps) to register value
 * @param range Requested range in dps (245, 500, or 2000)
 * @return Register full-scale value, or -EINVAL if unsupported
 */
static int lsm9ds1_gyro_range_to_fs_val(int32_t range)
{
	switch (range)
	{
	case 245:
		return LSM9DS1_245dps;
	case 500:
		return LSM9DS1_500dps;
	case 2000:
		return LSM9DS1_2000dps;
	default:
		return -EINVAL;
	}
}

/**
 * @brief Get accelerometer sensitivity (µg/LSB) for a full-scale setting
 * @param fs Full-scale register value
 * @return Sensitivity in µg/LSB
 */
static int lsm9ds1_accel_fs_val_to_gain(int fs)
{
	return lsm9ds1_accel_fs_sens[fs];
}

/**
 * @brief Map requested accelerometer frequency (Hz) to ODR register value
 * @param freq Requested rate in Hz
 * @return ODR index for lsm9ds1_odr_map, or -EINVAL if out of range
 */
static int lsm9ds1_accel_freq_to_odr_val(uint16_t freq)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lsm9ds1_odr_map); i++)
	{
		if (freq <= lsm9ds1_odr_map[i])
		{
			return i;
		}
	}

	return -EINVAL;
}

/**
 * @brief Map requested gyroscope frequency (Hz) to ODR register value
 * @param freq Requested rate in Hz
 * @return ODR index for lsm9ds1_gyro_odr_map, or -EINVAL if out of range
 */
static int lsm9ds1_gyro_freq_to_odr_val(uint16_t freq)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lsm9ds1_gyro_odr_map); i++)
	{
		if (freq <= lsm9ds1_gyro_odr_map[i])
		{
			return i;
		}
	}

	return -EINVAL;
}

/**
 * @brief Map requested magnetometer range to register value
 * @param range Requested range in Ga (4, 8, 12, or 16)
 * @return Register value, or -EINVAL if out of range
 */
static int lsm9ds1_mag_range_to_fs_val(int32_t range)
{
	switch (range) {
	case 4:
		return LSM9DS1_4Ga;
	case 8:
		return LSM9DS1_8Ga;
	case 12:
		return LSM9DS1_12Ga;
	case 16:
		return LSM9DS1_16Ga;
	default:
		return -EINVAL;
	}
}

/**
 * @brief Set accelerometer output data rate by raw ODR value
 * @param dev Device handle
 * @param odr ODR register value (index into lsm9ds1_odr_map)
 * @return 0 on success, negative errno on failure
 */
static int lsm9ds1_accel_set_odr_raw(const struct lsm9ds1_device *dev, uint8_t odr)
{
	const struct lsm9ds1_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *) &cfg->ctx;
	struct lsm9ds1_data *data = dev->data;
	int ret;

	lsm9ds1_ctrl_reg6_xl_t ctrl_reg6_xl;

	ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG6_XL, (uint8_t *) &ctrl_reg6_xl, 1);
	if (ret < 0)
	{
		return ret;
	}

	ctrl_reg6_xl.odr_xl = odr;

	ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG6_XL, (uint8_t *) &ctrl_reg6_xl, 1);
	if (ret < 0)
	{
		return ret;
	}

	data->accel_odr = odr;

	return 0;
}

/**
 * @brief Set gyroscope output data rate by raw ODR value
 * @param dev Device handle
 * @param odr ODR register value (index into lsm9ds1_gyro_odr_map)
 * @return 0 on success, negative errno on failure
 */
static int lsm9ds1_gyro_set_odr_raw(const struct lsm9ds1_device *dev, uint8_t odr)
{
	const struct lsm9ds1_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *) &cfg->ctx;
	struct lsm9ds1_data *data = dev->data;
	int ret;

	lsm9ds1_ctrl_reg1_g_t ctrl_reg1;

	ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG1_G, (uint8_t *) &ctrl_reg1, 1);
	if (ret < 0)
	{
		return ret;
	}

	ctrl_reg1.odr_g = odr;

	ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG1_G, (uint8_t *) &ctrl_reg1, 1);
	if (ret < 0)
	{
		return ret;
	}

	data->gyro_odr = odr;
	return 0;
}

/**
 * @brief Set gyroscope output data rate (Hz)
 * @param dev  Device handle
 * @param freq Requested rate in Hz
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_gyro_odr_set(const struct lsm9ds1_device *dev, uint16_t freq)
{
	struct lsm9ds1_data *data = dev->data;
	int odr;
	int ret;

	odr = lsm9ds1_gyro_freq_to_odr_val(freq);

	if (odr == data->gyro_odr)
	{
		return 0;
	}

	LOG_INF("LSM9DS1 You are also changing the odr of the accelerometer");

	ret = lsm9ds1_gyro_set_odr_raw(dev, odr);
	if (ret < 0)
	{
		LOG_DBG("LSM9DS1 failed to set gyroscope sampling rate");
		return ret;
	}

	/*
	 * When the gyroscope is on, the value of the accelerometer odr must be
	 * the same as the value of the gyroscope.
	 */

	ret = lsm9ds1_accel_set_odr_raw(dev, odr);
	if (ret < 0)
	{
		LOG_ERR("LSM9DS1 failed to set accelerometer sampling rate");
		return ret;
	}

	return 0;
}

/**
 * @brief Set accelerometer output data rate (Hz)
 * @param dev  Device handle
 * @param freq Requested rate in Hz
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_accel_odr_set(const struct lsm9ds1_device *dev, uint16_t freq)
{
	const struct lsm9ds1_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *) &cfg->ctx;
	struct lsm9ds1_data *data = dev->data;
	int odr, ret;
	lsm9ds1_imu_odr_t old_odr;

	ret = lsm9ds1_imu_data_rate_get(ctx, &old_odr);
	if (ret < 0)
	{
		return ret;
	}

	/*
	 * The gyroscope is on :
	 * we have to change the odr on both the accelerometer and the gyroscope
	 */
	if (old_odr & LSM9DS1_GYRO_ODR_MASK)
	{

		odr = lsm9ds1_gyro_freq_to_odr_val(freq);

		if (odr == data->gyro_odr)
		{
			return 0;
		}

		LOG_INF("LSM9DS1 You are also changing the odr of the gyroscope");

		ret = lsm9ds1_accel_set_odr_raw(dev, odr);
		if (ret < 0)
		{
			LOG_DBG("LSM9DS1 failed to set accelerometer sampling rate");
			return ret;
		}

		ret = lsm9ds1_gyro_set_odr_raw(dev, odr);
		if (ret < 0)
		{
			LOG_ERR("LSM9DS1 failed to set gyroscope sampling rate");
			return ret;
		}

		/* The gyroscope is off, we have to change the odr of just the accelerometer */
	}
	else
	{

		odr = lsm9ds1_accel_freq_to_odr_val(freq);

		if (odr == data->accel_odr)
		{
			return 0;
		}

		if (odr < 0)
		{
			return odr;
		}

		ret = lsm9ds1_accel_set_odr_raw(dev, odr);
		if (ret < 0)
		{
			LOG_DBG("LSM9DS1 failed to set accelerometer sampling rate");
			return ret;
		}
	}
	return 0;
}

/**
 * @brief Set accelerometer full-scale range
 * @param dev   Device handle
 * @param range Range in g (2, 4, 8, or 16)
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_accel_range_set(const struct lsm9ds1_device *dev, int32_t range)
{
	int fs;
	struct lsm9ds1_data *data = dev->data;
	const struct lsm9ds1_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *) &cfg->ctx;
	int ret;

	fs = lsm9ds1_accel_range_to_fs_val(range);
	if (fs < 0)
	{
		LOG_DBG("LSM9DS1 full scale value not supported");
		return fs;
	}

	ret = lsm9ds1_xl_full_scale_set(ctx, fs);
	if (ret < 0)
	{
		LOG_DBG("LSM9DS1 failed to set accelerometer full-scale");
		return ret;
	}

	data->acc_gain = lsm9ds1_accel_fs_val_to_gain(fs);
	return 0;
}

/**
 * @brief Set gyroscope full-scale range
 * @param dev   Device handle
 * @param range Range in dps (245, 500, or 2000)
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_gyro_range_set(const struct lsm9ds1_device *dev, int32_t range)
{
	int fs;
	struct lsm9ds1_data *data = dev->data;
	const struct lsm9ds1_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *) &cfg->ctx;
	int ret;

	fs = lsm9ds1_gyro_range_to_fs_val(range);
	if (fs < 0)
	{
		return fs;
	}

	ret = lsm9ds1_gy_full_scale_set(ctx, fs);
	if (ret < 0)
	{
		LOG_DBG("LSM9DS1 failed to set gyroscope full-scale");
		return ret;
	}

	data->gyro_gain = (lsm9ds1_gyro_fs_sens[fs] * LSM9DS1_GAIN_UNIT_G);
	return 0;
}

/**
 * @brief Fetch accelerometer sample into dev->data
 * @param dev Device handle
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_sample_fetch_accel(const struct lsm9ds1_device *dev)
{
	const struct lsm9ds1_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm9ds1_data *data = dev->data;
	int ret;

	ret = lsm9ds1_acceleration_raw_get(ctx, data->acc);
	if (ret < 0) {
		LOG_DBG("Failed to read sample");
		return ret;
	}

	return 0;
}

/**
 * @brief Fetch gyroscope sample into dev->data
 * @param dev Device handle
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_sample_fetch_gyro(const struct lsm9ds1_device *dev)
{
	const struct lsm9ds1_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm9ds1_data *data = dev->data;
	int ret;

	ret = lsm9ds1_angular_rate_raw_get(ctx, data->gyro);
	if (ret < 0) {
		LOG_DBG("Failed to read sample");
		return ret;
	}
	return 0;
}

#if defined(CONFIG_LSM9DS1_ENABLE_TEMP)
/**
 * @brief Fetch temperature sample into dev->data
 * @param dev Device handle
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_sample_fetch_temp(const struct lsm9ds1_device *dev)
{
	const struct lsm9ds1_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm9ds1_data *data = dev->data;
	int ret;

	ret = lsm9ds1_temperature_raw_get(ctx, &data->temp_sample);
	if (ret < 0) {
		LOG_DBG("Failed to read sample");
		return ret;
	}

	return 0;
}
#endif

/**
 * @brief Convert raw accelerometer sample to sensor_value (m/s²)
 * @param val         Output sensor value
 * @param raw_val     Raw sample
 * @param sensitivity Sensitivity in µg/LSB
 */
static inline void lsm9ds1_accel_convert(struct sensor_value *val, int raw_val,
					 uint32_t sensitivity)
{
	/* Sensitivity is exposed in ug/LSB */
	/* Convert to m/s^2 */
	sensor_ug_to_ms2(raw_val * sensitivity, val);
}

/**
 * @brief Get one or more accelerometer channel values (internal)
 * @param chan        Channel (X, Y, Z, or XYZ)
 * @param val         Output sensor value(s)
 * @param data        Device data with raw accel samples
 * @param sensitivity Sensitivity in µg/LSB
 * @return 0 on success, -ENOTSUP if channel not supported
 */
static inline int lsm9ds1_accel_get_channel(enum sensor_channel chan, struct sensor_value *val,
					    struct lsm9ds1_data *data, uint32_t sensitivity)
{
	uint8_t i;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		lsm9ds1_accel_convert(val, data->acc[0], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		lsm9ds1_accel_convert(val, data->acc[1], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		lsm9ds1_accel_convert(val, data->acc[2], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		for (i = 0; i < 3; i++) {
			lsm9ds1_accel_convert(val++, data->acc[i], sensitivity);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief Get accelerometer channel value in sensor_value form
 * @param chan Channel (SENSOR_CHAN_ACCEL_X/Y/Z or ACCEL_XYZ)
 * @param val  Output sensor value(s)
 * @param data Device data (must have been fetched)
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_accel_channel_get(enum sensor_channel chan, struct sensor_value *val,
			      struct lsm9ds1_data *data)
{
	return lsm9ds1_accel_get_channel(chan, val, data, data->acc_gain);
}

/**
 * @brief Convert raw gyroscope sample to sensor_value (rad/s)
 * @param val         Output sensor value
 * @param raw_val     Raw sample
 * @param sensitivity Sensitivity (mdps/LSB scale)
 */
static inline void lsm9ds1_gyro_convert(struct sensor_value *val, int raw_val, uint32_t sensitivity)
{
	/* Sensitivity is exposed in udps/LSB */
	/* Convert to rad/s */
	sensor_10udegrees_to_rad((raw_val * (int32_t)sensitivity) / 10, val);
}

/**
 * @brief Get one or more gyroscope channel values (internal)
 * @param chan        Channel (X, Y, Z, or XYZ)
 * @param val         Output sensor value(s)
 * @param data        Device data with raw gyro samples
 * @param sensitivity Sensitivity for unit conversion
 * @return 0 on success, -ENOTSUP if channel not supported
 */
static inline int lsm9ds1_gyro_get_channel(enum sensor_channel chan, struct sensor_value *val,
					   struct lsm9ds1_data *data, uint32_t sensitivity)
{
	uint8_t i;

	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
		lsm9ds1_gyro_convert(val, data->gyro[0], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Y:
		lsm9ds1_gyro_convert(val, data->gyro[1], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Z:
		lsm9ds1_gyro_convert(val, data->gyro[2], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		for (i = 0; i < 3; i++) {
			lsm9ds1_gyro_convert(val++, data->gyro[i], sensitivity);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief Get gyroscope channel value in sensor_value form
 * @param chan Channel (SENSOR_CHAN_GYRO_X/Y/Z or GYRO_XYZ)
 * @param val  Output sensor value(s)
 * @param data Device data (must have been fetched)
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_gyro_channel_get(enum sensor_channel chan, struct sensor_value *val,
			     struct lsm9ds1_data *data)
{
	return lsm9ds1_gyro_get_channel(chan, val, data, data->gyro_gain);
}

#if defined(CONFIG_LSM9DS1_ENABLE_TEMP)
/**
 * @brief Fill sensor_value with temperature from device data
 * @param val  Output sensor value (val1 = integer °C, val2 = fractional)
 * @param data Device data containing temp_sample
 */
static void lsm9ds1_temp_channel_get(struct sensor_value *val, struct lsm9ds1_data *data)
{
	val->val1 = data->temp_sample / TEMP_SENSITIVITY + TEMP_OFFSET;
	val->val2 = (data->temp_sample % TEMP_SENSITIVITY) * (1000000 / TEMP_SENSITIVITY);
}
#endif

/**
 * @brief Set magnetometer full-scale range
 * @param dev   Magnetometer device handle
 * @param range Range in Ga (4, 8, 12, or 16)
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_mag_range_set(const struct lsm9ds1_mag_device *dev, int32_t range)
{
	const struct lsm9ds1_mag_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm9ds1_mag_data *data = dev->data;
	int fs, ret;

	fs = lsm9ds1_mag_range_to_fs_val(range);

	ret = lsm9ds1_mag_full_scale_set(ctx, fs);
	if (ret < 0) {
		return ret;
	}

	data->mag_gain = lsm9ds1_mag_fs_sens[fs];

	return 0;
}

/**
 * @brief Map requested magnetometer frequency (Hz) to normal ODR register value
 * @param freq Requested rate in Hz (normal mode: up to 80 Hz)
 * @return ODR index for lsm9ds1_mag_odr_map, or -EINVAL if out of range
 */
static int lsm9ds1_mag_freq_to_odr_val(uint16_t freq)
{
	for (int i = 0; i < ARRAY_SIZE(lsm9ds1_mag_odr_map); i++) {
		if (freq <= lsm9ds1_mag_odr_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

/**
 * @brief Map requested magnetometer frequency (Hz) to fast ODR register value
 * @param freq Requested rate in Hz (fast mode: 155, 300, 560, or 1000 Hz)
 * @return ODR index for lsm9ds1_mag_fast_odr_map, or -EINVAL if out of range
 */
static int lsm9ds1_mag_freq_to_fast_odr_val(uint16_t freq)
{
	for (int i = ARRAY_SIZE(lsm9ds1_mag_fast_odr_map) - 1; i >= 0; i--) {
		if (freq <= lsm9ds1_mag_fast_odr_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

/**
 * @brief Set magnetometer output data rate in normal mode (≤ 80 Hz)
 * @param dev  Magnetometer device handle
 * @param freq Requested rate in Hz
 * @return 0 on success, negative errno on failure
 */
static int lsm9ds1_mag_odr_set_normal(const struct lsm9ds1_mag_device *dev, uint16_t freq)
{
	const struct lsm9ds1_mag_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm9ds1_mag_data *data = dev->data;
	lsm9ds1_ctrl_reg1_m_t ctrl_reg1_m;
	lsm9ds1_ctrl_reg4_m_t ctrl_reg4_m;
	int odr, ret;

	odr = lsm9ds1_mag_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG1_M, (uint8_t *)&ctrl_reg1_m, 1);
	if (ret < 0) {
		return ret;
	}

	if (ctrl_reg1_m.fast_odr) { /* restore the operating mode */
		ctrl_reg1_m.om = data->old_om;

		ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG4_M, (uint8_t *)&ctrl_reg4_m, 1);
		if (ret < 0) {
			return ret;
		}

		ctrl_reg4_m.omz = data->old_om;

		ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG4_M, (uint8_t *)&ctrl_reg4_m, 1);
		if (ret < 0) {
			return ret;
		}
	}

	ctrl_reg1_m._do = odr;
	ctrl_reg1_m.fast_odr = 0;

	ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG1_M, (uint8_t *)&ctrl_reg1_m, 1);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/**
 * @brief Set magnetometer output data rate in fast mode (155–1000 Hz)
 * @param dev  Magnetometer device handle
 * @param freq Requested rate in Hz
 * @return 0 on success, negative errno on failure
 */
static int lsm9ds1_mag_fast_odr_set(const struct lsm9ds1_mag_device *dev, uint16_t freq)
{
	const struct lsm9ds1_mag_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm9ds1_mag_data *data = dev->data;
	lsm9ds1_ctrl_reg1_m_t ctrl_reg1_m;
	lsm9ds1_ctrl_reg4_m_t ctrl_reg4_m;
	int odr, ret;

	odr = lsm9ds1_mag_freq_to_fast_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG1_M, (uint8_t *)&ctrl_reg1_m, 1);
	if (ret < 0) {
		return ret;
	}

	if (!ctrl_reg1_m.fast_odr) { /* preserve the operating mode */
		data->old_om = ctrl_reg1_m.om;
	}

	ctrl_reg1_m._do = 0;
	ctrl_reg1_m.fast_odr = 1;
	ctrl_reg1_m.om = odr;

	ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG1_M, (uint8_t *)&ctrl_reg1_m, 1);
	if (ret < 0) {
		return ret;
	}

	ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG4_M, (uint8_t *)&ctrl_reg4_m, 1);
	if (ret < 0) {
		return ret;
	}

	ctrl_reg4_m.omz = odr;

	ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG4_M, (uint8_t *)&ctrl_reg4_m, 1);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/**
 * @brief Set magnetometer output data rate or power down
 * @param dev Magnetometer device handle
 * @param val Desired rate (val1 = Hz; 0/0 = power down). For freq > 80 Hz uses fast ODR mode.
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_mag_odr_set(const struct lsm9ds1_mag_device *dev, const struct sensor_value *val)
{
	const struct lsm9ds1_mag_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm9ds1_mag_data *data = dev->data;
	int ret;
	lsm9ds1_ctrl_reg3_m_t ctrl_reg3_m;

	if (val->val1 == 0 && val->val2 == 0) { /* We want to power down the sensor */
		ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG3_M, (uint8_t *)&ctrl_reg3_m, 1);
		if (ret < 0) {
			return ret;
		}

		ctrl_reg3_m.md = LSM9DS1_MAG_POWER_DOWN_VALUE;

		ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG3_M, (uint8_t *)&ctrl_reg3_m, 1);
		if (ret < 0) {
			return ret;
		}

		data->powered_down = 1;

		return 0;
	}

	if (data->powered_down) {
		ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG3_M, (uint8_t *)&ctrl_reg3_m, 1);
		if (ret < 0) {
			return ret;
		}

		ctrl_reg3_m.md = 0;

		ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG3_M, (uint8_t *)&ctrl_reg3_m, 1);
		if (ret < 0) {
			return ret;
		}

		data->powered_down = 0;
	}

	if (val->val1 <= MAX_NORMAL_ODR) {
		return lsm9ds1_mag_odr_set_normal(dev, val->val1);
	}

	return lsm9ds1_mag_fast_odr_set(dev, val->val1);
}

/**
 * @brief Fetch magnetometer sample into dev->data
 * @param dev Magnetometer device handle
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_mag_sample_fetch(const struct lsm9ds1_mag_device *dev)
{
	const struct lsm9ds1_mag_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm9ds1_mag_data *data = dev->data;
	int ret;

	ret = lsm9ds1_magnetic_raw_get(ctx, data->mag);
	if (ret < 0) {
		LOG_DBG("failed to read sample");
		return -EIO;
	}

	return 0;
}

/**
 * @brief Convert raw magnetometer sample to sensor_value (µGauss)
 * @param val         Output sensor value
 * @param raw_val     Raw sample
 * @param sensitivity Sensitivity in µGauss/LSB
 */
static inline void lsm9ds1_mag_convert(struct sensor_value *val, int raw_val, uint32_t sensitivity)
{
	int64_t dval;

	/* sensitivity is exposed in µGauss/LSB. */
	dval = (int64_t)(raw_val) * sensitivity;
	val->val1 = (int32_t)(dval / 1000000);
	val->val2 = (int32_t)(dval % 1000000);
}

/**
 * @brief Get magnetometer channel value in sensor_value form
 * @param dev  Magnetometer device handle
 * @param chan Channel (SENSOR_CHAN_MAGN_X/Y/Z or MAGN_XYZ)
 * @param val  Output sensor value(s)
 * @return 0 on success, -ENOTSUP if channel not supported
 */
int lsm9ds1_mag_channel_get(const struct lsm9ds1_mag_device *dev, enum sensor_channel chan,
				   struct sensor_value *val)
{
	struct lsm9ds1_mag_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
		lsm9ds1_mag_convert(val, data->mag[0], data->mag_gain);
		break;
	case SENSOR_CHAN_MAGN_Y:
		lsm9ds1_mag_convert(val, data->mag[1], data->mag_gain);
		break;
	case SENSOR_CHAN_MAGN_Z:
		lsm9ds1_mag_convert(val, data->mag[2], data->mag_gain);
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		for (int i = 0; i < 3; i++) {
			lsm9ds1_mag_convert(val++, data->mag[i], data->mag_gain);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief Initialize LSM9DS1 IMU device (config and data)
 * @param dev Device handle (config and data must be set; ctx must be initialized)
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_init(const struct lsm9ds1_device *dev)
{
	const struct lsm9ds1_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *) &cfg->ctx;
	struct lsm9ds1_data *data = dev->data;
	uint8_t chip_id, fs;
	int ret;

	ret = lsm9ds1_reboot(dev);
	if (ret < 0)
	{
		LOG_ERR("LSM9DS1 Failed to reboot device");
		return ret;
	}

	ret = lsm9ds1_read_reg(ctx, LSM9DS1_WHO_AM_I, &chip_id, 1);
	if (ret < 0)
	{
		LOG_ERR("LSM9DS1 failed reading chip id");
		return ret;
	}

	if (chip_id != LSM9DS1_IMU_ID)
	{
		LOG_ERR("LSM9DS1 Invalid ID : got 0x%x", chip_id);
		return -EIO;
	}
	LOG_DBG("LSM9DS1 chip_id : %x", chip_id);

	LOG_DBG("LSM9DS1 output data rate is %d", cfg->imu_odr);
	ret = lsm9ds1_imu_data_rate_set(ctx, cfg->imu_odr);
	if (ret < 0)
	{
		LOG_ERR("LSM9DS1 failed to set IMU odr");
		return ret;
	}

	fs = cfg->accel_range;
	LOG_DBG("LSM9DS1 accel range is %d", fs);
	ret = lsm9ds1_xl_full_scale_set(ctx, fs);
	if (ret < 0)
	{
		LOG_ERR("LSM9DS1 failed to set accelerometer range %d", fs);
		return ret;
	}

	data->acc_gain = lsm9ds1_accel_fs_val_to_gain(fs);

	fs = cfg->gyro_range;
	LOG_DBG("LSM9DS1 gyro range is %d", fs);
	ret = lsm9ds1_gy_full_scale_set(ctx, fs);
	if (ret < 0)
	{
		LOG_ERR("LSM9DS1 failed to set gyroscope range %d", fs);
		return ret;
	}
	data->gyro_gain = (lsm9ds1_gyro_fs_sens[fs] * LSM9DS1_GAIN_UNIT_G);

	return 0;
}

/**
 * @brief Initialize LSM9DS1 Magnetometer device (config and data)
 * @param dev Device handle (config and data must be set; ctx must be initialized)
 * @return 0 on success, negative errno on failure
 */
int lsm9ds1_mag_init(const struct lsm9ds1_mag_device *dev)
{
	const struct lsm9ds1_mag_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm9ds1_mag_data *data = dev->data;
	uint8_t chip_id, fs;
	int ret;

	ret = lsm9ds1_mag_reboot(dev);
	if (ret < 0) {
		LOG_DBG("LSM9DS1 Mag Failed to reboot device");
		return ret;
	}

	ret = lsm9ds1_read_reg(ctx, LSM9DS1_WHO_AM_I_M, &chip_id, 1);
	if (ret < 0) {
		LOG_DBG("LSM9DS1 Mag failed reading chip id");
		return ret;
	}

	if (chip_id != LSM9DS1_MAG_ID) {
		LOG_DBG("LSM9DS1 Mag Invalid ID : got 0x%x", chip_id);
		return -EIO;
	}
	LOG_INF("LSM9DS1 mag chip_id : 0x%x", chip_id);

    LOG_DBG("LSM9DS1 mag output data rate is %d", cfg->mag_odr);
	ret = lsm9ds1_mag_data_rate_set(ctx, cfg->mag_odr);
	if (ret < 0) {
		LOG_ERR("LSM9DS1 Mag failed to set the odr");
		return ret;
	}

	if (cfg->mag_odr == LSM9DS1_MAG_POWER_DOWN) {
		data->powered_down = 1;
	}

	fs = cfg->mag_range;
	ret = lsm9ds1_mag_full_scale_set(ctx, fs);
	if (ret < 0) {
		LOG_ERR("LSM9DS1 failed to set magnetometer range");
		return ret;
	}

	data->mag_gain = lsm9ds1_mag_fs_sens[fs];

	return 0;
}
