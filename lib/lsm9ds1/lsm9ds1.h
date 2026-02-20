/**
 ******************************************************************************
 * @file           : lsm9ds1.h
 * @author         : Kyle Chen
 * @brief          : Application facing driver for LSM9DS1 IMU and Magnetometer
 * 
 *  This file was adapted from the corresponding driver file in the Zephyr
 *  Project.
 *
 ******************************************************************************
 */

#ifndef __LSM9DS1_H__
#define __LSM9DS1_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "lsm9ds1_reg.h"
#include "log.h"
#include "sensor.h"
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef ARRAY_SIZE
/** @brief Macro for number of elements in a statically allocated array */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

/** @brief SPI transfer timeout in milliseconds */
#define LSM9DS1_SPI_TIMEOUT 100

/** @brief GPIO port for IMU (accelerometer/gyroscope) chip select */
#define LSM9DS1_IMU_CS_GPIO_BUS GPIOB
/** @brief GPIO port for magnetometer chip select */
#define LSM9DS1_MAG_CS_GPIO_BUS GPIOB
/** @brief GPIO pin for IMU chip select */
#define LSM9DS1_IMU_CS_GPIO_PIN GPIO_PIN_6
/** @brief GPIO pin for magnetometer chip select */
#define LSM9DS1_MAG_CS_GPIO_PIN GPIO_PIN_7

/** @brief Accelerometer gain unit (µg/LSB) for scaling to standard units */
#define LSM9DS1_GAIN_UNIT_XL (61LL)
/** @brief Gyroscope gain unit (mdps/LSB) for scaling to standard units */
#define LSM9DS1_GAIN_UNIT_G  (8750LL)

/** @brief Temperature offset: raw zero corresponds to 25 °C */
#define LSM9DS1_TEMP_OFFSET      25
/** @brief Temperature sensitivity in LSB per °C */
#define LSM9DS1_TEMP_SENSITIVITY 16

/** @brief Mask for gyroscope output data rate register field */
#define LSM9DS1_GYRO_ODR_MASK 0x7

/**
 * @brief Read from LSM9DS1 IMU (accelerometer/gyroscope) registers over SPI
 *
 * @param hspi  SPI handle
 * @param addr  Register address (read bit is set automatically)
 * @param pData Buffer to store read data
 * @param Size  Number of bytes to read
 * @return HAL status
 */
static inline HAL_StatusTypeDef lsm9ds1_spi_mem_read(SPI_HandleTypeDef *hspi, uint8_t addr,
                                                     uint8_t *pData, uint16_t Size)
{
	addr = (addr & 0x7F) | (1 << 7U);
	HAL_GPIO_WritePin(LSM9DS1_IMU_CS_GPIO_BUS, LSM9DS1_IMU_CS_GPIO_PIN, GPIO_PIN_RESET);
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(hspi, &addr, 1, LSM9DS1_SPI_TIMEOUT);
	ret |= HAL_SPI_Receive(hspi, pData, Size, LSM9DS1_SPI_TIMEOUT);
	HAL_GPIO_WritePin(LSM9DS1_IMU_CS_GPIO_BUS, LSM9DS1_IMU_CS_GPIO_PIN, GPIO_PIN_SET);
	return ret;
}

/**
 * @brief Write to LSM9DS1 IMU (accelerometer/gyroscope) registers over SPI
 *
 * @param hspi  SPI handle
 * @param addr  Register address
 * @param pData Data to write
 * @param Size  Number of bytes to write
 * @return HAL status
 */
static inline HAL_StatusTypeDef lsm9ds1_spi_mem_write(SPI_HandleTypeDef *hspi, uint8_t addr,
                                                      uint8_t *pData, uint16_t Size)
{
	addr = (addr & 0x7F);
	HAL_GPIO_WritePin(LSM9DS1_IMU_CS_GPIO_BUS, LSM9DS1_IMU_CS_GPIO_PIN, GPIO_PIN_RESET);
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(hspi, &addr, 1, LSM9DS1_SPI_TIMEOUT);
	ret |= HAL_SPI_Transmit(hspi, pData, Size, LSM9DS1_SPI_TIMEOUT);
	HAL_GPIO_WritePin(LSM9DS1_IMU_CS_GPIO_BUS, LSM9DS1_IMU_CS_GPIO_PIN, GPIO_PIN_SET);
	return ret;
}

/**
 * @brief Read from LSM9DS1 magnetometer registers over SPI
 *
 * @param hspi  SPI handle
 * @param addr  Register address (read bit is set automatically)
 * @param pData Buffer to store read data
 * @param Size  Number of bytes to read
 * @return HAL status
 */
static inline HAL_StatusTypeDef lsm9ds1_spi_mag_read(SPI_HandleTypeDef *hspi, uint8_t addr,
                                                     uint8_t *pData, uint16_t Size)
{
	addr = (addr & 0x3F) | (1 << 7U);
    if (Size > 1)
    {
        addr = addr | (1 << 6U);
    }
	HAL_GPIO_WritePin(LSM9DS1_MAG_CS_GPIO_BUS, LSM9DS1_MAG_CS_GPIO_PIN, GPIO_PIN_RESET);
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(hspi, &addr, 1, LSM9DS1_SPI_TIMEOUT);
	ret |= HAL_SPI_Receive(hspi, pData, Size, LSM9DS1_SPI_TIMEOUT);
	HAL_GPIO_WritePin(LSM9DS1_MAG_CS_GPIO_BUS, LSM9DS1_MAG_CS_GPIO_PIN, GPIO_PIN_SET);
	return ret;
}

/**
 * @brief Write to LSM9DS1 magnetometer registers over SPI
 *
 * @param hspi  SPI handle
 * @param addr  Register address
 * @param pData Data to write
 * @param Size  Number of bytes to write
 * @return HAL status
 */
static inline HAL_StatusTypeDef lsm9ds1_spi_mag_write(SPI_HandleTypeDef *hspi, uint8_t addr,
                                                      uint8_t *pData, uint16_t Size)
{
	addr = (addr & 0x3F);
    if (Size > 1)
    {
        addr = addr | (1 << 6U);
    }
	HAL_GPIO_WritePin(LSM9DS1_MAG_CS_GPIO_BUS, LSM9DS1_MAG_CS_GPIO_PIN, GPIO_PIN_RESET);
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(hspi, &addr, 1, LSM9DS1_SPI_TIMEOUT);
	ret |= HAL_SPI_Transmit(hspi, pData, Size, LSM9DS1_SPI_TIMEOUT);
	HAL_GPIO_WritePin(LSM9DS1_MAG_CS_GPIO_BUS, LSM9DS1_MAG_CS_GPIO_PIN, GPIO_PIN_SET);
	return ret;
}

/** @brief LSM9DS1 IMU device configuration */
struct lsm9ds1_config {
	stmdev_ctx_t ctx;     /**< Register read/write context */
	uint8_t accel_range;  /**< Accelerometer full-scale range setting */
	uint8_t gyro_range;   /**< Gyroscope full-scale range setting */
	uint8_t imu_odr;      /**< IMU output data rate setting */
};

/** @brief LSM9DS1 IMU sampled data and scaling factors */
struct lsm9ds1_data {
	int16_t acc[3];      /**< Accelerometer X, Y, Z (raw) */
	uint32_t acc_gain;   /**< Accelerometer gain for unit conversion */
	int16_t gyro[3];     /**< Gyroscope X, Y, Z (raw) */
	uint32_t gyro_gain;  /**< Gyroscope gain for unit conversion */

	uint16_t accel_odr;  /**< Accelerometer output data rate (Hz) */
	uint16_t gyro_odr;   /**< Gyroscope output data rate (Hz) */
#if defined(CONFIG_LSM9DS1_ENABLE_TEMP)
	int16_t temp_sample; /**< Temperature sample (raw) */
#endif
};

/** @brief LSM9DS1 IMU device handle */
struct lsm9ds1_device {
	struct lsm9ds1_config *config; /**< Device configuration */
	struct lsm9ds1_data *data;     /**< Sample and gain data */
};

/** @brief LSM9DS1 Magnetometer device configuration */
struct lsm9ds1_mag_config {
	stmdev_ctx_t ctx;   /**< Register read/write context */
	uint8_t mag_range;  /**< Magnetometer full-scale range setting */
	uint8_t mag_odr;    /**< Magnetometer output data rate setting */
};

/** @brief LSM9DS1 Magnetometer sampled data and scaling factors  */
struct lsm9ds1_mag_data {
	int16_t mag[3];         /**< Magnetometer X, Y, Z (raw) */
	uint32_t mag_gain;      /**< Magnetometer gain for unit conversion */
	uint8_t old_om;         /**< Old Operating Mode */
	uint8_t powered_down;   /**< Sensor is powered down */
};

/** @brief LSM9DS1 Magnetometer device handle */
struct lsm9ds1_mag_device {
	struct lsm9ds1_mag_config *config; /**< Device configuration */
	struct lsm9ds1_mag_data *data;     /**< Sample and gain data */
};

#define MAX_NORMAL_ODR 80 /* 80 Hz : output data rate > 80 must be set differently */

/* Value to write in the register of the sensor to power it down */
#define LSM9DS1_MAG_POWER_DOWN_VALUE 2

void lsm9ds1_ctx_init_imu(stmdev_ctx_t *ctx, SPI_HandleTypeDef *hspi);
void lsm9ds1_ctx_init_mag(stmdev_ctx_t *ctx, SPI_HandleTypeDef *hspi);

int lsm9ds1_init(const struct lsm9ds1_device *dev);

int lsm9ds1_sample_fetch_accel(const struct lsm9ds1_device *dev);
int lsm9ds1_accel_channel_get(enum sensor_channel chan, struct sensor_value *val,
                              struct lsm9ds1_data *data);

int lsm9ds1_sample_fetch_gyro(const struct lsm9ds1_device *dev);
int lsm9ds1_gyro_channel_get(enum sensor_channel chan, struct sensor_value *val,
                             struct lsm9ds1_data *data);

int lsm9ds1_gyro_odr_set(const struct lsm9ds1_device *dev, uint16_t freq);
int lsm9ds1_accel_odr_set(const struct lsm9ds1_device *dev, uint16_t freq);
int lsm9ds1_accel_range_set(const struct lsm9ds1_device *dev, int32_t range);
int lsm9ds1_gyro_range_set(const struct lsm9ds1_device *dev, int32_t range);

#if defined(CONFIG_LSM9DS1_ENABLE_TEMP)
int lsm9ds1_sample_fetch_temp(const struct lsm9ds1_device *dev);
#endif


int lsm9ds1_mag_init(const struct lsm9ds1_mag_device *dev);

int lsm9ds1_mag_sample_fetch(const struct lsm9ds1_mag_device *dev);
int lsm9ds1_mag_channel_get(const struct lsm9ds1_mag_device *dev, enum sensor_channel chan,
				   struct sensor_value *val);

int lsm9ds1_mag_range_set(const struct lsm9ds1_mag_device *dev, int32_t range);
int lsm9ds1_mag_odr_set(const struct lsm9ds1_mag_device *dev, const struct sensor_value *val);

#endif /* __LSM9DS1_H__ */