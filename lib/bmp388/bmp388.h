#ifndef __BMP388_H__
#define __BMP388_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "sensor.h"
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#define BIT(n) (1 << (n))

#define BMP388_SPI_TIMEOUT 100

#define BMP388_CS_GPIO_PORT GPIOB
#define BMP388_CS_GPIO_PIN  GPIO_PIN_0

// I2C Device Addresses
#define BMP388_DEVICE_ADDR       0x76
#define BMP388_DEVICE_ADDR_WRITE ((BMP388_DEVICE_ADDR << 1) | 1)
#define BMP388_DEVICE_ADDR_READ  ((BMP388_DEVICE_ADDR << 1) | 0)

/* registers */
#define BMP388_REG_CHIPID       0x00
#define BMP388_REG_ERR_REG      0x02
#define BMP388_REG_STATUS       0x03
#define BMP388_REG_DATA0        0x04
#define BMP388_REG_DATA1        0x05
#define BMP388_REG_DATA2        0x06
#define BMP388_REG_DATA3        0x07
#define BMP388_REG_DATA4        0x08
#define BMP388_REG_DATA5        0x09
#define BMP388_REG_SENSORTIME0  0x0C
#define BMP388_REG_SENSORTIME1  0x0D
#define BMP388_REG_SENSORTIME2  0x0E
#define BMP388_REG_SENSORTIME3  0x0F
#define BMP388_REG_EVENT        0x10
#define BMP388_REG_INT_STATUS   0x11
#define BMP388_REG_FIFO_LENGTH0 0x12
#define BMP388_REG_FIFO_LENGTH1 0x13
#define BMP388_REG_FIFO_DATA    0x14
#define BMP388_REG_FIFO_WTM0    0x15
#define BMP388_REG_FIFO_WTM1    0x16
#define BMP388_REG_FIFO_CONFIG1 0x17
#define BMP388_REG_FIFO_CONFIG2 0x18
#define BMP388_REG_INT_CTRL     0x19
#define BMP388_REG_IF_CONF      0x1A
#define BMP388_REG_PWR_CTRL     0x1B
#define BMP388_REG_OSR          0x1C
#define BMP388_REG_ODR          0x1D
#define BMP388_REG_CONFIG       0x1F
#define BMP388_REG_CALIB0       0x31
#define BMP388_REG_CMD          0x7E

/* BMP388_REG_CHIPID */
#define BMP388_CHIP_ID 0x50

/* BMP388_REG_STATUS */
#define BMP388_STATUS_FATAL_ERR  BIT(0)
#define BMP388_STATUS_CMD_ERR    BIT(1)
#define BMP388_STATUS_CONF_ERR   BIT(2)
#define BMP388_STATUS_CMD_RDY    BIT(4)
#define BMP388_STATUS_DRDY_PRESS BIT(5)
#define BMP388_STATUS_DRDY_TEMP  BIT(6)

/* BMP388_REG_INT_CTRL */
#define BMP388_INT_CTRL_DRDY_EN_POS  6
#define BMP388_INT_CTRL_DRDY_EN_MASK BIT(6)

/* BMP388_REG_PWR_CTRL */
#define BMP388_PWR_CTRL_PRESS_EN    BIT(0)
#define BMP388_PWR_CTRL_TEMP_EN     BIT(1)
#define BMP388_PWR_CTRL_MODE_POS    4
#define BMP388_PWR_CTRL_MODE_MASK   (0x03 << BMP388_PWR_CTRL_MODE_POS)
#define BMP388_PWR_CTRL_MODE_SLEEP  (0x00 << BMP388_PWR_CTRL_MODE_POS)
#define BMP388_PWR_CTRL_MODE_FORCED (0x01 << BMP388_PWR_CTRL_MODE_POS)
#define BMP388_PWR_CTRL_MODE_NORMAL (0x03 << BMP388_PWR_CTRL_MODE_POS)

/* BMP388_REG_OSR */
#define BMP388_ODR_POS  0
#define BMP388_ODR_MASK 0x1F

/* BMP388_REG_ODR */
#define BMP388_OSR_PRESSURE_POS  0
#define BMP388_OSR_PRESSURE_MASK (0x07 << BMP388_OSR_PRESSURE_POS)
#define BMP388_OSR_TEMP_POS      3
#define BMP388_OSR_TEMP_MASK     (0x07 << BMP388_OSR_TEMP_POS)

/* BMP388_REG_CONFIG */
#define BMP388_IIR_FILTER_POS  1
#define BMP388_IIR_FILTER_MASK (0x7 << BMP388_IIR_FILTER_POS)

/* BMP388_REG_CMD */
#define BMP388_CMD_FIFO_FLUSH 0xB0
#define BMP388_CMD_SOFT_RESET 0xB6

/* default PWR_CTRL settings */
#define BMP388_PWR_CTRL_ON                                                                         \
	(BMP388_PWR_CTRL_PRESS_EN | BMP388_PWR_CTRL_TEMP_EN | BMP388_PWR_CTRL_MODE_NORMAL)
#define BMP388_PWR_CTRL_OFF 0

#define BMP388_SAMPLE_BUFFER_SIZE (6)

/** @brief BMP388 calibration data read from device */
struct bmp388_cal_data
{
	uint16_t t1;
	uint16_t t2;
	int8_t t3;
	int16_t p1;
	int16_t p2;
	int8_t p3;
	int8_t p4;
	uint16_t p5;
	uint16_t p6;
	int8_t p7;
	int8_t p8;
	int16_t p9;
	int8_t p10;
	int8_t p11;
} __attribute__((packed));

/** @brief BMP388 device configuration */
struct bmp388_config
{
	SPI_HandleTypeDef *hspi;
};

/** @brief BMP388 sampled data and calibration */
struct bmp388_data
{
	struct bmp388_cal_data cal;
	uint32_t press;
	uint32_t raw_temp;
	int64_t comp_temp;
};

/** @brief BMP388 device handle */
struct bmp388_device
{
	struct bmp388_config *config;
	struct bmp388_data *data;
};

/**
 * @brief Compute altitude from reference and static pressure (kPa).
 * @param press_0    Reference pressure at sea level (kPa)
 * @param press_static Static pressure (kPa)
 * @return Altitude in meters
 */
static inline float bmp388_calc_altitude(float press_0, float press_static)
{
	if (press_0 <= 0.0f)
	{
		return 0.0f;
	}

	const float A = 145366.45f;
	const float EXP = 0.190284f;

	float frac = press_static / press_0;
	return A * (1.0f - powf(frac, EXP));
}

int bmp388_init(const struct bmp388_device *dev);
int bmp388_sample_fetch(const struct bmp388_device *dev);
int bmp388_channel_get(const struct bmp388_device *dev, enum sensor_channel chan,
                       struct sensor_value *val);

#endif
