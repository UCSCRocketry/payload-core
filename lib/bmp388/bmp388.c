#include "bmp388.h"
#include "stm32f4xx_hal_gpio.h"
#include "utils.h"
#include "log.h"

static HAL_StatusTypeDef bmp388_spi_mem_read(SPI_HandleTypeDef *hspi, uint16_t addr, uint8_t *pData,
                                             uint16_t Size)
{
	HAL_StatusTypeDef ret;
	uint8_t addr_mod = (addr & 0x7F) | (1U << 7U);
	HAL_GPIO_WritePin(BMP388_CS_GPIO_PORT, BMP388_CS_GPIO_PIN, GPIO_PIN_RESET);
	ret = HAL_SPI_Transmit(hspi, &addr_mod, 1, BMP388_SPI_TIMEOUT);
	if (ret != HAL_OK)
	{
		return ret;
	}
	ret = HAL_SPI_Receive(hspi, pData, 1, BMP388_SPI_TIMEOUT);
	if (ret != HAL_OK)
	{
		return ret;
	}
	ret = HAL_SPI_Receive(hspi, pData, Size, BMP388_SPI_TIMEOUT);
	HAL_GPIO_WritePin(BMP388_CS_GPIO_PORT, BMP388_CS_GPIO_PIN, GPIO_PIN_SET);
	return ret;
}

static HAL_StatusTypeDef bmp388_spi_mem_write(SPI_HandleTypeDef *hspi, uint16_t addr, uint8_t Data,
                                              uint16_t Size)
{
	HAL_StatusTypeDef ret;
	uint8_t addr_mod = (addr & 0x7F);
	HAL_GPIO_WritePin(BMP388_CS_GPIO_PORT, BMP388_CS_GPIO_PIN, GPIO_PIN_RESET);
	ret = HAL_SPI_Transmit(hspi, &addr_mod, 1, BMP388_SPI_TIMEOUT);
	if (ret != HAL_OK)
	{
		return ret;
	}
	ret = HAL_SPI_Transmit(hspi, &Data, Size, BMP388_SPI_TIMEOUT);
	HAL_GPIO_WritePin(BMP388_CS_GPIO_PORT, BMP388_CS_GPIO_PIN, GPIO_PIN_SET);
	return ret;
}

static int bmp388_fetch_calibration_data(SPI_HandleTypeDef *hspi, struct bmp388_cal_data *cal)
{
	if (bmp388_spi_mem_read(hspi, BMP388_REG_CALIB0, (uint8_t *) cal, sizeof(*cal)) != HAL_OK)
	{
		return -1;
	}

	cal->t1 = sys_get_le16((const uint8_t *) &cal->t1);
	cal->t2 = sys_get_le16((const uint8_t *) &cal->t2);
	cal->p1 = (int16_t) sys_get_le16((const uint8_t *) &cal->p1);
	cal->p2 = (int16_t) sys_get_le16((const uint8_t *) &cal->p2);
	cal->p5 = sys_get_le16((const uint8_t *) &cal->p5);
	cal->p6 = sys_get_le16((const uint8_t *) &cal->p6);
	cal->p9 = (int16_t) sys_get_le16((const uint8_t *) &cal->p9);

	return 0;
}

static void bmp388_comp_temp(struct bmp388_cal_data *cal, struct bmp388_data *data)
{
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;

	partial_data1 = (int64_t) data->raw_temp - (256 * cal->t1);
	partial_data2 = cal->t2 * partial_data1;
	partial_data3 = (int64_t) (partial_data1 * partial_data1);
	partial_data4 = partial_data3 * cal->t3;
	partial_data5 = (int64_t) (partial_data2 * 262144) + partial_data4;

	data->comp_temp = partial_data5 / 4294967296;
}

static uint64_t bmp388_comp_press(struct bmp388_cal_data *cal, struct bmp388_data *data)
{
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	int64_t offset;
	int64_t sensitivity;

	int64_t comp_temp = data->comp_temp;
	int64_t raw_press = data->press;

	partial_data1 = comp_temp * comp_temp;
	partial_data2 = partial_data1 / 64;
	partial_data3 = (partial_data2 * comp_temp) / 256;
	partial_data4 = (cal->p8 * partial_data3) / 32;
	partial_data5 = (cal->p7 * partial_data1) * 16;
	partial_data6 = (cal->p6 * comp_temp) * 4194304;
	offset = (cal->p5 * 140737488355328LL) + partial_data4 + partial_data5 + partial_data6;
	partial_data2 = (cal->p4 * partial_data3) / 32;
	partial_data4 = (cal->p3 * partial_data1) * 4;
	partial_data5 = (cal->p2 - 16384) * comp_temp * 2097152;
	sensitivity = ((cal->p1 - 16384) * 70368744177664LL) + partial_data2 + partial_data4
	              + partial_data5;
	partial_data1 = (sensitivity / 16777216) * raw_press;
	partial_data2 = cal->p10 * comp_temp;
	partial_data3 = partial_data2 + (65536 * cal->p9);
	partial_data4 = (partial_data3 * raw_press) / 8192;
	partial_data5 = (raw_press * (partial_data4 / 10)) / 512;
	partial_data5 = partial_data5 * 10;
	partial_data6 = (int64_t) raw_press * (int64_t) raw_press;
	partial_data2 = (cal->p11 * partial_data6) / 65536;
	partial_data3 = (partial_data2 * raw_press) / 128;
	partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;

	return (uint64_t) ((partial_data4 * 25) / 1099511627776ULL);
}

int bmp388_init(const struct bmp388_device *dev)
{
	SPI_HandleTypeDef *hspi;
	uint8_t val = 0U;

	if (dev == NULL || dev->config == NULL || dev->data == NULL)
	{
		return -1;
	}

	hspi = dev->config->hspi;

	HAL_GPIO_WritePin(BMP388_CS_GPIO_PORT, BMP388_CS_GPIO_PIN, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(BMP388_CS_GPIO_PORT, BMP388_CS_GPIO_PIN, GPIO_PIN_SET);

	if (bmp388_spi_mem_write(hspi, BMP388_REG_CMD, BMP388_CMD_SOFT_RESET, 1) != HAL_OK)
	{
		LOG_ERR("Failed to reboot bmp388 device");
		return -2;
	}

	HAL_Delay(200);

	if (bmp388_spi_mem_read(hspi, BMP388_REG_CHIPID, &val, 1) != HAL_OK)
	{
		LOG_ERR("Failed to read bmp388 chip ID");
		return -3;
	}

	if (val != BMP388_CHIP_ID)
	{
		LOG_ERR("bmp388 chip ID not correct, expected 0x%x got 0x%x", BMP388_CHIP_ID, val);
		return -4;
	}

	if (bmp388_spi_mem_write(hspi, BMP388_REG_PWR_CTRL, BMP388_PWR_CTRL_ON, 1) != HAL_OK)
	{
		LOG_ERR("Failed to enable sensing on bmp388");
		return -5;
	}

	if (bmp388_spi_mem_read(hspi, BMP388_REG_ERR_REG, &val, 1) != HAL_OK)
	{
		LOG_ERR("Failed to read error register on bmp388");
		return -6;
	}

	if (val & BMP388_STATUS_CONF_ERR)
	{
		LOG_ERR("Failed to properly initialize bmp388");
		return -7;
	}

	if (bmp388_fetch_calibration_data(hspi, &dev->data->cal) != 0)
	{
		LOG_ERR("Failed to fetch bmp388 calibration data");
		return -8;
	}

	LOG_INF("BMP388 Initialized");
	return 0;
}

int bmp388_sample_fetch(const struct bmp388_device *dev)
{
	SPI_HandleTypeDef *hspi;
	uint8_t raw[BMP388_SAMPLE_BUFFER_SIZE] = { 0 };
	HAL_StatusTypeDef ret;

	if (dev == NULL || dev->config == NULL || dev->data == NULL)
	{
		return -1;
	}

	hspi = dev->config->hspi;

	while ((raw[0] & BMP388_STATUS_DRDY_PRESS) == 0U)
	{
		ret = bmp388_spi_mem_read(hspi, BMP388_REG_STATUS, raw, 1);
		if (ret != HAL_OK)
		{
			return (int) ret;
		}
	}

	ret = bmp388_spi_mem_read(hspi, BMP388_REG_DATA0, raw, BMP388_SAMPLE_BUFFER_SIZE);
	if (ret != HAL_OK)
	{
		return (int) ret;
	}

	dev->data->press = (uint32_t) (raw[0] | (raw[1] << 8) | (raw[2] << 16));
	dev->data->raw_temp = (uint32_t) (raw[3] | (raw[4] << 8) | (raw[5] << 16));
	dev->data->comp_temp = 0;

	return 0;
}

int bmp388_channel_get(const struct bmp388_device *dev, enum sensor_channel chan,
                       struct sensor_value *val)
{
	struct bmp388_cal_data *cal;
	struct bmp388_data *data;
	uint64_t tmp;

	if (dev == NULL || dev->data == NULL || val == NULL)
	{
		return -1;
	}

	cal = &dev->data->cal;
	data = dev->data;

	if (data->comp_temp == 0)
	{
		bmp388_comp_temp(cal, data);
	}

	switch (chan)
	{
	case SENSOR_CHAN_DIE_TEMP:
	case SENSOR_CHAN_AMBIENT_TEMP:
		tmp = (data->comp_temp * 250000ULL) / 16384;
		val->val1 = (int32_t) (tmp / 1000000);
		val->val2 = (int32_t) (tmp % 1000000);
		break;
	case SENSOR_CHAN_PRESS:
		tmp = bmp388_comp_press(cal, data);
		val->val1 = (int32_t) (tmp / 100000);
		val->val2 = (int32_t) ((tmp % 100000) * 10);
		break;
	default:
        LOG_ERR("bmp388 invalid sensor channel");
		return -1;
	}

	return 0;
}
