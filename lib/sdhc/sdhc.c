#include "sdhc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "errno.h"
#include <string.h>
#include "log.h"
#include "utils.h"

#define ANY_INST_REQUIRES_EXPLICIT_FF

#ifdef ANY_INST_REQUIRES_EXPLICIT_FF
static const uint8_t sdhc_ones[] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
#endif // ANY_INST_REQUIRES_EXPLICIT_FF

/* Receives a block of bytes */
static int sdhc_spi_rx(SPI_HandleTypeDef *hspi, uint8_t *buf, int len)
{
	int ret;
	HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_RESET);
	ret = HAL_SPI_TransmitReceive(hspi, (uint8_t *) sdhc_ones, buf, len, SDHC_SPI_TIMEOUT);
	HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_SET);
	return ret;
}

static int sdhc_spi_tx(SPI_HandleTypeDef *hspi, uint8_t *buf, int len)
{
	uint8_t tmp[len];
	int ret;
	HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_RESET);
	ret = HAL_SPI_TransmitReceive(hspi, buf, tmp, len, SDHC_SPI_TIMEOUT);
	HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_SET);
	return ret;
}

static int sdhc_spi_txrx(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, int len)
{
	int ret;
	HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_RESET);
	ret = HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, len, SDHC_SPI_TIMEOUT);
	HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_SET);
	return ret;
}

int sdhc_init_card(SPI_HandleTypeDef *hspi)
{
	// Do at least 74 clock pulses of blank data while unselected.
	static uint8_t dummy[10];
	HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_SET);
	return HAL_SPI_TransmitReceive(hspi, (uint8_t *) sdhc_ones, dummy, 10, SDHC_SPI_TIMEOUT);
}

/* Checks if SPI SD card is sending busy signal */
int sdhc_spi_card_busy(SPI_HandleTypeDef *hspi)
{
	int ret;
	uint8_t response;

	ret = sdhc_spi_rx(hspi, &response, 1);
	if (ret)
	{
		return -EIO;
	}

	if (response == 0xFF)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

/* Waits for SPI SD card to stop sending busy signal */
int sdhc_spi_wait_unbusy(SPI_HandleTypeDef *hspi, int timeout_ms, int interval_ms)
{
	int ret;
	uint8_t response;

	while (timeout_ms > 0)
	{
		ret = sdhc_spi_rx(hspi, &response, 1);
		if (ret)
		{
			return ret;
		}
		if (response == 0xFF)
		{
			return 0;
		}
		HAL_Delay(interval_ms);
		timeout_ms -= interval_ms;
	}
	return -ETIMEDOUT;
}

/* Read SD command from SPI response */
int sdhc_spi_response_get(struct sdhc_spi_device *dev, struct sdhc_command *cmd, int rx_len)
{
	const struct sdhc_spi_config *config = dev->config;
	struct sdhc_spi_data *dev_data = dev->data;
	uint8_t *response = dev_data->scratch;
	uint8_t *end = response + rx_len;
	int ret, timeout = cmd->timeout_ms;
	uint8_t i;

	/* First step is finding the first valid byte of the response.
	 * All SPI responses start with R1, which will have MSB of zero.
	 * we know we can ignore the first 7 bytes, which hold the command and
	 * initial "card ready" byte.
	 */
	response += 8;
	while (response < end && ((*response & SD_SPI_START) == SD_SPI_START))
	{
		response++;
	}
	if (response == end)
	{
		/* Some cards are slow, and need more time to respond. Continue
		 * with single byte reads until the card responds.
		 */
		response = dev_data->scratch;
		end = response + 1;
		while (timeout > 0)
		{
			ret = sdhc_spi_rx((SPI_HandleTypeDef *) config->hspi, response, 1);
			if (ret < 0)
			{
				return ret;
			}
			if (*response != 0xff)
			{
				break;
			}
			/* Delay for a bit, and poll the card again */
			HAL_Delay(10);
			timeout -= 10;
		}
		if (*response == 0xff)
		{
			return -ETIMEDOUT;
		}
	}
	/* Record R1 response */
	cmd->response[0] = *response++;
	/* Check response for error */
	if (cmd->response[0] != 0)
	{
		if (cmd->response[0] & (SD_SPI_R1PARAMETER_ERR | SD_SPI_R1ADDRESS_ERR))
		{
			return -EFAULT; /* Bad address */
		}
		else if (cmd->response[0] & (SD_SPI_R1ILLEGAL_CMD_ERR))
		{
			return -EINVAL; /* Invalid command */
		}
		else if (cmd->response[0] & (SD_SPI_R1CMD_CRC_ERR))
		{
			return -EILSEQ; /* Illegal byte sequence */
		}
		else if (cmd->response[0] & (SD_SPI_R1ERASE_SEQ_ERR | SD_SPI_R1ERASE_RESET))
		{
			return -EIO;
		}
		/* else IDLE_STATE bit is set, which is not an error, card is just resetting */
	}
	switch ((cmd->response_type & SDHC_SPI_RESPONSE_TYPE_MASK))
	{
	case SD_SPI_RSP_TYPE_R1:
		/* R1 response - one byte*/
		break;
	case SD_SPI_RSP_TYPE_R1b:
		/* R1b response - one byte plus busy signal */
		/* Read remaining bytes to see if card is still busy.
		 * card will be ready when it stops driving data out
		 * low.
		 */
		while (response < end && (*response == 0x0))
		{
			response++;
		}
		if (response == end)
		{
			response--;
			/* Periodically check busy line */
			ret = sdhc_spi_wait_unbusy((SPI_HandleTypeDef *) config->hspi, SPI_R1B_TIMEOUT_MS,
			                           1000);
		}
		break;
	case SD_SPI_RSP_TYPE_R2:
	case SD_SPI_RSP_TYPE_R5:
		/* R2/R5 response - R1 response + 1 byte*/
		if (response == end)
		{
			response = dev_data->scratch;
			end = response + 1;
			/* Read the next byte */
			ret = sdhc_spi_rx((SPI_HandleTypeDef *) config->hspi, response, 1);
			if (ret)
			{
				return ret;
			}
		}
		cmd->response[0] = (*response) << 8;
		break;
	case SD_SPI_RSP_TYPE_R3:
	case SD_SPI_RSP_TYPE_R4:
	case SD_SPI_RSP_TYPE_R7:
		/* R3/R4/R7 response - R1 response + 4 bytes */
		cmd->response[1] = 0;
		for (i = 0; i < 4; i++)
		{
			cmd->response[1] <<= 8;
			/* Read bytes of response */
			if (response == end)
			{
				response = dev_data->scratch;
				end = response + 1;
				/* Read the next byte */
				ret = sdhc_spi_rx((SPI_HandleTypeDef *) config->hspi, response, 1);
				if (ret)
				{
					return ret;
				}
			}
			cmd->response[1] |= *response++;
		}
		break;
	default:
		/* Other RSP types not supported */
		return -ENOTSUP;
	}
	return 0;
}

static uint8_t sdhc_crc7(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < len; i++)
	{
		uint8_t b = data[i];
		for (int bit = 7; bit >= 0; bit--)
		{
			uint8_t in_bit = (b >> bit) & 1U;
			uint8_t msb = (crc >> 6) & 1U;
			uint8_t fb = msb ^ in_bit;

			crc = (uint8_t) ((crc << 1) & 0x7FU);
			if (fb)
			{
				crc ^= 0x09U;
			}
		}
	}

	return crc & 0x7FU;
}

static uint16_t sdhc_crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0x0000;

	for (size_t j = 0; j < len; j++)
	{
		uint8_t byte = data[j];

		// Process MSB first
		for (int i = 0; i < 8; i++)
		{
			uint8_t data_bit = (byte >> (7 - i)) & 1;
			uint8_t crc_msb = (crc >> 15) & 1;

			crc <<= 1;

			if (crc_msb ^ data_bit)
			{
				crc ^= 0x1021;
			}
		}
	}

	return crc;
}

/* Send SD command using SPI */
int sdhc_spi_send_cmd(struct sdhc_spi_device *dev, struct sdhc_command *cmd, int data_present)
{
	const struct sdhc_spi_config *config = dev->config;
	struct sdhc_spi_data *dev_data = dev->data;
	int err;
	uint8_t *cmd_buf;

	/* To reduce overhead, we will send entire command in one SPI
	 * transaction. The packet takes the following format:
	 * - all ones byte to ensure card is ready
	 * - opcode byte (which includes start and transmission bits)
	 * - 4 bytes for argument
	 * - crc7 byte (with end bit)
	 * The SD card can take up to 8 bytes worth of SCLK cycles to respond.
	 * therefore, we provide 8 bytes of all ones, to read data from the card.
	 * the maximum spi response length is 5 bytes, so we provide an
	 * additional 5 bytes of data, leaving us with 13 bytes of 0xff.
	 * Finally, we send a padding byte of all 0xff, to ensure that
	 * the card receives at least one 0xff byte before next command.
	 */

	/* Note: we can discard CMD data as we send it,
	 * so reuse the TX buf as RX
	 */
	uint8_t *scratch = dev_data->scratch;
	size_t scratch_len = sizeof(dev_data->scratch);

	if (data_present)
	{
		/* We cannot send extra SCLK cycles with our command,
		 * since we'll miss the data the card responds with. We
		 * send one 0xff byte, six command bytes, two additional 0xff
		 * bytes, since the min value of NCR (see SD SPI timing
		 * diagrams) is one, and we know there will be an R1 response.
		 */
		scratch_len = SD_SPI_CMD_SIZE + 3;
	}

	memset(scratch, 0xFF, sizeof(dev_data->scratch));
	cmd_buf = scratch + 1; // skip one 0xFF padding byte for command

	/* Command packet holds the following bits:
	 * [47]: start bit, 0b0
	 * [46]: transmission bit, 0b1
	 * [45-40]: command index
	 * [39-8]: argument
	 * [7-1]: CRC
	 * [0]: end bit, 0b1
	 * Note that packets are sent MSB first.
	 */
	/* Add start bit, tx bit, and cmd opcode */
	cmd_buf[0] = (cmd->opcode & SD_SPI_CMD);
	cmd_buf[0] = ((cmd_buf[0] | SD_SPI_TX) & ~SD_SPI_START);

	/* Add argument (big endian) */
	sys_put_be32(cmd->arg, &cmd_buf[1]);

	/* Add CRC, and set LSB as the end bit */
	uint8_t crc = sdhc_crc7(&cmd_buf[0], 5);
	cmd_buf[SD_SPI_CMD_BODY_SIZE] = (crc & 0x7F) << 1 | 0x01;

	LOG_DBG("cmd%d arg 0x%x", cmd->opcode, cmd->arg);

	/* Set data, will lock SPI bus */
	LOG_INF("Send Command 0x%x", scratch);
	err = sdhc_spi_txrx((SPI_HandleTypeDef *) config->hspi, scratch, scratch, scratch_len);
	if (err != 0)
	{
		return err;
	}

	/* Read command response */
	return sdhc_spi_response_get(dev, cmd, scratch_len);
}

/* Skips bytes in SDHC data stream. */
static int sdhc_skip(const struct sdhc_spi_device *dev, uint8_t skip_val)
{
	const struct sdhc_spi_config *config = dev->config;
	uint8_t buf;
	int ret;
	uint32_t retries = SD_SPI_SKIP_RETRIES;

	do
	{
		ret = sdhc_spi_rx((SPI_HandleTypeDef *) config->hspi, &buf, sizeof(buf));
		if (ret)
		{
			return ret;
		}
	} while (buf == skip_val && retries--);
	if (retries == 0)
	{
		return -ETIMEDOUT;
	}
	/* Return first non-skipped value */
	return buf;
}

/* Handles reading data from SD SPI device */
int sdhc_spi_read_data(const struct sdhc_spi_device *dev, struct sdhc_data *data)
{
	const struct sdhc_spi_config *config = dev->config;
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) config->hspi;
	uint8_t *read_location = data->data;
	uint32_t remaining = data->blocks;
	int ret;
	uint8_t crc[SD_SPI_CRC16_SIZE + 1];

#ifdef ANY_INST_REQUIRES_EXPLICIT_FF
	/* If the driver requires explicit 0xFF bytes on receive, we
	 * are limited to receiving the size of the sdhc_ones buffer
	 */
	if (data->block_size > sizeof(sdhc_ones))
	{
		return -ENOTSUP;
	}
#endif

	/* Read bytes until data stream starts. SD will send 0xff until
	 * data is available
	 */
	ret = sdhc_skip(dev, 0xff);
	if (ret < 0)
	{
		return ret;
	}
	/* Check token */
	if (ret != SD_SPI_TOKEN_SINGLE)
	{
		LOG_ERR("Check token fail");
		return -EIO;
	}

	/* Read blocks until we are out of data */
	while (remaining--)
	{
		ret = sdhc_spi_txrx(hspi, (uint8_t *) sdhc_ones, read_location, (int) data->block_size);
		if (ret != HAL_OK)
		{
			LOG_ERR("Data read failed");
			return -EIO;
		}
		/* Read CRC16 plus one end byte */
		ret = sdhc_spi_rx(hspi, crc, sizeof(crc));
		if (ret != HAL_OK)
		{
			LOG_ERR("CRC read failed");
			return -EIO;
		}
		if (sdhc_crc16(read_location, data->block_size) != sys_get_be16(crc))
		{
			LOG_ERR("Bad data CRC");
			return -EILSEQ;
		}
		read_location += data->block_size;
		if (remaining)
		{
			ret = sdhc_skip(dev, 0xff);
			if (ret != SD_SPI_TOKEN_SINGLE)
			{
				LOG_ERR("Bad token");
				return -EIO;
			}
		}
	}
	return 0;
}

/* Sends one write block (token + data + CRC) with CS held low. */
static int sdhc_spi_write_block(SPI_HandleTypeDef *hspi, uint8_t token, const uint8_t *block,
                                uint16_t block_size, const uint8_t *crc)
{
	HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_RESET);
	// Send token (7.3.3)
	if (HAL_SPI_Transmit(hspi, &token, 1, SDHC_SPI_TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_SET);
		return -EIO;
	}
	// Send block
	if (HAL_SPI_Transmit(hspi, (uint8_t *) block, block_size, SDHC_SPI_TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_SET);
		return -EIO;
	}
	// Send CRC16
	if (HAL_SPI_Transmit(hspi, (uint8_t *) crc, SD_SPI_CRC16_SIZE, SDHC_SPI_TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_SET);
		return -EIO;
	}
	HAL_GPIO_WritePin(SDHC_CS_GPIO_PORT, SDHC_CS_GPIO_PIN, GPIO_PIN_SET);
	return 0;
}

/* Handles writing data to SD SPI device */
int sdhc_spi_write_data(const struct sdhc_spi_device *dev, struct sdhc_data *data)
{
	const struct sdhc_spi_config *config = dev->config;
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) config->hspi;
	int ret;
	uint8_t token, resp;
	uint8_t *write_location = data->data;
	uint8_t crc[SD_SPI_CRC16_SIZE];
	uint32_t remaining = data->blocks;

	/* Single block uses 0xFE, multi-block uses 0xFC */
	if (remaining > 1)
	{
		token = SD_SPI_TOKEN_MULTI_WRITE;
	}
	else
	{
		token = SD_SPI_TOKEN_SINGLE;
	}

	while (remaining--)
	{
		sys_put_be16(sdhc_crc16(write_location, data->block_size), crc);
		ret = sdhc_spi_write_block(hspi, token, write_location, (uint16_t) data->block_size, crc);
		if (ret)
		{
			return ret;
		}
		ret = sdhc_spi_rx(hspi, &resp, sizeof(resp));
		if (ret != HAL_OK)
		{
			return -EIO;
		}
		if ((resp & 0xF) != SD_SPI_RESPONSE_ACCEPTED)
		{
			if ((resp & 0xF) == SD_SPI_RESPONSE_CRC_ERR)
			{
				return -EILSEQ;
			}
			if ((resp & 0xF) == SD_SPI_RESPONSE_WRITE_ERR)
			{
				return -EIO;
			}
			LOG_DBG("Unknown write response token 0x%x", resp);
			return -EIO;
		}
		write_location += data->block_size;
		ret = sdhc_spi_wait_unbusy(hspi, data->timeout_ms, 0);
		if (ret)
		{
			return ret;
		}
	}
	if (data->blocks > 1)
	{
		token = SD_SPI_TOKEN_STOP_TRAN;
		ret = sdhc_spi_tx(hspi, &token, 1);
		if (ret != HAL_OK)
		{
			return -EIO;
		}
		ret = sdhc_spi_wait_unbusy(hspi, data->timeout_ms, 0);
		if (ret)
		{
			return ret;
		}
	}
	return 0;
}