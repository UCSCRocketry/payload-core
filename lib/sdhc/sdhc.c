#include "sdhc.h"
#include "errno.h"
#include "sdhc_spi.h"
#include "sdspec.h"

int sdhc_init(struct sdhc_spi_device *dev)
{
	const struct sdhc_spi_config *config = dev->config;
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) config->hspi;
	struct sdhc_command cmd = { 0 };
	int data_present = 0;
	int ret;
	uint8_t attempts;

	// This performs power up
	if (sdhc_init_card(hspi) != HAL_OK)
	{
        LOG_ERR("SD Initialization: Power up fail");
		return -EIO;
	}
	if (sdhc_spi_wait_unbusy(hspi, 100, 100))
	{
        LOG_ERR("SD Initialization: Time out");
		return -ETIMEDOUT;
	}

	// Send CMD0
	cmd.opcode = SD_GO_IDLE_STATE;
	cmd.arg = 0; // stuff bits
	cmd.response_type = SD_SPI_RSP_TYPE_R1;

	ret = sdhc_spi_send_cmd(dev, &cmd, data_present);
	if (ret != 0)
	{
		LOG_ERR("SD Initialization: CMD0 (GO_IDLE_STATE) failed with return value %d", ret);
		return -EIO;
	}
	LOG_DBG("CMD0 sent: response[0]=0x%02x", cmd.response[0]);

	// Turn CRC MODE on
	cmd.opcode = SD_SPI_CRC_ON_OFF;
	cmd.arg = 1; // [0:0]: 1 = on, 0 = off
	cmd.response_type = SD_SPI_RSP_TYPE_R1;

	ret = sdhc_spi_send_cmd(dev, &cmd, data_present);
	if (ret != 0)
	{
		LOG_ERR("SD Initialization: CMD59 (CRC_ON_OFF) failed with return value %d", ret);
		return -EIO;
	}
	LOG_DBG("CMD59 sent: response[0]=0x%02x", cmd.response[0]);

	// Send CMD8 to check voltage information
	cmd.opcode = SD_SEND_IF_COND;
	// [11:8] supply voltage 2.7-3.6V, [7:0] check pattern
	cmd.arg = (1U << 8) | (0xDE);
	cmd.response_type = SD_SPI_RSP_TYPE_R7;

	ret = sdhc_spi_send_cmd(dev, &cmd, data_present);
	if (ret != 0)
	{
		LOG_ERR("SD Initialization: CMD8 (SEND_IF_COND) failed with return value %d", ret);
		return -EIO;
	}
#ifndef LOG_NODEBUG
	for (size_t i = 0; i < 4; i++)
	{
		LOG_DBG("CMD8 sent: response[%u]=0x%02x", i, cmd.response[i]);
	}
#endif

	// Check CMD8 R7 response
	if (cmd.response[1] != 0x1DE) // accepts 2.7-3.6V, writes back same check pattern
	{
		LOG_ERR("SD Initialization: CMD8 response 0x%x is invalid", cmd.response[1]);
		return -ENOTSUP;
	}

	// Initial send READ_OCR (CMD58) command
	cmd.opcode = SD_SPI_READ_OCR;
	cmd.arg = 0; // stuff bits
	cmd.response_type = SD_SPI_RSP_TYPE_R3;

	ret = sdhc_spi_send_cmd(dev, &cmd, data_present);
	if (ret != 0)
	{
		LOG_ERR("SD Initialization: CMD58 failed with return value %d", ret);
		return -EIO;
	}
	LOG_DBG("CMD58 sent: response[0]=0x%02x", cmd.response[0]);

	attempts = 0;
	// Keep sending ACMD41 until SD card is not busy anymore (idle
	// bit in response is low)
	do
	{
		// SEND CMD55 (APP_CMD) to signify next command is application command
		cmd.opcode = SD_APP_CMD;
		cmd.arg = 0;
		cmd.response_type = SD_SPI_RSP_TYPE_R1;

		ret = sdhc_spi_send_cmd(dev, &cmd, data_present);
		if (ret != 0)
		{
			LOG_ERR("SD Initialization: CMD55 failed with return value %d", ret);
			return -EIO;
		}
		LOG_DBG("CMD55 sent: response[0]=0x%02x", cmd.response[0]);

		// Send ACMD41 to set HCS (Host Capacity Support)
		cmd.opcode = SD_APP_SEND_OP_COND;
		cmd.arg = (1U << 30); // HCS bit
		cmd.response_type = SD_SPI_RSP_TYPE_R1;

		ret = sdhc_spi_send_cmd(dev, &cmd, data_present);
		if (ret != 0)
		{
			LOG_ERR("SD Initialization: ACMD41 failed with return value %d", ret);
			return -EIO;
		}
		LOG_DBG("ACMD41 sent: response[0]=0x%02x", cmd.response[0]);

		attempts++;
		if (attempts >= SDHC_MAX_ACMD41_SEND_ATTEMPTS)
		{
            LOG_ERR("SD Initialization: ACMD41 time out");
			return -ETIMEDOUT;
		}

	} while (cmd.response[0] & SD_SPI_R1IDLE_STATE);

	// Send READ_OCR (CMD58) command again to verify CCS bit is set
	cmd.opcode = SD_SPI_READ_OCR;
	cmd.arg = 0; // stuff bits
	cmd.response_type = SD_SPI_RSP_TYPE_R3;

	ret = sdhc_spi_send_cmd(dev, &cmd, data_present);
	if (ret != 0)
	{
		LOG_ERR("SD Initialization: CMD58 failed with return value %d", ret);
		return -EIO;
	}
	LOG_DBG("CMD58 sent: response[0]=0x%02x", cmd.response[0]);

	// Verify CCS bit is set
	if (!(cmd.response[1] & (1U << 30)))
	{
		LOG_ERR("SD Initialization: CCS bit is not set, device unsupported. \
            Are you using an SDSC card? Received response[1]=0x%0x",
		        cmd.response[1]);
		return -ENOTSUP;
	}

	LOG_INF("SD initialization successful");

	return 0;
}

int sdhc_write_data(struct sdhc_spi_device *dev, struct sdhc_data *data)
{
	struct sdhc_command cmd = { 0 };
	int data_present = 1;
	int ret;

	// Send either write single or write multiple block command
	cmd.opcode = (data->blocks > 1) ? SD_WRITE_MULTIPLE_BLOCK : SD_WRITE_SINGLE_BLOCK;
	cmd.arg = data->block_addr;
	cmd.response_type = SD_SPI_RSP_TYPE_R1;

	ret = sdhc_spi_send_cmd(dev, &cmd, data_present);
	if (ret != 0)
	{
		LOG_ERR("SDHC Write Data: Write block command send failed with return value %d", ret);
		return -EIO;
	}
	LOG_DBG("Write block command sent: response[0]=0x%02x", cmd.response[0]);

	// Send actual data
	ret = sdhc_spi_write_data(dev, data);
	if (ret != 0)
	{
		LOG_ERR("SDHC Write Data: Block transfer failed with return value %d", ret);
		return -EIO;
	}
	LOG_DBG("Write block data send successful");

	return 0;
}

int sdhc_read_data(struct sdhc_spi_device *dev, struct sdhc_data *data)
{
	struct sdhc_command cmd = { 0 };
	int data_present = 1;
	int ret;

	// Send either write single or write multiple block command
	cmd.opcode = (data->blocks > 1) ? SD_READ_MULTIPLE_BLOCK : SD_READ_SINGLE_BLOCK;
	cmd.arg = data->block_addr;
	cmd.response_type = SD_SPI_RSP_TYPE_R1;

	ret = sdhc_spi_send_cmd(dev, &cmd, data_present);
	if (ret != 0)
	{
		LOG_ERR("SDHC Read Data: Read block command send failed with return value %d", ret);
		return -EIO;
	}
	LOG_DBG("Read block command sent: response[0]=0x%02x", cmd.response[0]);

	ret = sdhc_spi_read_data(dev, data);
	if (ret != 0)
	{
		LOG_ERR("SDHC Read Data: Block transfer failed with return value %d", ret);
		return -EIO;
	}

	if (data->blocks > 1)
	{
		cmd.opcode = SD_STOP_TRANSMISSION;
		cmd.arg = 0;
		cmd.response_type = SD_SPI_RSP_TYPE_R1b;
		data_present = 0;

		ret = sdhc_spi_send_cmd(dev, &cmd, data_present);
		if (ret != 0)
		{
			LOG_ERR("SDHC Read Data: Stop transmission command send failed with return value %d",
			        ret);
		}
	}
	LOG_DBG("Read block data successful");
}