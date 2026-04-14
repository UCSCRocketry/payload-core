#include "spi_flash.h"
#include "log.h"
#include <string.h>

// Helper to pull CS Low
static void spi_flash_cs_low(void)
{
	// GPIO definition matches the schematic/main.h
	//  This will need to be configured correctly in the actual payload core environment
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // SPI1 CS example
}

// Helper to push CS High
static void spi_flash_cs_high(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

// Helper to wait until WIP (Write-In-Progress) clears
static int spi_flash_wait_wip(SPI_HandleTypeDef *hspi)
{
	uint8_t cmd = SPI_FLASH_CMD_RDSR;
	uint8_t status = 0xFF;
	uint32_t start_tick = HAL_GetTick();

	spi_flash_cs_low();
	if (HAL_SPI_Transmit(hspi, &cmd, 1, 100) != HAL_OK)
	{
		spi_flash_cs_high();
		return -1;
	}

	do
	{
		if (HAL_SPI_Receive(hspi, &status, 1, 100) != HAL_OK)
		{
			spi_flash_cs_high();
			return -1;
		}
		if ((HAL_GetTick() - start_tick) > 2000)
		{ // 2s timeout
			spi_flash_cs_high();
			return -2; // Timeout
		}
	} while (status & SPI_FLASH_SR_WIP);

	spi_flash_cs_high();
	return 0;
}

// Helper: Send Write Enable (WREN)
static int spi_flash_write_enable(SPI_HandleTypeDef *hspi)
{
	uint8_t cmd = SPI_FLASH_CMD_WREN;
	spi_flash_cs_low();
	int ret = HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	spi_flash_cs_high();
	return (ret == HAL_OK) ? 0 : -1;
}

int spi_flash_init(SPI_HandleTypeDef *hspi)
{
	uint8_t tx_cmd = SPI_FLASH_CMD_RDID;
	uint8_t rx_data[3] = { 0 };

	// 1. Read JEDEC ID
	spi_flash_cs_low();
	HAL_SPI_Transmit(hspi, &tx_cmd, 1, 100);
	HAL_SPI_Receive(hspi, rx_data, 3, 100);
	spi_flash_cs_high();

	// Macronix MX25L25645G ID should be: 0xC2 (Mfg), 0x20 (Memory Type), 0x19 (Capacity)
	if (rx_data[0] != 0xC2 || rx_data[1] != 0x20 || rx_data[2] != 0x19)
	{
		LOG_ERR("SPI Flash JEDEC ID check failed: %02x %02x %02x", rx_data[0], rx_data[1],
		        rx_data[2]);
		return -1;
	}
	LOG_INF("SPI Flash MX25L25645G Found.");

	// 2. Wake from deep power down (if necessary)
	// Could add 0xAB command here if it sleeps.

	// 3. Enter 4-Byte Address Mode (EN4B)
	spi_flash_write_enable(hspi);
	tx_cmd = SPI_FLASH_CMD_EN4B;
	spi_flash_cs_low();
	HAL_SPI_Transmit(hspi, &tx_cmd, 1, 100);
	spi_flash_cs_high();

	// Wait for the mode switch to complete
	if (spi_flash_wait_wip(hspi) != 0)
	{
		LOG_ERR("SPI Flash failed to enter 4-byte address mode.");
		return -2;
	}
	LOG_INF("SPI Flash ready in 4-byte addressing mode.");

	return 0;
}

int spi_flash_write_page(SPI_HandleTypeDef *hspi, uint32_t address, const uint8_t *data)
{
	if (spi_flash_write_enable(hspi) != 0)
	{
		return -1;
	}

	uint8_t tx_header[5];
	tx_header[0] = SPI_FLASH_CMD_PP_4B;
	tx_header[1] = (address >> 24) & 0xFF;
	tx_header[2] = (address >> 16) & 0xFF;
	tx_header[3] = (address >> 8) & 0xFF;
	tx_header[4] = address & 0xFF;

	spi_flash_cs_low();
	// Send 4-byte PP command + Address
	if (HAL_SPI_Transmit(hspi, tx_header, 5, 100) != HAL_OK)
	{
		spi_flash_cs_high();
		return -1;
	}
	// Send 256 bytes of data
	if (HAL_SPI_Transmit(hspi, (uint8_t *) data, SPI_FLASH_PAGE_SIZE, 500) != HAL_OK)
	{
		spi_flash_cs_high();
		return -1;
	}
	spi_flash_cs_high();

	// Block until write completes
	return spi_flash_wait_wip(hspi);
}

int spi_flash_erase_sector(SPI_HandleTypeDef *hspi, uint32_t sector_address)
{
	if (spi_flash_write_enable(hspi) != 0)
	{
		return -1;
	}

	uint8_t tx_header[5];
	tx_header[0] = SPI_FLASH_CMD_SE_4B;
	tx_header[1] = (sector_address >> 24) & 0xFF;
	tx_header[2] = (sector_address >> 16) & 0xFF;
	tx_header[3] = (sector_address >> 8) & 0xFF;
	tx_header[4] = sector_address & 0xFF;

	spi_flash_cs_low();
	if (HAL_SPI_Transmit(hspi, tx_header, 5, 100) != HAL_OK)
	{
		spi_flash_cs_high();
		return -1;
	}
	spi_flash_cs_high();

	// Block until erase completes (can take ~20-50ms)
	return spi_flash_wait_wip(hspi);
}

int spi_flash_read(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *buffer, uint32_t length)
{
	uint8_t tx_header[5];
	tx_header[0] = SPI_FLASH_CMD_READ_4B;
	tx_header[1] = (address >> 24) & 0xFF;
	tx_header[2] = (address >> 16) & 0xFF;
	tx_header[3] = (address >> 8) & 0xFF;
	tx_header[4] = address & 0xFF;

	// Fast Read (0x0C) would require dummy bytes. We just use normal Read (0x13) to avoid them,
	// assuming SPI clock is safely < 50MHz depending on the chip.

	spi_flash_cs_low();
	if (HAL_SPI_Transmit(hspi, tx_header, 5, 100) != HAL_OK)
	{
		spi_flash_cs_high();
		return -1;
	}
	if (HAL_SPI_Receive(hspi, buffer, length, 500) != HAL_OK)
	{
		spi_flash_cs_high();
		return -1;
	}
	spi_flash_cs_high();

	return 0;
}
