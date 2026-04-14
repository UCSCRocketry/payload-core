#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

// MX25L25645G SPI Flash Constants
#define SPI_FLASH_PAGE_SIZE   256 // 256 bytes per page
#define SPI_FLASH_SECTOR_SIZE 4096 // 4KB sectors

// Standard SPI Flash Commands
#define SPI_FLASH_CMD_WREN 0x06 // Write Enable
#define SPI_FLASH_CMD_WRDI 0x04 // Write Disable
#define SPI_FLASH_CMD_RDSR 0x05 // Read Status Register
#define SPI_FLASH_CMD_RDID 0x9F // Read JEDEC ID
#define SPI_FLASH_CMD_EN4B 0xB7 // Enter 4-Byte Address Mode
#define SPI_FLASH_CMD_EX4B 0xE9 // Exit 4-Byte Address Mode

// 4-Byte Address Commands (Because it's a 256Mbit/32MB Chip)
#define SPI_FLASH_CMD_READ_4B 0x13 // Read Data (4-Byte Address)
#define SPI_FLASH_CMD_PP_4B   0x12 // Page Program (4-Byte Address)
#define SPI_FLASH_CMD_SE_4B   0x21 // Sector Erase 4KB (4-Byte Address)
#define SPI_FLASH_CMD_CE      0xC7 // Chip Erase

// Status Register Bits
#define SPI_FLASH_SR_WIP 0x01 // Write In Progress Bit

/**
 * @brief Initialize the SPI Flash chip.
 *        This will read the JEDEC ID to confirm communication,
 *        and place the chip into 4-Byte Addressing mode.
 * @param hspi Pointer to the SPI_HandleTypeDef structure.
 * @return 0 on success, negative error code on failure.
 */
int spi_flash_init(SPI_HandleTypeDef *hspi);

/**
 * @brief Write a full 256-byte page to the flash.
 *        This function auto-waits for WIP to clear before returning.
 * @param hspi Pointer to the SPI_HandleTypeDef.
 * @param address 32-bit flash address (must be a multiple of 256).
 * @param data Pointer to exactly 256 bytes of data.
 * @return 0 on success.
 */
int spi_flash_write_page(SPI_HandleTypeDef *hspi, uint32_t address, const uint8_t *data);

/**
 * @brief Erase a 4KB sector.
 *        This function auto-waits for WIP to clear before returning.
 * @param hspi Pointer to the SPI_HandleTypeDef.
 * @param sector_address A 32-bit address (must be a multiple of 4096).
 * @return 0 on success.
 */
int spi_flash_erase_sector(SPI_HandleTypeDef *hspi, uint32_t sector_address);

/**
 * @brief Read data from the flash.
 * @param hspi Pointer to the SPI_HandleTypeDef.
 * @param address 32-bit flash address to start reading from.
 * @param buffer Pointer to destination data buffer.
 * @param length Number of bytes to read.
 * @return 0 on success.
 */
int spi_flash_read(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *buffer, uint32_t length);

#endif // __SPI_FLASH_H__
