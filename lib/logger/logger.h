#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "spi_flash.h"

// 32-Byte Flight Record
typedef struct __attribute__((packed))
{
	uint32_t timestamp_ms; // 4
	int16_t accel[3]; // 6
	int16_t gyro[3]; // 6
	int16_t mag[3]; // 6
	uint32_t pressure; // 4
	int16_t temperature; // 2
	int16_t altitude; // 2
	uint8_t crc; // 1 (Simple XOR or CRC8)
	uint8_t reserved; // 1 (Pad to 32 bytes)
} flight_record_t;

// 256-Byte Page Buffer holds exactly 8 records
#define RECORDS_PER_PAGE (SPI_FLASH_PAGE_SIZE / sizeof(flight_record_t))

typedef struct
{
	flight_record_t records[RECORDS_PER_PAGE];
} page_buffer_t;

/**
 * @brief Initializes the flash logger.
 *        Reads the flash to find the next blank address.
 * @param hspi SPI Handle for the flash chip.
 * @return 0 on success.
 */
int logger_init(SPI_HandleTypeDef *hspi);

/**
 * @brief Logs a single 32-byte flight record.
 *        If the active buffer fills up, it swaps to the flush buffer
 *        and triggers a non-blocking DMA SPI write.
 * @param record Pointer to the record to log.
 * @return 0 on success.
 */
int logger_log(const flight_record_t *record);

/**
 * @brief Flushes any remaining data in the active buffer to flash
 *        before shutting down.
 * @return 0 on success.
 */
int logger_flush(void);

#endif // __LOGGER_H__
