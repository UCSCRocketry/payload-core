#include "logger.h"
#include "log.h"
#include <string.h>

static SPI_HandleTypeDef *logger_hspi = NULL;

// The sequential raw flash address.
static uint32_t current_flash_addr = 0x00000000;

// Maximum addresses for a 256 Mbit (32 MB) Chip
#define MAX_FLASH_ADDR (32 * 1024 * 1024)

// Double Buffers
static page_buffer_t buffer_a;
static page_buffer_t buffer_b;

// State pointers
static page_buffer_t *active_buffer = &buffer_a;
static page_buffer_t *flush_buffer = &buffer_b;
static uint8_t record_index = 0;
static uint8_t write_in_progress = 0; // Simulated DMA flag tracking

int logger_init(SPI_HandleTypeDef *hspi)
{
	if (!hspi)
	{
		return -1;
	}
	logger_hspi = hspi;

	if (spi_flash_init(hspi) != 0)
	{
		LOG_ERR("Logger: SPI Flash failed to initialize.");
		return -1;
	}

	// Usually, you would implement a routine here that reads the flash
	// starting from 0x00000000 until it finds a blank 0xFFFFFFFF page
	// to figure out where the last flight left off.
	// For now, assume a clean wipe before flight.
	current_flash_addr = 0x00000000;

	// Auto-erase the very first block
	spi_flash_erase_sector(logger_hspi, current_flash_addr);

	memset(&buffer_a, 0xFF, sizeof(page_buffer_t));
	memset(&buffer_b, 0xFF, sizeof(page_buffer_t));
	active_buffer = &buffer_a;
	record_index = 0;

	return 0;
}

// Internal function to swap buffers and trigger SPI write
static void logger_trigger_flush(void)
{
	// 1. Swap active and flush buffers
	page_buffer_t *temp = active_buffer;
	active_buffer = flush_buffer;
	flush_buffer = temp;
	record_index = 0; // Reset active buffer

	// 2. Clear the new active buffer (pad with 0xFF)
	memset(active_buffer, 0xFF, sizeof(page_buffer_t));

	// 3. Mark the background write as in progress (For DMA)
	write_in_progress = 1;

	// 4. Trigger DMA SPI page program (blocking for now to mock DMA)
	if (spi_flash_write_page(logger_hspi, current_flash_addr, (const uint8_t *) flush_buffer) != 0)
	{
		LOG_ERR("Logger: SPI Flash write failed at 0x%08X", current_flash_addr);
	}
	write_in_progress = 0;

	// 5. Update Address
	current_flash_addr += SPI_FLASH_PAGE_SIZE;

	// 6. Sector management - if we just crossed a 4KB boundary, pre-erase the NEXT sector
	if (current_flash_addr % SPI_FLASH_SECTOR_SIZE == 0)
	{
		if (current_flash_addr < MAX_FLASH_ADDR)
		{
			spi_flash_erase_sector(logger_hspi, current_flash_addr);
		}
	}
}

int logger_log(const flight_record_t *record)
{
	if (!logger_hspi || current_flash_addr >= MAX_FLASH_ADDR)
	{
		return -1; // Not initialized or Flash is full
	}

	// If DMA is still writing the previous buffer and we somehow filled this one
	// up too fast, we'd have to drop the record or block.
	if (record_index >= RECORDS_PER_PAGE && write_in_progress)
	{
		LOG_WRN("Logger: Buffer overrun! DMA too slow.");
		return -1;
	}

	// Copy record into active buffer
	memcpy(&active_buffer->records[record_index], record, sizeof(flight_record_t));
	record_index++;

	// If buffer is full, trigger background flush
	if (record_index >= RECORDS_PER_PAGE)
	{
		logger_trigger_flush();
	}

	return 0;
}

int logger_flush(void)
{
	if (!logger_hspi || record_index == 0)
	{
		return 0;
	}

	// Fill the remainder of the active page with 0xFF and write it out
	while (record_index < RECORDS_PER_PAGE)
	{
		memset(&active_buffer->records[record_index], 0xFF, sizeof(flight_record_t));
		record_index++;
	}

	logger_trigger_flush();
	return 0;
}
