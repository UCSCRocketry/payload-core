/**
 * @file dump.c
 * @brief Functions to dump the flash to SD card and verify
 */

// DATA.BIN:
//      bytes 0-3   total page count in uint32_t
//      bytes 4+    raw flash pages, each 256 bytes

#include "dump.h"
#include "../lib/sdhc/sdhc.h"
#include "../lib/fatfs/ff.h"
#include "../lib/common/log.h"
#include "../lib/common/errno.h"
#include "stm32f4xx_hal.h"

extern CRC_HandleTypeDef hcrc;
extern SPI_HandleTypeDef hspi2;

struct sdhc_spi_device sd_dev;

/**
 * @brief Dumps flash content to SD card
 *
 * Transfers data from the SPI flash chip onto a binary file on the SD card,
 * verifies the data using CRC, and formats the flash if verified.
 *
 * @warning This has the possibility of formatting contents of the flash chip.
 *
 * @param spif Flash handle (must already be initialised).
 * @return status int
 */
int dump_and_format_flash(SPIF_HandleTypeDef *spif)
{
    // ******** Init devices ********
	static struct sdhc_spi_config sd_cfg = {
		.hspi = &hspi2,
		.spi_max_freq = 20000000,
	};
	static struct sdhc_spi_data sd_dat = { 0 };

	static FATFS fs;

	sd_dev.config = &sd_cfg;
	sd_dev.data = &sd_dat;

	if (sdhc_init(&sd_dev))
	{
		LOG_ERR("Dump: SD card init failed");
		return -EIO;
	}

	if (f_mount(&fs, "", 0) != FR_OK)
	{
		LOG_ERR("Dump: filesystem mount failed");
		return -EIO;
	}
    
    // ******** Perform Dump Operations ********

	uint32_t num_pages = spif->PageCnt;
	uint8_t page_buf[SPIF_PAGE_SIZE];
	uint32_t crc_wr = 0;
	uint32_t crc_rd = 0;
    int ret = 1;
	FIL file;
	UINT num_xferred;

    // Open the file to write
	if (f_open(&file, "DATA.BIN", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
		LOG_ERR("Dump: cannot open DATA.BIN for write");
		goto unmount;
	}

    // // Write the header
	// if (f_write(&file, &num_pages, sizeof(num_pages), &num_xferred) != FR_OK
	//     || num_xferred != sizeof(num_pages))
	// {
	// 	LOG_ERR("Dump: header write failed");
	// 	(void) f_close(&file);
	// 	goto unmount;
	// }

    // Perform the data transfer
	__HAL_CRC_DR_RESET(&hcrc);
	for (uint32_t i = 0; i < num_pages; i++)
	{
        // Read from flash
		if (!SPIF_ReadPage(spif, i, page_buf, SPIF_PAGE_SIZE, 0))
		{
			LOG_ERR("Dump: flash read error at page %lu", i);
			(void) f_close(&file);
			goto unmount;
		}

        // Accumulate CRC32 value
		crc_wr = HAL_CRC_Accumulate(&hcrc, (uint32_t *) page_buf,
		                               SPIF_PAGE_SIZE / sizeof(uint32_t));

        // Write to SD
		if (f_write(&file, page_buf, SPIF_PAGE_SIZE, &num_xferred) != FR_OK || num_xferred != SPIF_PAGE_SIZE)
		{
			LOG_ERR("Dump: SD write error at page %lu", i);
			(void) f_close(&file);
			goto unmount;
		}
	}
	(void) f_close(&file);
	LOG_INF("Dump: wrote %lu pages, CRC=0x%08lX", num_pages, crc_wr);

	// ******** Verify the dump ********
    // Open the file
	if (f_open(&file, "DATA.BIN", FA_READ) != FR_OK)
	{
		LOG_ERR("Dump: cannot open DATA.BIN for verify");
		goto unmount;
	}

    // // Skip over the header
	// if (f_lseek(&file, sizeof(num_pages)) != FR_OK)
	// {
	// 	LOG_ERR("Dump: lseek failed during verify");
	// 	(void) f_close(&file);
	// 	goto unmount;
	// }

    // Perform the CRC calculation
	__HAL_CRC_DR_RESET(&hcrc);
	for (uint32_t p = 0; p < num_pages; p++)
	{
        // Read
		if (f_read(&file, page_buf, SPIF_PAGE_SIZE, &num_xferred) != FR_OK || num_xferred != SPIF_PAGE_SIZE)
		{
			LOG_ERR("Dump: read error at page %lu during verify", p);
			(void) f_close(&file);
			goto unmount;
		}

        // And accumulate
		crc_rd = HAL_CRC_Accumulate(&hcrc, (uint32_t *) page_buf,
		                              SPIF_PAGE_SIZE / sizeof(uint32_t));
	}
	(void) f_close(&file);
	LOG_INF("Dump: verify CRC=0x%08lX", crc_rd);

    // Check CRC is matching
	if (crc_wr != crc_rd)
	{
		LOG_ERR("Dump: CRC mismatch - flash NOT erased (write=0x%08lX read=0x%08lX)", crc_wr,
		        crc_rd);
		goto unmount;
	}

	// Erase Flash
	LOG_INF("Dump: CRC OK. Erasing flash chip...");
	if (!SPIF_EraseChip(spif))
	{
		LOG_ERR("Dump: flash chip erase failed");
		goto unmount;
	}
	LOG_INF("Dump: flash erased. Done.");
	ret = 0;

unmount:
	if (f_mount(NULL, "", 0) == FR_OK)
	{
		LOG_INF("Dump: SD Card unmounted, safe to remove");
	}
	else
	{
		LOG_INF("Dump: Error unmounting SD Card, take caution during removal");
	}
	

	return ret;
}
