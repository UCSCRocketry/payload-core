#ifndef __SDHC_H__
#define __SDHC_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "sdspec.h"

#define SDHC_CS_GPIO_PIN  GPIO_PIN_2
#define SDHC_CS_GPIO_PORT GPIOB

#define MAX_CMD_READ        21
#define SPI_R1B_TIMEOUT_MS  3000
#define SD_SPI_SKIP_RETRIES 1000000

#define SDHC_SPI_TIMEOUT 500

/* Byte length of SD SPI mode command */
#define SD_SPI_CMD_SIZE      6
#define SD_SPI_CMD_BODY_SIZE (SD_SPI_CMD_SIZE - 1)
/* Byte length of CRC16 appended to data blocks in SPI mode */
#define SD_SPI_CRC16_SIZE 2

/* SPI Command flags */
#define SD_SPI_START 0x80
#define SD_SPI_TX    0x40
#define SD_SPI_CMD   0x3F

/* SPI Data block tokens */
#define SD_SPI_TOKEN_SINGLE      0xFE
#define SD_SPI_TOKEN_MULTI_WRITE 0xFC
#define SD_SPI_TOKEN_STOP_TRAN   0xFD

/* SPI Data block responses */
#define SD_SPI_RESPONSE_ACCEPTED  0x05
#define SD_SPI_RESPONSE_CRC_ERR   0x0B
#define SD_SPI_RESPONSE_WRITE_ERR 0x0C

/* Masks used in SD interface condition query (CMD8) */
#define SD_IF_COND_VHS_MASK (0x0F << 8)
#define SD_IF_COND_VHS_3V3  BIT(8)
#define SD_IF_COND_CHECK    0xAA

/**
 * @brief SD host controller command structure
 *
 * This command structure is used to send command requests to an SD
 * host controller, which will be sent to SD devices.
 */
struct sdhc_command
{
	uint32_t opcode; /*!< SD Host specification CMD index */
	uint32_t arg; /*!< SD host specification argument */
	uint32_t response[4]; /*!< SD card response field */
	uint32_t response_type; /*!< Expected SD response type */
	unsigned int retries; /*!< Max number of retries */
	int timeout_ms; /*!< Command timeout in milliseconds */
};

#define SDHC_NATIVE_RESPONSE_MASK   0xF
#define SDHC_SPI_RESPONSE_TYPE_MASK 0xF0

/**
 * @brief SD host controller data structure
 *
 * This command structure is used to send data transfer requests to an SD
 * host controller, which will be sent to SD devices.
 */
struct sdhc_data
{
	unsigned int block_addr; /*!< Block to start read from */
	unsigned int block_size; /*!< Block size */
	unsigned int blocks; /*!< Number of blocks */
	unsigned int bytes_xfered; /*!< populated with number of bytes sent by SDHC */
#if defined(CONFIG_SDHC_SCATTER_GATHER_TRANSFER) || defined(__DOXYGEN__)
	bool is_sg_data; /*!< Is scatter gather data using net_buf data structure */
#endif
	void *data; /*!< Data to transfer or receive */
	int timeout_ms; /*!< data timeout in milliseconds */
};

/**
 * @brief SD bus mode.
 *
 * Most controllers will use push/pull, including spi, but
 * SDHC controllers that implement SD host specification can support open
 * drain mode
 */
enum sdhc_bus_mode
{
	SDHC_BUSMODE_OPENDRAIN = 1,
	SDHC_BUSMODE_PUSHPULL = 2,
};

/**
 * @brief SD host controller power
 *
 * Many host controllers can control power to attached SD cards.
 * This enum allows applications to request the host controller power off
 * the SD card.
 */
enum sdhc_power
{
	SDHC_POWER_OFF = 1,
	SDHC_POWER_ON = 2,
};

/**
 * @brief SD host controller bus width
 *
 * Only relevant in SD mode, SPI does not support bus width. UHS cards will
 * use 4 bit data bus, all cards start in 1 bit mode
 */
enum sdhc_bus_width
{
	SDHC_BUS_WIDTH1BIT = 1U,
	SDHC_BUS_WIDTH4BIT = 4U,
	SDHC_BUS_WIDTH8BIT = 8U,
};

/**
 * @brief SD host controller timing mode
 *
 * Used by SD host controller to determine the timing of the cards attached
 * to the bus. Cards start with legacy timing, but UHS-II cards can go up to
 * SDR104.
 */
enum sdhc_timing_mode
{
	SDHC_TIMING_LEGACY = 1U,
	/*!< Legacy 3.3V Mode */
	SDHC_TIMING_HS = 2U,
	/*!< Legacy High speed mode (3.3V) */
	SDHC_TIMING_SDR12 = 3U,
	/*!< Identification mode & SDR12 */
	SDHC_TIMING_SDR25 = 4U,
	/*!< High speed mode & SDR25 */
	SDHC_TIMING_SDR50 = 5U,
	/*!< SDR49 mode*/
	SDHC_TIMING_SDR104 = 6U,
	/*!< SDR104 mode */
	SDHC_TIMING_DDR50 = 7U,
	/*!< DDR50 mode */
	SDHC_TIMING_DDR52 = 8U,
	/*!< DDR52 mode */
	SDHC_TIMING_HS200 = 9U,
	/*!< HS200 mode */
	SDHC_TIMING_HS400 = 10U,
	/*!< HS400 mode */
};

/**
 * @brief SD voltage
 *
 * UHS cards can run with 1.8V signalling for improved power consumption. Legacy
 * cards may support 3.0V signalling, and all cards start at 3.3V.
 * Only relevant for SD controllers, not SPI ones.
 */
enum sd_voltage
{
	SD_VOL_3_3_V = 1U,
	/*!< card operation voltage around 3.3v */
	SD_VOL_3_0_V = 2U,
	/*!< card operation voltage around 3.0v */
	SD_VOL_1_8_V = 3U,
	/*!< card operation voltage around 1.8v */
	SD_VOL_1_2_V = 4U,
	/*!< card operation voltage around 1.2v */
};

struct sdhc_spi_config
{
	const SPI_HandleTypeDef *hspi;
	// const struct gpio_dt_spec pwr_gpio;
	// const struct gpio_dt_spec cd_gpio;
	const uint32_t spi_max_freq;
	uint32_t power_delay_ms;
};

struct sdhc_spi_data
{
	enum sdhc_power power_mode;
	struct sdhc_spi_config *spi_cfg;
	struct sdhc_spi_config cfg_a;
	struct sdhc_spi_config cfg_b;
	uint8_t scratch[MAX_CMD_READ];
};

/** @brief SDHC device handle */
struct sdhc_spi_device
{
	struct sdhc_spi_config *config; /**< Device configuration */
	struct sdhc_spi_data *data; /**< Sample and gain data */
};

int sdhc_init_card(SPI_HandleTypeDef *hspi);
int sdhc_spi_card_busy(SPI_HandleTypeDef *hspi);
int sdhc_spi_wait_unbusy(SPI_HandleTypeDef *hspi, int timeout_ms, int interval_ms);
int sdhc_spi_response_get(struct sdhc_spi_device *dev, struct sdhc_command *cmd, int rx_len);
int sdhc_spi_send_cmd(struct sdhc_spi_device *dev, struct sdhc_command *cmd, int data_present);

int sdhc_spi_read_data(const struct sdhc_spi_device *dev, struct sdhc_data *data);
int sdhc_spi_write_data(const struct sdhc_spi_device *dev, struct sdhc_data *data);

#endif // __SDHC_H__