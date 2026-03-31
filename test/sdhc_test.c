
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"

#include "sdhc.h"
#include "log.h"
#include <stdio.h>
#include <string.h>

CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI1_Init(void);
static void MX_CRC_Init(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
	MX_RTC_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_SPI4_Init();
	MX_SPI1_Init();
	MX_CRC_Init();
	log_init(&huart2);

	int ret;
	static struct sdhc_spi_config test_config = {
		.hspi = &hspi2,
		.spi_max_freq = 20000000,
	};
	static struct sdhc_spi_data test_spi_data = { 0 };
	static struct sdhc_spi_device test_dev = {
		.config = &test_config,
		.data = &test_spi_data,
	};

	static uint8_t buf[512] = { 0 };
	struct sdhc_data test_data = { 0 };
	test_data.block_addr = 0xDE; /*!< Block to start read from */
	test_data.block_size = 512; /*!< Block size */
	test_data.blocks = 1; /*!< Number of blocks */
	test_data.bytes_xfered = 0; /*!< populated with number of bytes sent by SDHC */
	test_data.data = buf; /*!< Data to transfer or receive */
	test_data.timeout_ms = 500; /*!< data timeout in milliseconds */

	ret = sdhc_init(&test_dev);
    if (ret != 0)
    {
        LOG_ERR("SD card init fail with return val %d", ret);
        Error_Handler();
    }

	// ====================
	//  SINGLE-BLOCK TESTS
	// ====================

	// WRITE ZEROS ONTO THE BLOCK
	// Send Write single block command CMD24
	ret = sdhc_write_data(&test_dev, &test_data);
	if (ret != 0)
	{
		LOG_ERR("Test write 1 failed with return 0x%02x", ret);
		Error_Handler();
	}
	LOG_INF("Test write 1 success");

	// READ THE BLOCK, VERIFY ALL 0
	// Setup read command as CMD17 (READ_SINGLE_BLOCK), R1 response expected
	ret = sdhc_read_data(&test_dev, &test_data);
	if (ret != 0)
	{
		LOG_INF("Test read 1 failed with return value %d", ret);
		Error_Handler();
	}
	LOG_INF("Test read 1 success");

	// Print block
	for (size_t off = 0; off < test_data.block_size; off += 16)
	{
		LOG_DBG("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x",
		        buf[off], buf[off + 1], buf[off + 2], buf[off + 3], buf[off + 4], buf[off + 5],
		        buf[off + 6], buf[off + 7], buf[off + 8], buf[off + 9], buf[off + 10],
		        buf[off + 11], buf[off + 12], buf[off + 13], buf[off + 14], buf[off + 15]);
	}

	// WRITE DEADBEEF PATTERN TO THE BLOCK
	for (size_t i = 0; (i + 3) < test_data.block_size; i += 4)
	{
		buf[i] = 0xDE;
		buf[i + 1] = 0xAD;
		buf[i + 2] = 0xBE;
		buf[i + 3] = 0xEF;
	}
	ret = sdhc_write_data(&test_dev, &test_data);
	if (ret != 0)
	{
		LOG_ERR("Test write 2 failed with return 0x%02x", ret);
		Error_Handler();
	}
	LOG_INF("Test write 2 success");

	// READ IT AGAIN, CHECK DEADBEEF PATTERN NOW ON BLOCK
	memset(buf, 0, sizeof(buf));
	ret = sdhc_read_data(&test_dev, &test_data);
	if (ret != 0)
	{
		LOG_INF("Test read 2 failed with return 0x%02x", ret);
		Error_Handler();
	}
	LOG_INF("Test read 2 success");

	// Print Block
	for (size_t off = 0; off < test_data.block_size; off += 16)
	{
		LOG_DBG("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x",
		        buf[off], buf[off + 1], buf[off + 2], buf[off + 3], buf[off + 4], buf[off + 5],
		        buf[off + 6], buf[off + 7], buf[off + 8], buf[off + 9], buf[off + 10],
		        buf[off + 11], buf[off + 12], buf[off + 13], buf[off + 14], buf[off + 15]);
	}

	// ===================
	//  MULTI-BLOCK TESTS
	// ===================
	static uint8_t multi_write_buf[4 * 512];
	static uint8_t multi_read_buf[4 * 512];

	test_data.block_addr = 0x100;
	test_data.block_size = 512;
	test_data.blocks = 4;
	test_data.timeout_ms = 500;

	// Fill write buffer with incrementing pattern
	for (size_t i = 0; i < sizeof(multi_write_buf); i++)
	{
		multi_write_buf[i] = (uint8_t) (i & 0xFF);
	}
	test_data.data = multi_write_buf;

	// Send command to write multiple block (incrementing)
	ret = sdhc_write_data(&test_dev, &test_data);
	if (ret != 0)
	{
		LOG_ERR("Multi-block write (incremental) failed: %d", ret);
		Error_Handler();
	}
	LOG_INF("Write 4-block incrementing pattern success");

	memset(multi_read_buf, 0, sizeof(multi_read_buf));
	test_data.data = multi_read_buf;

	ret = sdhc_read_data(&test_dev, &test_data);
	if (ret != 0)
	{
		LOG_INF("Multi-block read (incremental) failed with return 0x%02x", ret);
		Error_Handler();
	}

	// Verify data
	for (size_t i = 0; i < sizeof(multi_read_buf); i++)
	{
		if (multi_read_buf[i] != multi_write_buf[i])
		{
			LOG_ERR("Incremental verify fail @ byte %u: wrote 0x%02x, read 0x%02x", (unsigned) i,
			        multi_write_buf[i], multi_read_buf[i]);
			Error_Handler();
			break;
		}
	}
	LOG_INF("Read 4-block incrementing pattern success");

	// Test: multi-block write/read 0xAA/0x55 alternating pattern
	for (size_t i = 0; i < sizeof(multi_write_buf); i++)
	{
		multi_write_buf[i] = (i % 2 == 0) ? 0xAA : 0x55;
	}
	test_data.data = multi_write_buf;

	// Send command to write multiple block
	ret = sdhc_write_data(&test_dev, &test_data);
	if (ret != 0)
	{
		LOG_ERR("Multi-block write (0xAA55) failed: %d", ret);
		Error_Handler();
	}
	LOG_INF("CMD25 write 0xAA/0x55 alternating pattern success");

	memset(multi_read_buf, 0, sizeof(multi_read_buf));
	test_data.data = multi_read_buf;

	ret = sdhc_read_data(&test_dev, &test_data);
	if (ret != 0)
	{
		LOG_INF("Multi-block read (0xAA55) failed with return 0x%02x", ret);
		Error_Handler();
	}

	for (size_t i = 0; i < sizeof(multi_read_buf); i++)
	{
		uint8_t expected = (i % 2 == 0) ? 0xAA : 0x55;
		if (multi_read_buf[i] != expected)
		{
			LOG_ERR("0xAA55 verify fail @ byte %u: expected 0x%02x, got 0x%02x", (unsigned) i,
			        expected, multi_read_buf[i]);
			Error_Handler();
		}
	}
	LOG_INF("Read 4-block 0xAA55 pattern success");

	// Test: multi-block write/read all zeros
	memset(multi_write_buf, 0x00, sizeof(multi_write_buf));
	test_data.data = multi_write_buf;
	test_data.bytes_xfered = 0;

	ret = sdhc_write_data(&test_dev, &test_data);
	if (ret != 0)
	{
		LOG_ERR("Multi-block write (zeros) failed: %d", ret);
		Error_Handler();
	}
	LOG_INF("CMD25 write all-zeros success");

	memset(multi_read_buf, 0xFF, sizeof(multi_read_buf));
	test_data.data = multi_read_buf;
	test_data.bytes_xfered = 0;

	ret = sdhc_read_data(&test_dev, &test_data);
	if (ret != 0)
	{
		LOG_INF("Multi-block read (zeros) failed with return 0x%02x", ret);
		Error_Handler();
	}

	for (size_t i = 0; i < sizeof(multi_read_buf); i++)
	{
		if (multi_read_buf[i] != 0x00)
		{
			LOG_ERR("Zeros verify fail @ byte %u: got 0x%02x", (unsigned) i, multi_read_buf[i]);
			Error_Handler();
		}
	}
	LOG_INF("Read 4-block zeros pattern success");

	LOG_INF("All multi-block tests PASSED");

	/* Infinite loop */
	while (1)
	{
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 50;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType
	        = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // ~ 380 kHz
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */
}

/**
 * @brief SPI4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI4_Init(void)
{

	/* USER CODE BEGIN SPI4_Init 0 */

	/* USER CODE END SPI4_Init 0 */

	/* USER CODE BEGIN SPI4_Init 1 */

	/* USER CODE END SPI4_Init 1 */
	/* SPI4 parameter configuration*/
	hspi4.Instance = SPI4;
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi4.Init.NSS = SPI_NSS_SOFT;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI4_Init 2 */

	/* USER CODE END SPI4_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, SPI4_CS_MAG_Pin | SPI1_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, SPI3_CS_Pin | SPI4_CS_IMU_Pin | SPI2_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : GPIO_LED_Pin */
	GPIO_InitStruct.Pin = GPIO_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIO_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI4_CS_MAG_Pin SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI4_CS_MAG_Pin | SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI3_CS_Pin SPI4_CS_IMU_Pin SPI2_CS_Pin */
	GPIO_InitStruct.Pin = SPI3_CS_Pin | SPI4_CS_IMU_Pin | SPI2_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
