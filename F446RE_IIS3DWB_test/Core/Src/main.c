/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>

#define ARM_MATH_CM4
#include "arm_math.h"
#include "iis3dwb_reg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define    BOOT_TIME        10 //ms

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static int16_t data_raw_acceleration[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
char dev_id [8];

uint16_t rxBuf[16384];
uint16_t txBuf[16384];

char rxBufstr[16384];
char stringBuff[100];

float fft_in_buf[2048];
float fft_out_buf[2048];

arm_rfft_fast_instance_f32 fft_handler;

float real_fsample = 46875;
uint8_t callback_state = 0;
uint8_t outarray[14];
uint8_t uartfree = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void tx_com_ziyad( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);
void DoFFT();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	char uart_buf[50];
	int uart_buf_len;
	uint16_t timer_val;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_SPI2_Init();
	MX_TIM13_Init();
	/* USER CODE BEGIN 2 */

	HAL_UART_Transmit(&huart2, "\nInit ... \r\n", 10, 100);

	// Say something
	uart_buf_len = sprintf(uart_buf, "Timer test\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	// Start timer
	HAL_TIM_Base_Start(&htim13);

	stmdev_ctx_t dev_ctx;

	/* Initialize mems driver interface */
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hspi2; //&SENSOR_BUS;

	/* Init test platform */
	platform_init();

	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	iis3dwb_device_id_get(&dev_ctx, &whoamI);

	if (whoamI != IIS3DWB_ID)
		while (1);

	sprintf(dev_id, "%u", whoamI);

	HAL_UART_Transmit(&huart2, dev_id, 10, 100);

	HAL_Delay(50);

	/* Restore default configuration */
	iis3dwb_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		iis3dwb_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Enable Block Data Update */
	iis3dwb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_26k7Hz);

	/* Set full scale */
	iis3dwb_xl_full_scale_set(&dev_ctx, IIS3DWB_2g);

	/* Configure filtering chain(No aux interface)
	 * Accelerometer low pass filter path
	 */
	iis3dwb_xl_filt_path_on_out_set(&dev_ctx, IIS3DWB_LP_ODR_DIV_100);

	HAL_Delay(5000);

	arm_rfft_fast_init_f32(&fft_handler, 2048);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1)
	{
		// Get current time (microseconds)
		timer_val = __HAL_TIM_GET_COUNTER(&htim13);

		uint8_t reg;

		/* Read output only if new xl value is available */
		iis3dwb_xl_flag_data_ready_get(&dev_ctx, &reg);

		if (reg) {
			/* Read acceleration field data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));

			iis3dwb_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

			acceleration_mg[0] =
					iis3dwb_from_fs2g_to_mg(data_raw_acceleration[0]);

			acceleration_mg[1] =
					iis3dwb_from_fs2g_to_mg(data_raw_acceleration[1]);

			acceleration_mg[2] =
					iis3dwb_from_fs2g_to_mg(data_raw_acceleration[2]);

			/*
			// Fill the buffer with "x axis" acceleration data
			for (int i = 0; i < 8192; ++i) {
				// Read acceleration field data
				memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));

				iis3dwb_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

				acceleration_mg[0] = iis3dwb_from_fs2g_to_mg(data_raw_acceleration[0]);
				rxBuf[i] = (int)acceleration_mg[0];
				callback_state = 1;
			}


			for (int i = 0; i < rxBuf[i]; ++i) {
				sprintf(&rxBufstr, "%d", rxBuf[i]);
				sprintf(&stringBuff, "%s %d", "\nAcceleration : ", rxBuf[i] );
				// HAL_UART_Transmit(&huart2, stringBuff, sizeof(rxBufstr), 100);
			}

			//do audio loopback and push mono-sum to fft_in_buf
			int fft_in_ptr = 0;
			if (callback_state == 1) {
				for (int i=0; i<8192; i=i+4) {
					fft_in_buf[fft_in_ptr] = (float) ((int) (rxBuf[i]<<16)|rxBuf[i+1]);
					fft_in_buf[fft_in_ptr] += (float) ((int) (rxBuf[i+2]<<16)|rxBuf[i+3]);
					txBuf[i] = rxBuf[i];
					txBuf[i+1] = rxBuf[i+1];
					txBuf[i+2] = rxBuf[i+2];
					txBuf[i+3] = rxBuf[i+3];
					fft_in_ptr++;
				}
				DoFFT();
			}

			if (callback_state == 2) {
				for (int i=8192; i<16384; i=i+4) {
					fft_in_buf[fft_in_ptr] = (float) ((int) (rxBuf[i]<<16)|rxBuf[i+1]);
					fft_in_buf[fft_in_ptr] += (float) ((int) (rxBuf[i+2]<<16)|rxBuf[i+3]);
					txBuf[i] = rxBuf[i];
					txBuf[i+1] = rxBuf[i+1];
					txBuf[i+2] = rxBuf[i+2];
					txBuf[i+3] = rxBuf[i+3];
					fft_in_ptr++;
				}
				DoFFT();
			}
*/
		}
/*
		// Display acceleration results
		sprintf((char *)tx_buffer,
				"\nAcceleration [mg] : %6.2f    %6.2f    %6.2f\r\n",
				acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);

		HAL_UART_AbortTransmit(&huart2);

		tx_com_ziyad(tx_buffer, strlen((char const *)tx_buffer));
*/
		/*
		// Display FFT results frame | it takes ~10 000 microseconds
		sprintf(stringBuff, "%s", "\n31-5Hz\t63 Hz\t125 Hz\t250 Hz\t500 Hz\t1 kHz\t2.2 kHz\t4.5 kHz\t9 kHz\t15 kHz\t");
		sprintf((char *)tx_buffer,
				"\nFFT : %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\r\n",
				outarray[0], //frame start
				outarray[1], //31-5Hz
				outarray[2], //63 Hz
				outarray[3], //125 Hz
				outarray[4], //250 Hz
				outarray[5], //500 Hz
				outarray[6], //1 kHz
				outarray[7], //2.2 kHz
				outarray[8], //4.5 kHz
				outarray[9], //9 kHz
				outarray[10] //15 kHz
		);
		tx_com_ziyad(&stringBuff, strlen(stringBuff));
		tx_com_ziyad(tx_buffer, strlen((char const *)tx_buffer));
		*/
/*
		iis3dwb_temp_flag_data_ready_get(&dev_ctx, &reg);

		if (reg) {
			// Read temperature data
			memset(&data_raw_temperature, 0x00, sizeof(int16_t));

			iis3dwb_temperature_raw_get(&dev_ctx, &data_raw_temperature);

			temperature_degC = iis3dwb_from_lsb_to_celsius(data_raw_temperature);

			sprintf((char *)tx_buffer,
					"\nTemperature [degC] : %6.2f\r\n", temperature_degC);

			tx_com_ziyad(tx_buffer, strlen((char const *)tx_buffer));

		}
*/
		// Get elapsed time
		timer_val = __HAL_TIM_GET_COUNTER(&htim13) - timer_val;

		// Print elapsed time
		uart_buf_len = sprintf(uart_buf, "%u microseconds \r\n", timer_val);
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

		// HAL_Delay(1000);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
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
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
 * @brief TIM13 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM13_Init(void)
{

	/* USER CODE BEGIN TIM13_Init 0 */

	/* USER CODE END TIM13_Init 0 */

	/* USER CODE BEGIN TIM13_Init 1 */

	/* USER CODE END TIM13_Init 1 */
	htim13.Instance = TIM13;
	htim13.Init.Prescaler = 84 - 1;
	htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim13.Init.Period = 65536 - 1;
	htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM13_Init 2 */

	/* USER CODE END TIM13_Init 2 */

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
	huart2.Init.Parity = UART_PARITY_EVEN;
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
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : IIS3DWB_CS_Pin */
	GPIO_InitStruct.Pin = IIS3DWB_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(IIS3DWB_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len)
{
	if (handle == &hspi2) {
		HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(handle, &reg, 1, 1000);
		HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
		HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_SET);
	}
	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len)
{
	if (handle == &hspi2) {
		/* Read command */
		reg |= 0x80;
		HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(handle, &reg, 1, 1000);
		HAL_SPI_Receive(handle, bufp, len, 1000);
		HAL_GPIO_WritePin(IIS3DWB_CS_GPIO_Port, IIS3DWB_CS_Pin, GPIO_PIN_SET);
	}
	return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#ifdef STEVAL_MKI109V3
	CDC_Transmit_FS(tx_buffer, len);
#elif defined(SPC584B_DIS)
	sd_lld_write(&SD2, tx_buffer, len);
#endif
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com_ziyad(uint8_t *tx_buffer, uint16_t len)
{
	if (HAL_UART_Transmit(&huart2, tx_buffer, len, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
	HAL_Delay(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
	TIM3->CCR1 = PWM_3V3;
	TIM3->CCR2 = PWM_3V3;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_Delay(1000);
#endif
}

float complexABS(float real, float compl) {
	return sqrtf(real*real+compl*compl);
}

void DoFFT() {
	//Do FFT
	arm_rfft_fast_f32(&fft_handler, &fft_in_buf,&fft_out_buf,0);

	int freqs[1024];
	int freqpoint = 0;
	int offset = 150; //variable noisefloor offset

	//calculate abs values and linear-to-dB
	for (int i=0; i<2048; i=i+2) {
		freqs[freqpoint] = (int)(20*log10f(complexABS(fft_out_buf[i], fft_out_buf[i+1])))-offset;
		if (freqs[freqpoint]<0) freqs[freqpoint]=0;
		freqpoint++;
	}


	//push out data to Uart
	outarray[0] = 0xff; //frame start
	outarray[1] = (uint8_t)freqs[1]; //31-5Hz
	outarray[2] = (uint8_t)freqs[3]; //63 Hz
	outarray[3] = (uint8_t)freqs[5]; //125 Hz
	outarray[4] = (uint8_t)freqs[11]; //250 Hz
	outarray[5] = (uint8_t)freqs[22]; //500 Hz
	outarray[6] = (uint8_t)freqs[44]; //1 kHz
	outarray[7] = (uint8_t)freqs[96]; //2.2 kHz
	outarray[8] = (uint8_t)freqs[197]; //4.5 kHz
	outarray[9] = (uint8_t)freqs[393]; //9 kHz
	outarray[10] = (uint8_t)freqs[655]; //15 lHz


	if (uartfree==1) HAL_UART_Transmit_DMA(&huart2, &outarray[0], 11);
	uartfree = 0;
	callback_state=0;

}

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

#ifdef  USE_FULL_ASSERT
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

