/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "ST7735.h"
#include "fonts.h"
#include "GFX_FUNCTIONS.h"
//#include "testimg.h"
#include "ADXL.h"
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t data_rec[6];
uint16_t acc[3];
int16_t x, y, z;
float xg, yg, zg;
char x_str[50];
char y_str[50];
char z_str[50];
char xg_str[50];
char yg_str[50];
char zg_str[50];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void adxl_write (uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0] = address | 0x40;  // multibyte write
	data[1] = value;
	HAL_GPIO_WritePin (ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);  // pull the cs pin low
	HAL_SPI_Transmit (&hspi3, data, 2, 100);  // write data to register
	HAL_GPIO_WritePin (ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);  // pull the cs pin high
}

void adxl_read (uint8_t address)
{
	address |= 0x80;  // read operation
	address |= 0x40;  // multibyte read
	uint8_t rec;
	HAL_GPIO_WritePin (ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);  // pull the pin low
	HAL_SPI_Transmit  (&hspi3, &address, 1, 100);  // send address
	HAL_SPI_Receive   (&hspi3, data_rec, 6, 100);  // receive 6 bytes data
	HAL_GPIO_WritePin (ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);  // pull the pin high
}

void adxl_readId (uint8_t address)
{
	address |= 0x80;  // read operation
	HAL_GPIO_WritePin (ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);  // pull the pin low
	HAL_SPI_Transmit  (&hspi3, &address, 1, 100);  // send address
	HAL_SPI_Receive   (&hspi3, data_rec, 1, 100);  // receive 1 byte data
	HAL_GPIO_WritePin (ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);  // pull the pin high
}

void adxl_init (void)
{
	adxl_readId (0x00);        // Read device ID
	adxl_write  (0x2C, 0x0F);  // Set output data rate to 3200 Hz
	adxl_write  (0x2E, 0x00);  // Set INT_ENABLE register to 0x00
	adxl_write  (0x31, (0x04 | 0x0B));  // data_format range= +- 2g
	adxl_write  (0x38, 0x00);  // Bypass FIFO
	adxl_write  (0x2D, 0x00);  // reset all bits
	adxl_write  (0x2D, 0x08);  // power_cntl measure and wake up 8hz
}

/*
void ST7735init() {
    ST7735_Init();

    const char ready[] = "Ready!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)ready, sizeof(ready)-1, HAL_MAX_DELAY);
}
*/

/*
void loop() {
    // Check border
    ST7735_FillScreen(ST7735_BLACK);

    for(int x = 0; x < ST7735_WIDTH; x++) {
        ST7735_DrawPixel(x, 0, ST7735_RED);
        ST7735_DrawPixel(x, ST7735_HEIGHT-1, ST7735_RED);
    }

    for(int y = 0; y < ST7735_HEIGHT; y++) {
        ST7735_DrawPixel(0, y, ST7735_RED);
        ST7735_DrawPixel(ST7735_WIDTH-1, y, ST7735_RED);
    }

    HAL_Delay(3000);

    // Check fonts
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_WriteString(0, 0, "Font_7x10, red on black, lorem ipsum dolor sit amet", Font_7x10, ST7735_RED, ST7735_BLACK);
    ST7735_WriteString(0, 3*10, "Font_11x18, green, lorem ipsum", Font_11x18, ST7735_GREEN, ST7735_BLACK);
    ST7735_WriteString(0, 3*10+3*18, "Font_16x26", Font_16x26, ST7735_BLUE, ST7735_BLACK);
    HAL_Delay(2000);

    // Check colors
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_WriteString(0, 0, "BLACK", Font_11x18, ST7735_WHITE, ST7735_BLACK);
    HAL_Delay(500);

    ST7735_FillScreen(ST7735_BLUE);
    ST7735_WriteString(0, 0, "BLUE", Font_11x18, ST7735_BLACK, ST7735_BLUE);
    HAL_Delay(500);

    ST7735_FillScreen(ST7735_RED);
    ST7735_WriteString(0, 0, "RED", Font_11x18, ST7735_BLACK, ST7735_RED);
    HAL_Delay(500);

    ST7735_FillScreen(ST7735_GREEN);
    ST7735_WriteString(0, 0, "GREEN", Font_11x18, ST7735_BLACK, ST7735_GREEN);
    HAL_Delay(500);

    ST7735_FillScreen(ST7735_CYAN);
    ST7735_WriteString(0, 0, "CYAN", Font_11x18, ST7735_BLACK, ST7735_CYAN);
    HAL_Delay(500);

    ST7735_FillScreen(ST7735_MAGENTA);
    ST7735_WriteString(0, 0, "MAGENTA", Font_11x18, ST7735_BLACK, ST7735_MAGENTA);
    HAL_Delay(500);

    ST7735_FillScreen(ST7735_YELLOW);
    ST7735_WriteString(0, 0, "YELLOW", Font_11x18, ST7735_BLACK, ST7735_YELLOW);
    HAL_Delay(500);

    ST7735_FillScreen(ST7735_WHITE);
    ST7735_WriteString(0, 0, "WHITE", Font_11x18, ST7735_BLACK, ST7735_WHITE);
    HAL_Delay(500);

#ifdef ST7735_IS_128X128
    // Display test image 128x128
    ST7735_DrawImage(0, 0, ST7735_WIDTH, ST7735_HEIGHT, (uint16_t*)test_img_128x128);

/*
    // Display test image 128x128 pixel by pixel
    for(int x = 0; x < ST7735_WIDTH; x++) {
        for(int y = 0; y < ST7735_HEIGHT; y++) {
            uint16_t color565 = test_img_128x128[y][x];
            // fix endiness
            color565 = ((color565 & 0xFF00) >> 8) | ((color565 & 0xFF) << 8);
            ST7735_DrawPixel(x, y, color565);
        }
    }
*/
/*
    HAL_Delay(10000);
#endif // ST7735_IS_128X128

}

*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  adxl_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  ST7735_Init(0);
  fillScreen(BLACK);
  ST7735_SetRotation(0);

  /*
  ADXL_InitTypeDef ADXL_InitStruct;
  ADXL_InitStruct.SPIMode = SPIMODE_4WIRE;
//  ADXL_InitStruct.IntMode = ;
  ADXL_InitStruct.LPMode = LPMODE_NORMAL;
//  ADXL_InitStruct.Rate = ;
  ADXL_InitStruct.Range = RANGE_2G;
  ADXL_InitStruct.Resolution = RESOLUTION_10BIT;
  ADXL_InitStruct.Justify = JUSTIFY_MSB;
  ADXL_InitStruct.AutoSleep = AUTOSLEEPOFF;
  ADXL_InitStruct.LinkMode = LINKMODEOFF;

  ADXL_Init(&ADXL_InitStruct);
*/

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  ADXL_Measure(ON);

	  // READ DATA
//	  ADXL_getAccel(data_rec, OUTPUT_FLOAT);
//	  ADXL_getAccel(acc, OUTPUT_SIGNED);

 adxl_read(0x32);

	  xg = ((data_rec[1]<<8)|data_rec[0]); // acc[0];
	  yg = ((data_rec[3]<<8)|data_rec[2]); // acc[1];
	  zg = ((data_rec[5]<<8)|data_rec[4]); // acc[2];


	  // Convert into 'g'

	  x = xg * 0.0078;
	  y = yg * 0.0078;
	  z = zg * 0.0078;


	  // CONVERT TO STRING
/*
	  sprintf(x_str,"%f", x);
	  sprintf(y_str,"%f", y);
	  sprintf(z_str,"%f", z);
*/
	  sprintf(xg_str,"%.2f", xg);
	  sprintf(yg_str,"%.2f", yg);
	  sprintf(zg_str,"%.2f", zg);

	  sprintf(x_str,"%d", data_rec[0]);
	  sprintf(y_str,"%d", data_rec[1]);
	  sprintf(z_str,"%d", data_rec[2]);

	  // DISPLAY RESULT

	  ST7735_WriteString(2, 0, "x :", Font_7x10, BLACK, BLACK);
	  ST7735_WriteString(2, 0, "x :", Font_7x10, RED, BLACK);

	  ST7735_WriteString(12, 10, x_str, Font_7x10, BLACK, BLACK);
	  ST7735_WriteString(12, 10, x_str, Font_7x10, GREEN, BLACK);

	  ST7735_WriteString(12, 20, xg_str, Font_7x10, BLACK, BLACK);
	  ST7735_WriteString(12, 20, xg_str, Font_7x10, GREEN, BLACK);


	  ST7735_WriteString(2, 40, "y :", Font_7x10, BLACK, BLACK);
	  ST7735_WriteString(2, 40, "y :", Font_7x10, RED, BLACK);

	  ST7735_WriteString(12, 50, y_str, Font_7x10, BLACK, BLACK);
	  ST7735_WriteString(12, 50, y_str, Font_7x10, GREEN, BLACK);

	  ST7735_WriteString(12, 60, yg_str, Font_7x10, BLACK, BLACK);
	  ST7735_WriteString(12, 60, yg_str, Font_7x10, GREEN, BLACK);


	  ST7735_WriteString(2, 80, "z :", Font_7x10, BLACK, BLACK);
	  ST7735_WriteString(2, 80, "z :", Font_7x10, RED, BLACK);

	  ST7735_WriteString(12, 90, z_str, Font_7x10, BLACK, BLACK);
	  ST7735_WriteString(12, 90, z_str, Font_7x10, GREEN, BLACK);

	  ST7735_WriteString(12, 100, zg_str, Font_7x10, BLACK, BLACK);
	  ST7735_WriteString(12, 100, zg_str, Font_7x10, GREEN, BLACK);

/*
	  ST7735_FillScreen(ST7735_BLACK);
	  ST7735_WriteString(2, 0, "x :", Font_7x10, ST7735_RED, ST7735_BLACK);
	  ST7735_WriteString(12, 10, x_str, Font_7x10, ST7735_GREEN, ST7735_BLACK);
	  ST7735_WriteString(12, 20, xg_str, Font_7x10, ST7735_GREEN, ST7735_BLACK);

	  ST7735_WriteString(2, 40, "y :", Font_7x10, ST7735_RED, ST7735_BLACK);
	  ST7735_WriteString(12, 50, y_str, Font_7x10, ST7735_GREEN, ST7735_BLACK);
	  ST7735_WriteString(12, 60, yg_str, Font_7x10, ST7735_GREEN, ST7735_BLACK);

	  ST7735_WriteString(2, 80, "z :", Font_7x10, ST7735_RED, ST7735_BLACK);
	  ST7735_WriteString(12, 90, z_str, Font_7x10, ST7735_GREEN, ST7735_BLACK);
	  ST7735_WriteString(12, 100, zg_str, Font_7x10, ST7735_GREEN, ST7735_BLACK);
*/

/*
	  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0) {
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  SPI_Transmit_Test(0x32);
		  //HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);
		  //HAL_Delay(100);
		  //HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);
	}
*/

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_RS_DC_Pin|GPIO_CS_Pin|GPIO_Rst_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_RS_DC_Pin GPIO_CS_Pin GPIO_Rst_Pin */
  GPIO_InitStruct.Pin = GPIO_RS_DC_Pin|GPIO_CS_Pin|GPIO_Rst_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ADXL_CS_Pin */
  GPIO_InitStruct.Pin = ADXL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADXL_CS_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
