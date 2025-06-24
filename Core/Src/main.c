/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <math.h>
#include <string.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// image size 96 x 128 = 12288
#define BUFFER_SIZE 12288

// input and output buffers
uint8_t rcvd_data[BUFFER_SIZE];
uint8_t trmt_data[BUFFER_SIZE];

// segmentation/thresholding constants
#define THRESHOLD 120
#define THRESHOLD1 120
#define THRESHOLD2 200

// gray level transformation constants
#define A 115
#define B 75

// histogram equalization constant and variables
#define NUM_GRAY_LEVEL 256 // 256 gray-scale brightness
uint32_t hist_cnt[NUM_GRAY_LEVEL] = {0};  // store counts
uint8_t mapped_levels[NUM_GRAY_LEVEL];


// segmentation/thresholding functions
void global_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold);
void band_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold1, uint8_t threshold2);
void semi_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold);

void global_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold);
void band_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold1, uint8_t threshold2);
void semi_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold);

// gray level quantization functions
void gray_level_quantization_c(uint8_t *x, uint32_t size, uint8_t shift_factor);
void gray_level_quantization_hybrid(uint8_t *x, uint32_t size, uint8_t shift_factor);

// gray level transformation functions
void gray_level_transformation1_c(uint8_t *x, uint32_t size);
void gray_level_transformation2_c(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);
void gray_level_transformation3_c(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);

void gray_level_transformation1_hybrid(uint8_t *x, uint32_t size);
void gray_level_transformation2_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);
void gray_level_transformation3_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);

// histogram equalization functions
void calculate_histogram_c(uint8_t *x, uint32_t *hist, uint32_t size);
void map_levels_c(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels);
void transform_image_c(uint8_t *x, uint8_t *mapping_table, uint32_t size);

void calculate_histogram_hybrid(uint8_t *x, uint32_t *hist, uint32_t size);
void map_levels_hybrid(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels);
void transform_image_hybrid(uint8_t *x, uint8_t *mapping_table, uint32_t size);


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_UART_Receive_IT(&huart2,rcvd_data,BUFFER_SIZE);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // for debugging
	// copy received image to the transmit buffer
	// if no image processing function is selected below,
	// the original image received will be sent back (loop back)
	memcpy(trmt_data, rcvd_data, BUFFER_SIZE);
	// clear the histogram array for each new image received
    memset(hist_cnt, 0, NUM_GRAY_LEVEL);

    // test your image processing functions here
    gray_level_quantization_c(trmt_data, BUFFER_SIZE, 4);

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // for debugging

	// transmit processed image
	HAL_UART_Transmit(huart,trmt_data,BUFFER_SIZE,HAL_MAX_DELAY);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // for debugging
}

void global_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold)
{
	for (int i = 0; i < size; i++) {
		if (x[i] >= threshold) {
			x[i] = 255;
		} else x[i] = 0;
	}
}

__attribute__ ((naked)) void global_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold)
{
	__asm volatile (
	// x -> r0, size -> r1, threshold -> r2
			"PUSH {r4, r5, r6, lr}\n\t" // save return address

			// loop over the array
			"MOV r3, #0\n\t" // loop counter r3
			"MOV r4, #255\n\t" // constant used
			"MOV r5, #0\n\t" // constant used"
			"loop_gth: CMP r3, r1\n\t" // terminate the loop when r3 >= r1
			"BGE exit_gth\n\t"
			"LDRB r6, [r0]\n\t" // load array element to r6
			"CMP r6, r2\n\t" // compare with threshold r2
			"BGE else_gth\n\t"
			"STRB r5, [r0], #1\n\t" // store r5 to its memory location, post increment r0 by 1
			"B endif_gth\n\t"
			"else_gth: STRB r4, [r0], #1\n\t" // store r4 to its memory location, post increment r0 by 1
			"endif_gth: ADD r3, r3, #1\n\t" // increment loop counter
			"B loop_gth\n\t"
			"exit_gth: POP {r4, r5, r6, pc}\n\t" // return from function

    );

}

void band_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold1, uint8_t threshold2)
{
	for (int i=0 ; i<size; i++) {
		int pixel = x[i];
		if (pixel >= threshold1 && pixel <= threshold2) {
			x[i] = 255;
		} else x[i] = 0;
	}
}

__attribute__ ((naked)) void band_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold1, uint8_t threshold2)
{
	__asm volatile (
			// x -> r0, size -> r1, threshold1 -> r2, threshold2 ->r3
			"PUSH {r5, r6, r7, lr}\n\t" // save return address

			// loop over x
			"MOV r4, #0\n\t" // loop counter r4
			"MOV r5, #255 \n\t" // set 255 upper bound
			"MOV r6, #0\n\t" // set 0 lower bound
			"loop_bth: CMP r4, r1\n\t" // compare with size (r1)
			"BGE exit_bth\n\t" // exit if counter (r4) > size (r1)

			// counter < size
			"LDRB r7, [r0]\n\t" // load number (byte) from x (r0)
			"CMP r7, r3\n\t" // check with upper threshold (r3)
			"BGT else_bth\n\t" // go to else block if val (r7) > thresh2 (r3)
			"CMP r7, r2\n\t" // check with lower threshold (r2)
			"BLT else_bth\n\t" // go to else if val(r7) < thresh1(r2)

			// thresh1 <= x[i] <= thresh2
			"STRB r5, [r0], #1\n\t" // x[i] = 255
			"B endif_bth\n\t"

			// thresh1 > x[i] or thresh2 < x[1]
			"else_bth: STRB r6, [r0], #1\n\t" // x[i] = 0

			"endif_bth: ADD r4, r4, #1\n\t" // increment counter
			"B loop_bth\n\t"

			// counter >= size
			"exit_bth: POP {r5, r6, r7, pc}\n\t" // return
	);
}

void semi_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold)
{
	for (int i=0; i<size; i++) {
		if (x[i] < threshold)
			x[i] = 0;
	}
}

__attribute__ ((naked)) void semi_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold)
{
	__asm volatile(
			// x-> r0, size -> r1, threshold -> r2
			"PUSH {r4, r5, lr}\n\t" //save return address

			// loop over x
			"MOV r3, #0\n\t" // initialize counter r3
			"MOV r4, #0\n\t" // set lower bound 0
			"loop_sth: CMP r3, r1\n\t" // compare with size (r1)
			"BGE exit_sth\n\t" // exit if counter (r3) > size (r1)

			// counter < size
			"LDRB r5, [r0]\n\t" // load x[i] from x (r0)
			"CMP r5, r2\n\t" // compare with threshold r2
			"BGE else_sth\n\t" // else keep same val and increment pointer

			// x[i] >= threshold
			"STRB r4, [r0], #1\n\t" // x[i] = 0
			"B endif_sth\n\t"

			// x[i] < threshold - do nothing
			"else_sth: STRB r5, [r0], #1\n\t" // x[i] = x[i]

			"endif_sth: ADD r3, r3, #1\n\t" // increment counter
			"B loop_sth\n\t" // continue loop

			// counter >= size
			"exit_sth: POP {r4, r5, pc}\n\t" // exit loop and return
	);
}

// shift factor is 1, 2, 3, 4 for 128, 64, 32, 16 gray levels
void gray_level_quantization_c(uint8_t *x, uint32_t size, uint8_t shift_factor)
{
	float shift = (float) pow(2, shift_factor);
	for(int i=0; i<size; i++) {
		float pixel = (float)x[i];
		pixel = pixel/shift;
		x[i] = (int)pixel;
	}
}

__attribute__ ((naked)) void gray_level_quantization_hybrid(uint8_t *x, uint32_t size, uint8_t shift_factor)
{
	__asm volatile(
			// x -> r0, size -> r1, shift_factor -> r2
			"PUSH {r4, lr}\n\t" // save return address

			// loop over x
			"MOV r3, #0\n\t" // loop counter r3
			"loop_glq: CMP r3, r1\n\t" // compare with size (r1)
			"BGE exit_glq\n\t" // exit loop

			// counter < size
			"LDRB r4, [r0]\n\t" // load x[i] into r4
			"MOV r4, r4, LSR r2\n\t" // x[i] >> factor
			"STRB r4, [r0], #1\n\t" // x[i] = r4
			"ADD r3, r3, #1\n\t" // increment counter
			"B loop_glq\n\t"

			// counter >= size
			"exit_glq: POP {r4, pc}\n\t" // return
	);
}

void gray_level_transformation1_c(uint8_t *x, uint32_t size)
{
	for (int i = 0; i<size; i++) {
		x[i] = 255 - x[i];
	}
}

__attribute__ ((naked)) void gray_level_transformation1_hybrid (uint8_t *x, uint32_t size)
{
	__asm volatile(
			// x -> r0, size -> r1
			"PUSH {r3, r4, lr}\n\t" // save address

			// loop over x
			"MOV r2, #0\n\t" // initialize loop counter r2
			"MOV r3, #255\n\t" // set constant
			"loop_glt1: CMP r2, r1\n\t" // compare size with counter
			"BGE exit_glt1\n\t" // exit loop

			// counter < size
			"LDRB r4, [r0]\n\t" // load x[i] into r4
			"SUB r4, r3, r4\n\t" // r4 = 255 - x[i]
			"STRB r4, [r0], #1\n\t" // x[i] = r4
			"ADD r2, r2, #1\n\t" // increment counter
			"B loop_glt1\n\t"

			// counter >= size
			"exit_glt1: POP {r3, r4, pc}\n\t" // return
	);
}


void gray_level_transformation2_c(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0)
{
	for(int i=0; i<size; i++) {
		if (x[i] <= b0) {
			x[i] = (int)((float)(a0 * x[i]) / (float)(b0));
		} else {
			x[i] = (int)((float)((255 - a0) * (x[i] - b0)) / (float)(255 - b0)) + a0;
		}
	}
}

__attribute__ ((naked)) void gray_level_transformation2_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0)
{
	// make sure you "MUL" first, then "UDIV", otherwise you will get zero
	__asm volatile(
			// x -> r0, size -> r1, a0 -> r2, b0 -> r3
			"PUSH {r5, r6, r7, r8, lr}\n\t" // save return address

			// loop over x
			"MOV r4, #0\n\t" // initialize loop counter r4
			"MOV r5, #255\n\t" // set constant
			"loop_glt2: CMP r4, r1\n\t" // compare counter with size
			"BGE exit_glt2\n\t" // exit loop

			// counter < size
			"LDRB r6, [r0]\n\t" // load x[i] (byte) into r6
			"CMP r6, r3\n\t" // compare x[i] with b0 (r3)
			"BGT else_glt2\n\t" // jump to else if x[i] > b0

			// x[i] <= b0
			"MUL r7, r2, r6\n\t" // r7 = x[i] * a0
			"UDIV r7, r7, r3\n\t" // r7 = r7 / b0
			"B endif_glt2\n\t" // jump to end of if block

			// x[i] > b0
			"else_glt2: SUB r7, r5, r2\n\t" // r7 = (255 - a0)
			"SUB r8, r5, r3\n\t" // r8 = (255 - b0)
			"SUB r6, r6, r3\n\t" // r6 = (x[i] - b0)
			"MUL r7, r7, r6\n\t" // r7  = r7 * r6 i.e. (255 - a0)*(x[i] - b0)
			"UDIV r7, r7, r8\n\t" // r7 = r7/r8 i.e. r7 / (255 - b0)
			"ADD r7, r7, r2\n\t" // r7 = r7 + a0

			"endif_glt2: STRB r7, [r0], #1\n\t" // x[i] = r7
			"ADD r4, r4, #1\n\t" // increment loop counter
			"B loop_glt2\n\t" // continue loop

			// counter >= size
			"exit_glt2: POP {r5, r6, r7, r8, pc}\n\t" // return

	);
}

void gray_level_transformation3_c(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0)
{
	for (int i=0; i<size; i++) {
		if (x[i] <= b0) {
			x[i] = (int)((float)(a0 * x[i]) / (float)(b0));
		} else if (x[i] <= (255 - b0)) {
			x[i] = (int)((float)((255 - 2*a0) * (x[i] - b0)) / (float)(255 - 2*b0)) + a0;
		} else {
			x[i] = (int)((float)(a0 * (x[i] - (255 - b0))) / (float)(b0)) + (255 - a0);
		}
	}
}

__attribute__ ((naked)) void gray_level_transformation3_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0)
{
	// make sure you "MUL" first, then "UDIV", otherwise you will get zero
	__asm volatile(
			// x -> r0, size -> r1, a0 -> r2, b0 -> r3
			"PUSH {r5, r6, r7, r8, r9, lr}\n\t" // save return address

			// loop over x
			"MOV r4, #0\n\t"
			"MOV r5, #255\n\t"
			"SUB r6, r5, r2\n\t" // 255 - a0
			"SUB r7, r5, r3\n\t" // 255 - b0
			"loop_glt3: CMP r4, r1\n\t" // compare counter with size
			"BGE exit_glt3\n\t" // exit loop

			// counter < size
			"LDRB r8, [r0]\n\t" // load x[i]
			"CMP r8, r3\n\t" // compare x[i] with b0
			"BLE if_glt3\n\t" // jump if x[i] <= b0
			"CMP r8, r7\n\t" // compare x[i] with (255 - b0)
			"BLE elif_glt3\n\t" // jump if x[i] <= (255 - b0)

			// x[i] > (255 - b0)
			"SUB r8, r8, r7\n\t" // r8  = x[i] - (255 - b0)
			"MUL r8, r8, r2\n\t" // r8 = r8 * a0
			"UDIV r8, r8, r3\n\t" // r8  = r8 / b0
			"ADD r8, r8, r6\n\t" // r8 = r8 + (255 - a0)
			"B endif_glt3\n\t" // jump to end of if block

			// x[i] <= b0
			"if_glt3: MUL r8, r8, r2\n\t" // r8  = x[i] * a0
			"UDIV r8, r8, r3\n\t" // r8  = r8 / b0
			"B endif_glt3\n\t" // jump to end of if block

			// x[i] <= (255 - b0)
			"elif_glt3: SUB r8, r8, r3\n\t" // r8 = x[i] - b0
			"SUB r9, r5, r2, LSL #1\n\t" // r9 =  255 - 2*a0 ( a0 << 1)
			"MUL r8, r8, r9\n\t" // r8 = r8 * r9
			"SUB r9, r5, r3, LSL #1\n\t" // r9 = 255 - 2*b0 (b0 << 1)
			"UDIV r8, r8, r9\n\t" // r8 = r8 / r9
			"ADD r8, r8, r2\n\t" // r8 = r8 + (255 - a0)

			"endif_glt3: STRB r8, [r0], #1\n\t" // x[i] = r8
			"ADD r4, r4, #1\n\t" // increment loop counter
			"B loop_glt3\n\t" // continue loop

			// counter >= size
			"exit_glt3: POP {r5, r6, r7, r8, r9, pc}\n\t" // return

	);
}

/*
"calculate_histogram" is a function that counts the frequency of the occurrence of each
grayscale level in an image.
*/
void calculate_histogram_c(uint8_t *x, uint32_t *hist, uint32_t size)
{
	for (int i=0; i<size; i++) {
		hist[x[i]]++;
	}
}

__attribute__ ((naked)) void calculate_histogram_hybrid(uint8_t *x, uint32_t *hist, uint32_t size)
{
	__asm volatile(
			// x -> r0, hist -> r1, size -> r2
			"PUSH {r4, r5, lr}\n\t"

			// loop over x
			"MOV r3, #0\n\t" //initialize counter
			"loop_chh: CMP r3, r2\n\t" // compare counter with size
			"BGE exit_chh\n\t"

			// counter < size
			"LDRB r4, [r0, r3]\n\t" // load 8 bit x[i] with r3 increment
			"LDR r5, [r1, r4, LSL #2]\n\t" // load 32 bit hist[x[i]] with 4 byte shift
			"ADD r5, r5, #1\n\t" // increment hist number
			"STR r5, [r1, r4, LSL #2]\n\t" // store 32 bit r5 with 4 byte increment
			"ADD r3, r3, #1\n\t" // increment counter
			"B loop_chh\n\t"

			// counter >= size
			"exit_chh: POP {r4, r5, pc}\n\t" // return
	);
}

/*
"map_levels" is a function that generates a mapping table (mapped_levels) to
equalize the histogram.
*/
void map_levels_c(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels)
{
	int sum = 0;
	for (int i=0; i<levels; i++) {
		sum += hist[i];
		mapping_table[i] = (int)((float)((levels - 1) * sum)/ (float)(size));
	}
}

__attribute__ ((naked)) void map_levels_hybrid(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels)
{
	// make sure you "MUL" first, then "UDIV", otherwise you will get zero
	// you need to accumulate the histogram counts, multiply by (levels-1), then divide by size
	__asm volatile(
			// hist -> r0, map_table -> r1, size -> r2, levels -> r3
			"PUSH {r5, r6, r7, lr}\n\t" // save return address

			// loop over hist
			"MOV r4, #0\n\t" // initialize counter i
			"MOV r5, #0\n\t" // intialize sum
			"loop_mlh: CMP r4, r3\n\t" // compare counter with levels
			"BGE exit_mlh\n\t"

			// counter < levels
			"LDR r6, [r0, r4, LSL #2]\n\t" // r6 = hist[i] with 4 byte increment (32bit)
			"ADD r5, r5, r6\n\t" // sum = sum + r6
			"SUB r7, r3, #1\n\t" // r7 = levels - 1
			"MUL r7, r7, r5\n\t" // r7 = r7 * sum
			"UDIV r7, r7, r2\n\t" // r7 = r7 / size
			"STRB r7, [r1, r4]\n\t" // store 8 bit r6 in map_table with 1 byte increment
			"ADD r4, r4, #1\n\t" // increment counter
			"B loop_mlh\n\t"

			// counter >= size
			"exit_mlh: POP {r5, r6, r7, pc}\n\t" // return

	);
}



/*
"transform_image" is a function that transforms the brightness levels of an image array
according to the mapping_table using histogram equalization
*/

void transform_image_c(uint8_t *x, uint8_t *mapping_table, uint32_t size)
{
	for(int i=0; i<size; i++) {
		x[i] = mapping_table[x[i]];
	}
}

__attribute__ ((naked)) void transform_image_hybrid(uint8_t *x, uint8_t *mapping_table, uint32_t size)
{
	__asm volatile(
			// x -> r0, map_table -> r1, size -> r2
			"PUSH {r4, r5, lr}\n\t"

			// loop over x
			"MOV r3, #0\n\t" // intialize counter
			"loop_tih: CMP r3, r2\n\t"
			"BGE exit_tih\n\t"

			// counter < size
			"LDRB r4, [r0]\n\t"
			"LDRB r5, [r1, r4]\n\t"
			"STRB r5, [r0], #1\n\t"
			"ADD r3, r3, #1\n\t"
			"B loop_tih\n\t"

			// counter >= size
			"exit_tih: POP {r4, r5, pc}"
	);
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
