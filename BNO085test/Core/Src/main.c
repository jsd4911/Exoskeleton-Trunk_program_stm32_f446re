/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <math.h>          // 新增：用來算 atan2 和 asin
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO08x_ADDR (0x4B << 1) // STM32 I2C 地址需左移 1 位
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
uint8_t isStreaming = 0;
uint32_t lastCmdTick = 0;

// BNO085 啟動指令 (要求 20Hz 回報 Rotation Vector)
uint8_t setFeatureCmd[21] = {
    21, 0, 2, 0,
    0xFD, 0x05, 0, 0, 0,
    0x50, 0xC3, 0x00, 0x00,
    0, 0, 0, 0, 0, 0, 0, 0
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    // 透過 USART2 把字元一個一個推給電腦
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100);
    return len;
}
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);
    printf("\r\n=== BNO085 姿態解算啟動 ===\r\n");

    // 硬體重置 BNO085
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(500);

    if (HAL_I2C_IsDeviceReady(&hi2c1, BNO08x_ADDR, 5, 100) == HAL_OK) {
        printf("-> BNO085 (0x4B) 連線成功！準備接收資料...\r\n");
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // --- 任務 A：自動喚醒機制 ---
	        // 如果還沒收到資料，每 2 秒瘋狂敲門一次
	        if (isStreaming == 0 && (HAL_GetTick() - lastCmdTick > 2000)) {
	            printf("-> 發送啟動指令給 BNO085...\r\n");
	            HAL_I2C_Master_Transmit(&hi2c1, BNO08x_ADDR, setFeatureCmd, 21, 100);
	            lastCmdTick = HAL_GetTick();
	        }

	        // --- 任務 B：資料接收與解算 ---
	        // 當 INT 腳位 (PB0) 被拉低時，代表感測器有話要說
	        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
	        {
	            uint8_t header[4];
	            if (HAL_I2C_Master_Receive(&hi2c1, BNO08x_ADDR, header, 4, 100) == HAL_OK)
	            {
	                uint16_t length = ((header[1] << 8) | header[0]) & 0x7FFF;
	                uint8_t channel = header[2];

	                if (length > 0 && length <= 512)
	                {
	                    uint8_t shtpData[512];
	                    if (HAL_I2C_Master_Receive(&hi2c1, BNO08x_ADDR, shtpData, length, 100) == HAL_OK)
	                    {
	                        // 判斷是否為 Rotation Vector (通道 3, 且 Feature ID 為 0x05)
	                        if (channel == 3 && shtpData[9] == 0x05)
	                        {
	                            isStreaming = 1; // 標記為成功，停止奪命連環呼叫

	                            // 提取四元數原始資料
	                            int16_t i_raw = (shtpData[14] << 8) | shtpData[13];
	                            int16_t j_raw = (shtpData[16] << 8) | shtpData[15];
	                            int16_t k_raw = (shtpData[18] << 8) | shtpData[17];
	                            int16_t r_raw = (shtpData[20] << 8) | shtpData[19];

	                            float qI = (float)i_raw / 16384.0f;
	                            float qJ = (float)j_raw / 16384.0f;
	                            float qK = (float)k_raw / 16384.0f;
	                            float qR = (float)r_raw / 16384.0f;

	                            // 四元數轉歐拉角 (Roll, Pitch, Yaw)
	                            float sqI = qI * qI, sqJ = qJ * qJ, sqK = qK * qK;

	                            float roll_rad  = atan2(2.0f * (qR * qI + qJ * qK), 1.0f - 2.0f * (sqI + sqJ));
	                            float pitch_rad = asin(2.0f * (qR * qJ - qK * qI));
	                            float yaw_rad   = atan2(2.0f * (qR * qK + qI * qJ), 1.0f - 2.0f * (sqJ + sqK));

	                            // 弧度轉角度
	                            roll  = roll_rad  * 180.0f / 3.14159265f;
	                            pitch = pitch_rad * 180.0f / 3.14159265f;
	                            yaw   = yaw_rad   * 180.0f / 3.14159265f;

	                            // 🌟 見證奇蹟的時刻：印出角度！
	                            //printf("Roll: %6.1f | Pitch: %6.1f | Yaw: %6.1f\r\n", roll, pitch, yaw);

	                            // 🌟 見證奇蹟的時刻：印出角度！
	                            printf("%.1f,%.1f,%.1f\r\n", roll, pitch, yaw);
	                            //printf("%.2f,%.2f,%.2f\r\n", roll, pitch, yaw);
	                            // 2. 👉 新增：打包成字串發送給「馬達板」 (透過 USART3)
	                            //char sync_buf[30];
	                            // 將角度塞進字串，記得最後一定要有 \n (換行符號)，馬達板才知道這句話講完了
	                            //sprintf(sync_buf, "P:%.1f,R:%.1f,Y:%.1f\n", pitch, roll, yaw);

	                            // 透過 USART3 傳送出去，設定 10ms 的超時保護
	                            //HAL_UART_Transmit(&huart3, (uint8_t*)sync_buf, strlen(sync_buf), 10);
	                            // 👆 ==============================================================
	                        }
	                    }
	                }
	            }
	        }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
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
