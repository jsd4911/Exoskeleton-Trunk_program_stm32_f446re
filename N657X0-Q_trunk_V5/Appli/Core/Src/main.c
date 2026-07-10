/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Complete BNO085 Attitude Engine for STM32N657X0 (I2C4)
  * : Uses robust 32-Byte Single-Transaction SHTP Receiver.
  * : Added Cold Boot Baudrate Sync and USB-VCP Enumeration Delay.
  * : Strengthened with Handoff Synchronization & Clear Interrupt on Boot.
  * : Fixed Cold Boot VTOR deadlock and reordered USART1 initialization.
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
#include <math.h>          // 用於高精度 atan2f 與 asinf 計算

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO08x_ADDR (0x4B << 1) // STM32 I2C 寫入位址 (0x96)

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
uint8_t isStreaming = 0;
uint32_t lastCmdTick = 0;
volatile uint8_t bno_data_ready = 0; // EXTI11 中斷旗標

// BNO085 訂閱指令 (要求以 20Hz 速率回報 Rotation Vector 姿態)
// 封包長度 21 位元組，通道 2 (Control)
uint8_t setFeatureCmd[21] = {
    21, 0, 2, 0,
    0xFD, 0x05, 0, 0, 0,
    0x50, 0xC3, 0x00, 0x00, // 50,000 微秒 = 50ms = 20Hz 回報率
    0, 0, 0, 0, 0, 0, 0, 0
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_I2C4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void I2C4_Scan(void);
void BNO085_ReadAndParse(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// printf 重導向至 USART1 VCP 終端機
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

// I2C4 總線掃描診斷器
void I2C4_Scan(void) {
    printf("[I2C4] 開始掃描總線裝置...\r\n");
    uint8_t devices_found = 0;

    for (uint16_t i = 1; i < 128; i++) {
        if (HAL_I2C_IsDeviceReady(&hi2c4, (i << 1), 3, 10) == HAL_OK) {
            printf("  -> 發現裝置！ 7位元位址: 0x%02X (寫入位址: 0x%02X)", i, (i << 1));
            if (i == 0x4B) {
                printf(" [BNO085 活躍位址 (AD0拉高)]");
            } else if (i == 0x4A) {
                printf(" [BNO085 預設位址 (GND)]");
            }
            printf("\r\n");
            devices_found++;
        }
        HAL_Delay(5);
    }

    if (devices_found == 0) {
        printf("[I2C4] 警告: 未在總線上發現任何裝置。請檢查實體接線！\r\n");
    } else {
        printf("[I2C4] 掃描完畢，共發現 %d 個活躍裝置。\r\n", devices_found);
    }
    printf("==============================================\r\n");
}

// 🚀 高效、防死鎖的 32-Byte 單次 I2C 讀取與解算函式
void BNO085_ReadAndParse(void) {
    uint8_t rx_buf[32];
    memset(rx_buf, 0, sizeof(rx_buf));

    // 單次讀取 32 位元組，完美包覆 23 位元組的 Rotation Vector 封包
    if (HAL_I2C_Master_Receive(&hi2c4, BNO08x_ADDR, rx_buf, 32, 50) == HAL_OK) {
        uint16_t length = ((rx_buf[1] << 8) | rx_buf[0]) & 0x7FFF;
        uint8_t channel = rx_buf[2];

        // 檢查是否為有效的 Sensor Reports (通道 3) 且 Report ID 為 0x05 (Rotation Vector)
        if (channel == 3 && rx_buf[9] == 0x05) {
            isStreaming = 1; // 標記已成功建立串流，防止喚醒機制重複發送

            // 1. 解析 16 位元原始四元數 (Little-Endian 拼接)
            int16_t i_raw = (rx_buf[14] << 8) | rx_buf[13];
            int16_t j_raw = (rx_buf[16] << 8) | rx_buf[15];
            int16_t k_raw = (rx_buf[18] << 8) | rx_buf[17];
            int16_t r_raw = (rx_buf[20] << 8) | rx_buf[19];

            // 2. 數值縮放 (14-bit Q-point = 2^14 = 16384.0f)
            float qI = (float)i_raw / 16384.0f;
            float qJ = (float)j_raw / 16384.0f;
            float qK = (float)k_raw / 16384.0f;
            float qR = (float)r_raw / 16384.0f;

            // 3. 計算三角函數中間項
            float sqI = qI * qI;
            float sqJ = qJ * qJ;
            float sqK = qK * qK;

            // 4. 高精度四元數轉換為歐拉角 (Roll, Pitch, Yaw)
            roll  = atan2f(2.0f * (qR * qI + qJ * qK), 1.0f - 2.0f * (sqI + sqJ)) * (180.0f / (float)M_PI);
            pitch = asinf(2.0f * (qR * qJ - qK * qI)) * (180.0f / (float)M_PI);
            yaw   = atan2f(2.0f * (qR * qK + qI * qJ), 1.0f - 2.0f * (sqJ + sqK)) * (180.0f / (float)M_PI);

            // 5. 格式化輸出歐拉角姿態至終端機
            printf("姿態角度 -> Roll: %6.1f | Pitch: %6.1f | Yaw: %6.1f\r\n", roll, pitch, yaw);
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // 🌟 [重中之重] 物理性強制定向中斷向量表 (VTOR)
  // 避免冷啟動時因為 system_stm32n6xx.c 中未啟用 USER_VECT_TAB_ADDRESS
  // 導致 HAL_Init() 一開啟 SysTick 中斷 CPU 就因為找不到向量而當場卡死！
  #if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    SCB->VTOR = 0x24000400; // 安全 AXI SRAM1 起點
  #else
    SCB->VTOR = 0x34000400; // 非安全 AXI SRAM1 起點
  #endif

  // 🌟 [冷啟動關鍵防線] 強制讓 CMSIS 與 HAL 庫讀取並同步當前系統時鐘變數
  SystemCoreClockUpdate();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  // 🚀 [除錯優先] 將 USART1 調整至最優先初始化，確保任何開機日誌皆能第一時間送出
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  // 🌟 [冷啟動列舉防護延遲] 故意等待 1.5 秒
  // 給予電腦主機充裕的時間完成 VCP (COM8) 的 USB 枚舉與連接建立
  HAL_Delay(1500);

  printf("\r\n==============================================\r\n");
  printf("=== STM32N657X0 BNO085 姿態解算引擎啟動 ===\r\n");
  printf("==============================================\r\n");
  printf("[SYS] 向量表偏移暫存器 (VTOR) 已強制設定為: 0x%08lX\r\n", SCB->VTOR);
  printf("[SYS] 系統當前核心時脈 (Core Clock): %lu Hz\r\n", SystemCoreClock);
  printf("==============================================\r\n");

  // 初始化 I2C4 與 BNO085
  MX_I2C4_Init();

  // 1. 強制重置 BNO085 (硬體引腳拉低復位)
  printf("[BNO085] 正在執行硬體重置...\r\n");
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // RST 拉低
  HAL_Delay(100);                                        // 保持 100ms 確保徹底重置
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // RST 拉高放開
  HAL_Delay(600);                                        // 給感測器充裕的開機引導時間
  printf("[BNO085] 硬體重置完成。\r\n");

  // [安全考量] 清除可能在引導或上電期間殘留的懸空 EXTI 中斷標誌，避免一開啟就卡死
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(GPIO_PIN_11);
  bno_data_ready = 0;

  // 2. I2C 總線診斷掃描
  I2C4_Scan();

  // 3. 進行地址就緒確認
  if (HAL_I2C_IsDeviceReady(&hi2c4, BNO08x_ADDR, 5, 100) == HAL_OK) {
      printf("-> BNO085 (0x4B) 連線成功！準備訂閱數據...\r\n");
  } else {
      printf("-> BNO085 連線失敗，請檢查實體接線與上拉電阻！\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // --- 任務 A：自動定時喚醒訂閱機制 ---
    // 如果串流尚未成功建立，每 2 秒主動重新發送一次 Rotation Vector 訂閱封包
    if (isStreaming == 0 && (HAL_GetTick() - lastCmdTick > 2000)) {
        printf("[BNO085] 正在發送 Rotation Vector 訂閱指令...\r\n");
        HAL_I2C_Master_Transmit(&hi2c4, BNO08x_ADDR, setFeatureCmd, 21, 100);
        lastCmdTick = HAL_GetTick();
    }

    // --- 任務 B：雙重防線數據接收 (中斷引腳觸發 或 實體 INT 保持低電平) ---
    if (bno_data_ready || HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_RESET)
    {
        bno_data_ready = 0; // 清除軟體中斷旗標
        BNO085_ReadAndParse(); // 執行高效解算
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{
  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10C0ECFF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PD12 (RST Reset Pin) */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE11 (INT Interrupt Pin) */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI11_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI11_IRQn);
}

/* USER CODE BEGIN 4 */
// 當 PE11 觸發外部中斷時，會自動調用此回調函式
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_11) {
        bno_data_ready = 1; // 快速標記數據就緒，通知主迴圈
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
