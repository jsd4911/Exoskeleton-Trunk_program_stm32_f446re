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

//------AI----------//
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"

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


// --- AI 核心控制變數 ---
ai_handle network_handle = AI_HANDLE_NULL;
ai_buffer *ptr_ai_in;
ai_buffer *ptr_ai_out;

// AI 運算用的暫存記憶體 (Activation Buffer)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

// --- 感測器滑動視窗與防抖變數 ---
float my_ai_in[20];  // 給 AI 的 [1, 2, 10] 扁平化資料
float my_ai_out[4];  // AI 輸出的 4 個狀態機率

float temp_pitch_array[10];
float temp_delta_array[10];
uint8_t window_index = 0;

float last_pitch = 0.0f;
float delta_buffer[10] = {0};
uint8_t delta_idx = 0;

uint8_t is_calibrated = 0;
float offset_pitch = 0.0f;


// BNO08x_ADDR 和 setFeatureCmd 請確保您在全域有宣告

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
//-------AI初始化大腦 (v10.2.0 正確指標版) ------------//
void My_AI_Init(void) {
    ai_error err;

    // 🌟 核心修正：將 activations 包裝成「指標陣列」交給 API
    ai_handle act_addr[] = { activations };

    // 傳入包裝好的 act_addr
    err = ai_network_create_and_init(&network_handle, act_addr, NULL);

    if (err.type != AI_ERROR_NONE) {
        char msg[60];
        // 萬一還有錯，把原廠錯誤代碼印出來讓我們抓蟲
        sprintf(msg, "AI Init Error! Type:%d Code:%d\r\n", err.type, err.code);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    } else {
        // 成功啟動後，向引擎索取輸入與輸出的指標
        ptr_ai_in = ai_network_inputs_get(network_handle, NULL);
        ptr_ai_out = ai_network_outputs_get(network_handle, NULL);

        char msg[] = "AI Engine Started!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
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

    My_AI_Init(); // 啟動神經網路
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

	                            // ==========================================
	                            // 🧠 AI 神經傳導邏輯 V4：相對角度與速度防抖引擎
	                            // ==========================================

	                            // 1. 開機自動歸零 (Auto-Tare)
	                            if (is_calibrated == 0) {
	                                offset_pitch = pitch;
	                                last_pitch = 0.0f; // 歸零初始值
	                                is_calibrated = 1;
	                                printf("--- 感測器 V4 歸零完成！ ---\r\n");
	                            }

	                            // 2. 特徵一：計算標準化 Pitch (站直絕對是 0)
	                            float pitch_normalized = pitch - offset_pitch;

	                            // 3. 特徵二：計算平滑變化速度 (Smoothed Delta)
	                            float raw_delta = pitch_normalized - last_pitch;
	                            last_pitch = pitch_normalized; // 更新歷史紀錄

	                            // 將原始變化量放入環形緩衝區 (10個點平均)
	                            delta_buffer[delta_idx] = raw_delta;
	                            delta_idx = (delta_idx + 1) % 10;

	                            float delta_pitch_smoothed = 0.0f;
	                            for(int i = 0; i < 10; i++) {
	                                delta_pitch_smoothed += delta_buffer[i];
	                            }
	                            delta_pitch_smoothed /= 10.0f; // 算出避震過後的純淨速度

	                            // 💡 【修正極性】
	                            // 如果您發現現在彎腰是負的，但當初訓練是正的，請把這行的註解打開，強制反轉極性：
	                            // pitch_normalized = pitch_normalized * -1.0f;
	                            // delta_pitch_smoothed = delta_pitch_smoothed * -1.0f;

	                            // 4. 連續滑動視窗引擎 (FIFO: 先進先出)
	                            // 把舊資料全部往左推一格
	                            for(int i = 0; i < 9; i++) {
	                                temp_pitch_array[i] = temp_pitch_array[i+1];
	                                temp_delta_array[i] = temp_delta_array[i+1];
	                            }
	                            // 將最新的一筆資料放在陣列最後面
	                            temp_pitch_array[9] = pitch_normalized;
	                            temp_delta_array[9] = delta_pitch_smoothed;

	                            // 為了確保開機前 10 筆資料還沒填滿時不要亂猜，我們加一個簡單的計數器
	                            if (window_index < 10) {
	                                window_index++;
	                            }

	                            // 5. 只要陣列一滿，"每一幀" 都觸發 AI 推論！(速度將恢復成與感測器同步)
	                            if (window_index >= 10) {
	                                // 【通道對齊】前 10 個放 Pitch，後 10 個放 Delta
	                                for(int i = 0; i < 10; i++) my_ai_in[i]      = temp_pitch_array[i];
	                                for(int i = 0; i < 10; i++) my_ai_in[10 + i] = temp_delta_array[i];

	                                ptr_ai_in[0].data = AI_HANDLE_PTR(my_ai_in);
	                                ptr_ai_out[0].data = AI_HANDLE_PTR(my_ai_out);

	                                // 執行推論！
	                                if (ai_network_run(network_handle, ptr_ai_in, ptr_ai_out) == 1) {
	                                    int current_state = 0;
	                                    float max_prob = my_ai_out[0];
	                                    for(int i = 1; i < 4; i++){
	                                        if(my_ai_out[i] > max_prob){
	                                            max_prob = my_ai_out[i];
	                                            current_state = i;
	                                        }
	                                    }

	                                    // 印出結果 (現在速度會變得飛快！)
	                                    printf("State: %d | Pitch: %5.1f | Spd: %5.2f\r\n",
	                                           current_state, temp_pitch_array[9], temp_delta_array[9]);
	                                }
	                            }
	                            // ==========================================
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = 2;
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
