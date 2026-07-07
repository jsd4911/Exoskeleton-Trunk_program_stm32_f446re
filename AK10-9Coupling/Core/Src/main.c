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
#include <math.h>
#include "AK10_9_Driver.h"
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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 🌟 儲存馬達回傳的真實角度 (單位: 弧度 Radian)
float motorA_actual_pos = 0.0f;
float motorB_actual_pos = 0.0f;

// 儲存馬達的開機初始位置
float motorA_offset = 0.0f;
int is_motorA_offset_set = 0;
float motorB_offset = 0.0f;
int is_motorB_offset_set = 0;
// 接收大腦板資料的緩衝區 (8 個 bytes)
uint8_t rx_buffer[13];

// 🌟 防抽動升級 1：將初始目標改為「水平伸直」(對應馬達 0 度)
// 這裡是非常關鍵的修改！L1(0.15) + L2(0.20) = 0.35
float target_X = 0.35f;
float target_Y = 0.0f;

// 🌟 防抽動升級 2：新增「平滑座標」緩衝變數
float smooth_X = 0.35f;
float smooth_Y = 0.0f;

// 機構常數 (圖紙上的 15cm 與 20cm)
const float L1 = 0.15f;
const float L2 = 0.20f;

const float g = 9.81f;       // 重力加速度
const float m_B = 0.5f;      // 假設連桿加上末端負載大約 0.5 公斤 (請依實際情況調整！)

float current_Kp = 0.0f; // 開機預設剛性為 0
const float MAX_Kp = 0.2f; // 您最終想要的工作剛性

// 共用體 (維持在全域變數區)
union {
    uint8_t bytes[4];
    float fval;
} data_converter;

// 把 printf 導向 USART2 (也就是通往電腦 COM5 的 USB)
int _write(int file, char *ptr, int len) {
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

    //向編譯器宣告這個結構體
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14; // 非常重要：告訴系統 14 號以後是給 CAN2 用的

    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // --- 設定 CAN2 過濾器 (給 B 馬達) ---
    sFilterConfig.FilterBank = 14; // CAN2 從 14 號開始
    HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    AK10_9_EnableMotor(&hcan1, 0x01);
    HAL_Delay(10);  // 給馬達晶片 10 毫秒的反應時間，避免指令擠在一起
    AK10_9_EnableMotor(&hcan2, 0x01);
    HAL_Delay(10);

    // 給馬達一點時間進入控制模式
    HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

          // ================= 📝 靜態變數宣告區 =================
          static int is_brain_ready = 0;
          static float current_pitch = 0.0f;
          static float current_Kp = 0.0f;
          static float tau_ff_A = 0.0f;

          #define STATE_IDLE 0
          #define STATE_BENDING_DOWN 1
          #define STATE_RETURNING_UP 2
          #define STATE_HOLDING 3
          static int current_state = STATE_IDLE;

          static float theta_B_zero = 0.0f;
          static int is_ik_init = 0;
          static float smooth_cmd_A = 0.0f;
          static float smooth_cmd_B = 0.0f;
          static int is_cmd_init = 0;

          // 🌟 終極升級：1000 Bytes 圓形循環 DMA 緩衝區 (Immortal DMA)
          #define RX_BUF_SIZE 1000
          static uint8_t rx_buf[RX_BUF_SIZE];
          static uint16_t read_ptr = 0;

          static uint32_t last_rx_tick = 0;
          static uint32_t last_can_tx_tick = 0;
          static int system_init = 0;
          static uint16_t heartbeat = 0;

          while (1)
          {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

            // 第一次執行時，啟動不死 DMA
            if (system_init == 0) {
                last_rx_tick = HAL_GetTick();

                // 強制停止之前的任何接收
                HAL_UART_AbortReceive(&huart3);

                // 🔥 強制改為 DMA 圓形循環模式 (Circular)，永不停止！
                if (huart3.hdmarx != NULL) {
                    huart3.hdmarx->Init.Mode = DMA_CIRCULAR;
                    HAL_DMA_Init(huart3.hdmarx);
                }
                // 啟動接收 (這輩子只呼叫這一次！)
                HAL_UART_Receive_DMA(&huart3, rx_buf, RX_BUF_SIZE);

                // 🔪 拔掉 HAL 的管轄權：關閉錯誤中斷，發生 ORE 也不會被強迫 Abort！
                CLEAR_BIT(huart3.Instance->CR3, USART_CR3_EIE);

                system_init = 1;
            }

            // ================= 📡 1. 默默清除硬體錯誤 =================
            if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET) {
                __HAL_UART_CLEAR_OREFLAG(&huart3);
                volatile uint32_t tmpreg = huart3.Instance->DR; (void)tmpreg;
            }
            if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE) != RESET) {
                __HAL_UART_CLEAR_NEFLAG(&huart3);
            }
            if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE) != RESET) {
                __HAL_UART_CLEAR_FEFLAG(&huart3);
            }

            // ================= 🧠 2. 非同步追蹤與解析 =================
            uint16_t write_ptr = 0;
            if (huart3.hdmarx != NULL) {
                write_ptr = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
                if (write_ptr == RX_BUF_SIZE) write_ptr = 0; // 防呆
            }

            // 🌟 環形緩衝區的魔術
            uint16_t bytes_available = 0;
            if (write_ptr >= read_ptr) {
                bytes_available = write_ptr - read_ptr;
            } else {
                bytes_available = RX_BUF_SIZE - read_ptr + write_ptr;
            }

            // 只要湊滿 13 Bytes，就開始滑動解析
            while (bytes_available >= 13) {
                // 使用 % RX_BUF_SIZE 確保跨越陣列尾端時能完美繞回頭部
                if (rx_buf[read_ptr] == 0xAA &&
                    rx_buf[(read_ptr + 1) % RX_BUF_SIZE] == 0x55 &&
                    rx_buf[(read_ptr + 11) % RX_BUF_SIZE] == 0x0D &&
                    rx_buf[(read_ptr + 12) % RX_BUF_SIZE] == 0x0A) {

                    // 成功抓到一包合法特徵碼
                    uint8_t status = rx_buf[(read_ptr + 2) % RX_BUF_SIZE];
                    (void)status; // 👈 加上這行，告訴編譯器「我知道有這個變數，不要煩我」
                    data_converter.bytes[0] = rx_buf[(read_ptr + 3) % RX_BUF_SIZE];
                    data_converter.bytes[1] = rx_buf[(read_ptr + 4) % RX_BUF_SIZE];
                    data_converter.bytes[2] = rx_buf[(read_ptr + 5) % RX_BUF_SIZE];
                    data_converter.bytes[3] = rx_buf[(read_ptr + 6) % RX_BUF_SIZE];
                    float temp_X = data_converter.fval;

                    read_ptr = (read_ptr + 13) % RX_BUF_SIZE;
                    bytes_available -= 13;

                    // 物理合理性防波堤
                    if (!isnan(temp_X) && temp_X > -180.0f && temp_X < 180.0f) {
                        last_rx_tick = HAL_GetTick(); // 🌟 收到健康封包，立刻刷新斷線計時器！

                        // 🔥🔥🔥 終極暴力破解：無視大腦板的 Status=0，直接強制啟動控制！
                        is_brain_ready = 1;

                        float delta_pitch = temp_X - current_pitch;
                        current_pitch = temp_X;

                        float target_Kp = 0.0f;
                        float target_tau = 0.0f;

                        // 狀態機邏輯
                        if (current_pitch < 15.0f && current_pitch > -15.0f) {
                            current_state = STATE_IDLE;
                        } else if (current_pitch > 45.0f && fabs(delta_pitch) < 0.2f) {
                            current_state = STATE_HOLDING;
                        } else if (delta_pitch > 0.1f) {
                            current_state = STATE_BENDING_DOWN;
                        } else if (delta_pitch < -0.1f) {
                            current_state = STATE_RETURNING_UP;
                        } else {
                            if (current_state == STATE_IDLE) {
                                current_state = STATE_BENDING_DOWN;
                            }
                        }

                        // 目標剛性與扭力
                        switch (current_state) {
                            case STATE_IDLE:
                                target_Kp = 15.0f;
                                target_tau = 0.0f;
                                break;
                            case STATE_BENDING_DOWN:
                                target_Kp = 5.0f;
                                target_tau = 0.5f * 9.8f * 0.175f * sinf(current_pitch * 3.14159f / 180.0f) * 0.8f;
                                break;
                            case STATE_HOLDING:
                                target_Kp = 25.0f;
                                target_tau = 0.5f * 9.8f * 0.175f * sinf(current_pitch * 3.14159f / 180.0f);
                            break;
                            case STATE_RETURNING_UP:
                                target_Kp = 15.0f;
                                target_tau = 0.5f * 9.8f * 0.175f * sinf(current_pitch * 3.14159f / 180.0f) + 3.0f;
                                break;
                        }

                        float smooth_factor = 0.5f;
                        current_Kp += (target_Kp - current_Kp) * smooth_factor;
                        tau_ff_A += (target_tau - tau_ff_A) * smooth_factor;

                        // 雙軸 IK 運算
                        float motor_cmd_A;
                        float motor_cmd_B;
                        float L1_len = 0.20f;
                        float L2_len = 0.075f;
                        float base_radius = 0.24f;

                        if (is_ik_init == 0) {
                            float cos_phi_zero = (L1_len*L1_len + L2_len*L2_len - base_radius*base_radius) / (2 * L1_len * L2_len);
                            if(cos_phi_zero > 1.0f) cos_phi_zero = 1.0f;
                            if(cos_phi_zero < -1.0f) cos_phi_zero = -1.0f;
                            theta_B_zero = 3.14159f - acosf(cos_phi_zero);
                            is_ik_init = 1;
                        }

                        if (current_state == STATE_IDLE) {
                            motor_cmd_A = motorA_offset;
                            motor_cmd_B = motorB_offset;
                        } else {
                            motor_cmd_A = -(current_pitch * 3.14159f / 180.0f) + motorA_offset;
                            float elongation = (current_pitch / 90.0f) * 0.03f;
                            if (elongation < 0.0f) elongation = 0.0f;
                            float current_radius = base_radius + elongation;

                            float cos_phi = (L1_len*L1_len + L2_len*L2_len - current_radius*current_radius) / (2 * L1_len * L2_len);
                            if(cos_phi > 1.0f) cos_phi = 1.0f;
                            if(cos_phi < -1.0f) cos_phi = -1.0f;

                            motor_cmd_B = -((3.14159f - acosf(cos_phi)) - theta_B_zero) + motorB_offset;
                        }

                        if (is_cmd_init == 0) {
                            smooth_cmd_A = motorA_offset;
                            smooth_cmd_B = motorB_offset;
                            is_cmd_init = 1;
                        }

                        smooth_cmd_A += (motor_cmd_A - smooth_cmd_A) * 0.80f;
                        smooth_cmd_B += (motor_cmd_B - smooth_cmd_B) * 0.10f;

                        // 🚀 發送指令
                        AK10_9_SendCommand(&hcan1, 0x01, smooth_cmd_A, 0.0f, current_Kp, 1.5f, tau_ff_A);
                        AK10_9_SendCommand(&hcan2, 0x01, smooth_cmd_B, 0.0f, current_Kp * 0.4f, 1.0f, 0.0f);
                    }
                } else {
                    // 找不到標頭，滑動 1 Byte 繼續找
                    read_ptr = (read_ptr + 1) % RX_BUF_SIZE;
                    bytes_available--;
                }
            }

            // ================= 🛡️ 3. 非阻塞式斷線保護 =================
            if (HAL_GetTick() - last_rx_tick > 150) {
                is_brain_ready = 0;
                current_Kp = 0.0f;
                tau_ff_A = 0.0f;

                if (HAL_GetTick() - last_can_tx_tick >= 10) {
                    AK10_9_SendCommand(&hcan1, 0x01, smooth_cmd_A, 0.0f, 0.0f, 0.1f, 0.0f);
                    AK10_9_SendCommand(&hcan2, 0x01, smooth_cmd_B, 0.0f, 0.0f, 0.1f, 0.0f);
                    last_can_tx_tick = HAL_GetTick();
                }
            }

            // ================= 🖨️ 4. 神級偵錯 Printf =================
            static uint32_t last_print_tick = 0;
            if (HAL_GetTick() - last_print_tick >= 100) {
                heartbeat++;
                if (is_brain_ready) {
                    printf("💓[%d] State: %d | Pitch: %.1f | CmdA: %.1f | CmdB: %.1f | Kp: %.2f | RX: %d\r\n",
                           heartbeat, current_state, current_pitch, smooth_cmd_A, smooth_cmd_B, current_Kp, write_ptr);
                } else {
                    if (HAL_GetTick() - last_rx_tick <= 150) {
                        printf("🔗[%d] 通訊正常！暖機中(Status=0) | RX指標: %d\r\n", heartbeat, write_ptr);
                    } else {
                        printf("❌[%d] 實體斷線！150ms沒封包 | RX: %d | Kp: 0.00\r\n", heartbeat, write_ptr);
                    }
                }
                last_print_tick = HAL_GetTick();
            }

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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        // 提取馬達位置 (Byte 1, Byte 2)
    	// 🌟 請務必將 int16_t 改為 uint16_t ！！！
    	uint16_t pos_int = (RxData[1] << 8) | RxData[2];
    	float position = ((float)pos_int * 25.0f / 65535.0f) - 12.5f;

        // 🌟 自動分流：判斷是 CAN1 還是 CAN2 收到的
        if (hcan->Instance == CAN1) {
            // 來自 A 馬達
            if (is_motorA_offset_set == 0) {
                motorA_offset = position;
                is_motorA_offset_set = 1;
            }
            motorA_actual_pos = position - motorA_offset;

        } else if (hcan->Instance == CAN2) {
            // 來自 B 馬達
            if (is_motorB_offset_set == 0) {
                motorB_offset = position;
                is_motorB_offset_set = 1;
            }
            motorB_actual_pos = position - motorB_offset;
        }
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
