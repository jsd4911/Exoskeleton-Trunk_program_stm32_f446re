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
#include "ak10_9.h"
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

FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */

/* 開機 Disable 命令狀態 */
volatile uint32_t motor_disable_tx_status = 0U;
volatile uint32_t motor_disable_tx_count = 0U;

/* FDCAN 接收暫存 */
static FDCAN_RxHeaderTypeDef fdcan_rx_header;
static uint8_t fdcan_rx_data[8];

/* 可在 Live Expressions 觀察的除錯變數 */
volatile uint32_t fdcan_status = 0U;
volatile uint32_t fdcan_rx_count = 0U;
volatile uint32_t fdcan_valid_rx_count = 0U;
volatile uint32_t fdcan_rx_error_count = 0U;
volatile uint32_t fdcan_rejected_count = 0U;

volatile uint32_t fdcan_last_id = 0U;
volatile uint32_t fdcan_last_dlc = 0U;
volatile uint8_t fdcan_last_data[8] = {0U};

/* AK10-9 即時回傳資料 */
volatile float motor_position_rad = 0.0f;
volatile float motor_velocity_rad_s = 0.0f;
volatile float motor_torque_nm = 0.0f;
volatile uint32_t motor_temperature_c = 0U;
volatile uint32_t motor_error_code = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void SystemIsolation_Config(void);
/* USER CODE BEGIN PFP */
static void FDCAN1_AK10_9_Init(void);
static uint32_t FDCAN_DlcToBytes(uint32_t data_length);
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  SystemIsolation_Config();
  /* USER CODE BEGIN 2 */

  FDCAN1_AK10_9_Init();

  /*
   * 等待 AK10-9 與 CAN 收發器完成上電，
   * 然後送出一次 Disable 命令。
   */
  HAL_Delay(500U);

  if (AK10_9_Disable(
          &hfdcan1,
          AK10_9_DEFAULT_MOTOR_ID) == HAL_OK)
  {
      motor_disable_tx_status = 0x01U;
      motor_disable_tx_count++;
  }
  else
  {
      motor_disable_tx_status = 0xE1U;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 12;
  hfdcan1.Init.NominalTimeSeg1 = 35;
  hfdcan1.Init.NominalTimeSeg2 = 12;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 3;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 3;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief RIF Initialization Function
  * @param None
  * @retval None
  */
  static void SystemIsolation_Config(void)
{

  /* USER CODE BEGIN RIF_Init 0 */

  /* USER CODE END RIF_Init 0 */

  /* set all required IPs as secure privileged */
  __HAL_RCC_RIFSC_CLK_ENABLE();
  RIMC_MasterConfig_t RIMC_master = {0};
  RIMC_master.MasterCID = RIF_CID_1;
  RIMC_master.SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;

  /*RIMC configuration*/
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_ETH1, &RIMC_master);

  /* RIF-Aware IPs Config */

  /* set up GPIO configuration */
  HAL_GPIO_ConfigPinAttributes(GPIOA,GPIO_PIN_10,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOA,GPIO_PIN_11,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_0,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_3,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_6,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_7,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_10,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_11,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOC,GPIO_PIN_1,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOD,GPIO_PIN_0,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOE,GPIO_PIN_3,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOH,GPIO_PIN_2,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOH,GPIO_PIN_9,GPIO_PIN_SEC|GPIO_PIN_NPRIV);

  /* USER CODE BEGIN RIF_Init 1 */

  /* USER CODE END RIF_Init 1 */
  /* USER CODE BEGIN RIF_Init 2 */

  /* USER CODE END RIF_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* 將 FDCAN DLC 轉換成實際資料長度 */
static uint32_t FDCAN_DlcToBytes(uint32_t data_length)
{
    switch (data_length)
    {
        case FDCAN_DLC_BYTES_0: return 0U;
        case FDCAN_DLC_BYTES_1: return 1U;
        case FDCAN_DLC_BYTES_2: return 2U;
        case FDCAN_DLC_BYTES_3: return 3U;
        case FDCAN_DLC_BYTES_4: return 4U;
        case FDCAN_DLC_BYTES_5: return 5U;
        case FDCAN_DLC_BYTES_6: return 6U;
        case FDCAN_DLC_BYTES_7: return 7U;
        case FDCAN_DLC_BYTES_8: return 8U;
        default:                return 0U;
    }
}


/* 設定 Filter、啟動 FDCAN1 及接收中斷 */
static void FDCAN1_AK10_9_Init(void)
{
    FDCAN_FilterTypeDef filter = {0};

    /*
     * 先接收所有 Standard ID。
     * 部分 AK 系列韌體的回傳 Arbitration ID 可能不同，
     * 後續再以資料中的 Motor ID = 0x01 判斷。
     */
    fdcan_status = 0x10U;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000U;
    filter.FilterID2 = 0x7FFU;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        fdcan_status = 0xE1U;
        Error_Handler();
    }

    /* 拒絕 Extended ID、Remote Frame 及其他未匹配資料 */
    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        fdcan_status = 0xE2U;
        Error_Handler();
    }

    fdcan_status = 0x20U;

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        fdcan_status = 0xE3U;
        Error_Handler();
    }

    fdcan_status = 0x30U;

    if (HAL_FDCAN_ActivateNotification(
            &hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0U) != HAL_OK)
    {
        fdcan_status = 0xE4U;
        Error_Handler();
    }

    /* FDCAN1 初始化及啟動完成 */
    fdcan_status = 0x01U;
}


/* RX FIFO0 收到新封包時由中斷呼叫 */
void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxFifo0ITs)
{
    uint32_t index;
    uint32_t data_bytes;
    AK10_9_FeedbackTypeDef feedback;

    if (hfdcan->Instance != FDCAN1)
    {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    /* 將 FIFO0 內現有的封包全部取出 */
    while (HAL_FDCAN_GetRxFifoFillLevel(
               hfdcan,
               FDCAN_RX_FIFO0) != 0U)
    {
        for (index = 0U; index < 8U; index++)
        {
            fdcan_rx_data[index] = 0U;
        }

        if (HAL_FDCAN_GetRxMessage(
                hfdcan,
                FDCAN_RX_FIFO0,
                &fdcan_rx_header,
                fdcan_rx_data) != HAL_OK)
        {
            fdcan_rx_error_count++;
            return;
        }

        data_bytes =
            FDCAN_DlcToBytes(fdcan_rx_header.DataLength);

        fdcan_last_id = fdcan_rx_header.Identifier;
        fdcan_last_dlc = data_bytes;

        for (index = 0U; index < 8U; index++)
        {
            fdcan_last_data[index] = fdcan_rx_data[index];
        }

        fdcan_rx_count++;

        /* 檢查 Classic CAN Standard Data Frame 並解析馬達資料 */
        if ((fdcan_rx_header.IdType != FDCAN_STANDARD_ID)
            || (fdcan_rx_header.RxFrameType != FDCAN_DATA_FRAME)
            || (fdcan_rx_header.FDFormat != FDCAN_CLASSIC_CAN)
            || !AK10_9_ParseFeedback(
                    fdcan_rx_data,
                    data_bytes,
                    (uint8_t)AK10_9_DEFAULT_MOTOR_ID,
                    &feedback))
        {
            fdcan_rejected_count++;
            continue;
        }

        motor_position_rad = feedback.position_rad;
        motor_velocity_rad_s = feedback.velocity_rad_s;
        motor_torque_nm = feedback.torque_nm;
        motor_temperature_c = feedback.temperature_c;
        motor_error_code = feedback.error_code;

        fdcan_valid_rx_count++;
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
