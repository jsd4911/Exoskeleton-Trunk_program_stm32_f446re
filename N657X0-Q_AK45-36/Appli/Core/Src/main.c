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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  AK45_REQUEST_NONE = 0U,
  AK45_REQUEST_DISABLE = 1U,
  AK45_REQUEST_SAFE_HANDSHAKE = 2U,
  AK45_REQUEST_MOVE_PLUS_0P10_RAD = 3U,
  AK45_REQUEST_MOVE_ZERO_RAD = 4U
} AK45_RequestTypeDef;

typedef enum
{
  AK45_SEQUENCE_IDLE = 0U,
  AK45_SEQUENCE_WAIT_ZERO_COMMAND = 1U,
  AK45_SEQUENCE_WAIT_MOTION_COMMAND = 2U,
  AK45_SEQUENCE_WAIT_AUTO_DISABLE = 3U
} AK45_SequenceTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AK45_CAN_ID                     0x001U

#define AK45_POSITION_MIN_RAD          (-12.5F)
#define AK45_POSITION_MAX_RAD            12.5F
#define AK45_VELOCITY_MIN_RAD_S         (-6.0F)
#define AK45_VELOCITY_MAX_RAD_S           6.0F
#define AK45_TORQUE_MIN_NM             (-34.0F)
#define AK45_TORQUE_MAX_NM               34.0F
#define AK45_KP_MIN                       0.0F
#define AK45_KP_MAX                     500.0F
#define AK45_KD_MIN                       0.0F
#define AK45_KD_MAX                       5.0F

/* Position commands require this one-time safety unlock key. */
#define AK45_MOTION_UNLOCK_KEY         0xA4536001UL

/* Internal execution result codes; these are not CubeMars CAN commands. */
#define AK45_RESULT_READY              0x00000001UL
#define AK45_RESULT_RUNNING            0x00000010UL
#define AK45_RESULT_OK                 0x00000020UL
#define AK45_RESULT_MOTION_LOCKED      0x000000E1UL
#define AK45_RESULT_BUSY               0x000000E2UL
#define AK45_RESULT_BAD_REQUEST        0x000000E3UL
#define AK45_RESULT_TX_ERROR           0x000000E4UL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */

/* Working buffers used by FDCAN transmit and receive operations. */
static FDCAN_TxHeaderTypeDef can_tx_header;
static uint8_t can_tx_work_data[8];
static FDCAN_RxHeaderTypeDef can_rx_header;
static uint8_t can_rx_work_data[8];

/*
 * Add these symbols to STM32CubeIDE Live Expressions while debugging.
 * The firmware never transmits at boot. A command is sent only after the user
 * writes a request value to ak45_request.
 */
volatile uint32_t can_status = 0U;
volatile uint32_t can_tx_count = 0U;
volatile uint32_t can_tx_error_count = 0U;
volatile uint32_t can_rx_count = 0U;
volatile uint32_t can_rx_error_count = 0U;
volatile uint32_t can_last_id = 0U;
volatile uint32_t can_last_id_type = 0U;
volatile uint32_t can_last_dlc = 0U;
volatile uint32_t can_tx_fifo_free = 0U;
volatile uint32_t can_last_hal_error = 0U;
volatile uint32_t can_protocol_last_error = 0U;
volatile uint32_t can_protocol_activity = 0U;
volatile uint32_t can_protocol_error_passive = 0U;
volatile uint32_t can_protocol_warning = 0U;
volatile uint32_t can_protocol_bus_off = 0U;
volatile uint32_t can_tx_error_counter = 0U;
volatile uint32_t can_rx_error_counter = 0U;
volatile uint8_t can_last_data[8] = {0U};

/* User command and AK45-36 decoded feedback variables. */
volatile uint32_t ak45_request = AK45_REQUEST_NONE;
volatile uint32_t ak45_motion_unlock = 0U;
volatile uint32_t ak45_last_request = AK45_REQUEST_NONE;
volatile uint32_t ak45_result = AK45_RESULT_READY;
volatile uint32_t ak45_sequence = AK45_SEQUENCE_IDLE;
volatile uint32_t ak45_enabled = 0U;
volatile uint32_t ak45_reply_count = 0U;
volatile uint32_t ak45_reply_id = 0U;
volatile uint32_t ak45_error_code = 0U;
volatile float ak45_position_rad = 0.0F;
volatile float ak45_velocity_rad_s = 0.0F;
volatile float ak45_torque_nm = 0.0F;
volatile float ak45_temperature_c = 0.0F;

static uint32_t ak45_sequence_deadline_ms = 0U;
static float ak45_pending_position_rad = 0.0F;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void SystemIsolation_Config(void);
/* USER CODE BEGIN PFP */

static void FDCAN1_AK45_Init(void);
static uint32_t FDCAN_DlcToBytes(uint32_t data_length);

static uint32_t AK45_FloatToUInt(float value,
                                float minimum,
                                float maximum,
                                uint32_t bits);

static float AK45_UIntToFloat(uint32_t value,
                             float minimum,
                             float maximum,
                             uint32_t bits);

static HAL_StatusTypeDef AK45_SendRawFrame(const uint8_t data[8]);
static HAL_StatusTypeDef AK45_SendSpecialCommand(uint8_t command);

static HAL_StatusTypeDef AK45_SendMITCommand(float position_rad,
                                             float velocity_rad_s,
                                             float kp,
                                             float kd,
                                             float torque_nm);

static void AK45_Service(void);
static void AK45_FinishWithTxError(void);
static void AK45_DecodeReply(const uint8_t data[8]);
static void FDCAN1_UpdateDiagnostics(void);

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  SystemIsolation_Config();
  /* USER CODE BEGIN 2 */

  /* Start FDCAN1. No CAN message is transmitted until ak45_request is set. */
  FDCAN1_AK45_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  AK45_Service();
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
  hfdcan1.Init.ProtocolException = ENABLE;
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

/**
  * @brief  Convert an STM32 FDCAN DLC constant to a Classic CAN byte count.
  * @param  data_length Value from FDCAN_RxHeaderTypeDef.DataLength.
  * @retval Number of valid bytes (0 to 8).
  */
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

/**
  * @brief  Configure and start FDCAN1 for the AK45-36 MIT protocol.
  * @note   Accepts every 11-bit standard identifier (0x000 to 0x7FF).
  * @retval None
  */
static void FDCAN1_AK45_Init(void)
{
  FDCAN_FilterTypeDef filter = {0};

  can_status = 0x10U; /* Configuring the standard-ID receive filter. */

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000U;
  filter.FilterID2 = 0x7FFU;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    can_status = 0xE1U; /* Filter configuration failed. */
    Error_Handler();
  }

  /* Reject unmatched frames and all remote frames. */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    can_status = 0xE4U; /* Global filter configuration failed. */
    Error_Handler();
  }

  can_status = 0x20U; /* Starting FDCAN1. */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    can_status = 0xE2U; /* FDCAN start failed. */
    Error_Handler();
  }

  can_status = 0x30U; /* Enabling the FIFO0 new-message interrupt. */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0U) != HAL_OK)
  {
    can_status = 0xE3U; /* Notification activation failed. */
    Error_Handler();
  }

  can_tx_header.Identifier = AK45_CAN_ID;
  can_tx_header.IdType = FDCAN_STANDARD_ID;
  can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
  can_tx_header.DataLength = FDCAN_DLC_BYTES_8;
  can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  can_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  can_tx_header.MessageMarker = 0U;

  can_status = 0x01U; /* AK45-36 application is ready; no frame sent yet. */
  ak45_result = AK45_RESULT_READY;
  FDCAN1_UpdateDiagnostics();
}

/**
  * @brief  Convert a bounded floating-point value to an unsigned bit field.
  * @note   Uses (2^bits - 1), so both endpoints map without wraparound.
  */
static uint32_t AK45_FloatToUInt(float value,
                                float minimum,
                                float maximum,
                                uint32_t bits)
{
  uint32_t maximum_integer;
  float scaled;

  if (value < minimum)
  {
    value = minimum;
  }
  else if (value > maximum)
  {
    value = maximum;
  }

  maximum_integer = (1UL << bits) - 1UL;
  scaled = ((value - minimum) * (float)maximum_integer) /
           (maximum - minimum);

  return (uint32_t)scaled;
}

/**
  * @brief  Convert an unsigned MIT bit field back to a floating-point value.
  */
static float AK45_UIntToFloat(uint32_t value,
                             float minimum,
                             float maximum,
                             uint32_t bits)
{
  uint32_t maximum_integer = (1UL << bits) - 1UL;

  return ((float)value * (maximum - minimum) /
          (float)maximum_integer) + minimum;
}
/**
  * @brief  Queue one Classic CAN data frame for motor ID 1.
  */
static HAL_StatusTypeDef AK45_SendRawFrame(const uint8_t data[8])
{
  HAL_StatusTypeDef status;
  uint32_t index;

  for (index = 0U; index < 8U; index++)
  {
    can_tx_work_data[index] = data[index];
  }

  status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
                                         &can_tx_header,
                                         can_tx_work_data);
  if (status == HAL_OK)
  {
    can_tx_count++;
  }
  else
  {
    can_tx_error_count++;
    can_last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
  }

  FDCAN1_UpdateDiagnostics();
  return status;
}
/**
  * @brief Send an AK45-36 MIT special command.
  * @param command 0xFC enter, 0xFD exit, 0xFE set origin.
  */
static HAL_StatusTypeDef AK45_SendSpecialCommand(uint8_t command)
{
  uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                     0xFFU, 0xFFU, 0xFFU, command};

  return AK45_SendRawFrame(data);
}
/**
  * @brief Pack and send one AK45-36 MIT command.
  * @note  AK45-36 ranges from CubeMars AK 2.0 Driver Manual V1.0.18:
  *        position +/-12.5 rad, velocity +/-6 rad/s, torque +/-34 N.m,
  *        Kp 0..500 and Kd 0..5.
  */
static HAL_StatusTypeDef AK45_SendMITCommand(float position_rad,
                                             float velocity_rad_s,
                                             float kp,
                                             float kd,
                                             float torque_nm)
{
  uint32_t position_integer;
  uint32_t velocity_integer;
  uint32_t kp_integer;
  uint32_t kd_integer;
  uint32_t torque_integer;
  uint8_t data[8];

  position_integer = AK45_FloatToUInt(position_rad,
                                      AK45_POSITION_MIN_RAD,
                                      AK45_POSITION_MAX_RAD,
                                      16U);

  velocity_integer = AK45_FloatToUInt(velocity_rad_s,
                                      AK45_VELOCITY_MIN_RAD_S,
                                      AK45_VELOCITY_MAX_RAD_S,
                                      12U);

  kp_integer = AK45_FloatToUInt(kp,
                                AK45_KP_MIN,
                                AK45_KP_MAX,
                                12U);

  kd_integer = AK45_FloatToUInt(kd,
                                AK45_KD_MIN,
                                AK45_KD_MAX,
                                12U);

  torque_integer = AK45_FloatToUInt(torque_nm,
                                    AK45_TORQUE_MIN_NM,
                                    AK45_TORQUE_MAX_NM,
                                    12U);

  data[0] = (uint8_t)(position_integer >> 8);
  data[1] = (uint8_t)(position_integer & 0xFFU);
  data[2] = (uint8_t)(velocity_integer >> 4);
  data[3] = (uint8_t)(((velocity_integer & 0x0FU) << 4) |
                      (kp_integer >> 8));
  data[4] = (uint8_t)(kp_integer & 0xFFU);
  data[5] = (uint8_t)(kd_integer >> 4);
  data[6] = (uint8_t)(((kd_integer & 0x0FU) << 4) |
                      (torque_integer >> 8));
  data[7] = (uint8_t)(torque_integer & 0xFFU);

  return AK45_SendRawFrame(data);
}
/**
  * @brief  Mark a sequence failed and make one best-effort disable attempt.
  */
static void AK45_FinishWithTxError(void)
{
  (void)AK45_SendSpecialCommand(0xFDU);
  ak45_enabled = 0U;
  ak45_sequence = AK45_SEQUENCE_IDLE;
  ak45_motion_unlock = 0U;
  ak45_result = AK45_RESULT_TX_ERROR;
}
/**
  * @brief Run user-triggered AK45-36 command sequences without boot motion.
  * @note  Request 1 always aborts the active sequence and sends MIT EXIT.
  */
static void AK45_Service(void)
{
  static uint32_t diagnostics_deadline_ms = 0U;
  uint32_t now_ms = HAL_GetTick();
  uint32_t request;

  if ((int32_t)(now_ms - diagnostics_deadline_ms) >= 0)
  {
    FDCAN1_UpdateDiagnostics();
    diagnostics_deadline_ms = now_ms + 100U;
  }

  /* Request 1 immediately aborts and disables the motor. */
  if (ak45_request == AK45_REQUEST_DISABLE)
  {
    ak45_request = AK45_REQUEST_NONE;
    ak45_last_request = AK45_REQUEST_DISABLE;
    ak45_sequence = AK45_SEQUENCE_IDLE;
    ak45_motion_unlock = 0U;
    ak45_result = AK45_RESULT_RUNNING;

    if (AK45_SendSpecialCommand(0xFDU) == HAL_OK)
    {
      ak45_enabled = 0U;
      ak45_result = AK45_RESULT_OK;
    }
    else
    {
      AK45_FinishWithTxError();
    }

    return;
  }

  if (ak45_sequence == AK45_SEQUENCE_IDLE)
  {
    request = ak45_request;

    if (request == AK45_REQUEST_NONE)
    {
      return;
    }

    ak45_request = AK45_REQUEST_NONE;
    ak45_last_request = request;
    ak45_result = AK45_RESULT_RUNNING;

    if (request == AK45_REQUEST_SAFE_HANDSHAKE)
    {
      if (AK45_SendSpecialCommand(0xFCU) != HAL_OK)
      {
        AK45_FinishWithTxError();
        return;
      }

      ak45_enabled = 1U;
      ak45_sequence_deadline_ms = now_ms + 20U;
      ak45_sequence = AK45_SEQUENCE_WAIT_ZERO_COMMAND;
      return;
    }

    if ((request == AK45_REQUEST_MOVE_PLUS_0P10_RAD) ||
        (request == AK45_REQUEST_MOVE_ZERO_RAD))
    {
      if (ak45_motion_unlock != AK45_MOTION_UNLOCK_KEY)
      {
        ak45_motion_unlock = 0U;
        ak45_result = AK45_RESULT_MOTION_LOCKED;
        return;
      }

      ak45_pending_position_rad =
        (request == AK45_REQUEST_MOVE_PLUS_0P10_RAD) ? 0.10F : 0.0F;

      if (AK45_SendSpecialCommand(0xFCU) != HAL_OK)
      {
        AK45_FinishWithTxError();
        return;
      }

      ak45_enabled = 1U;
      ak45_sequence_deadline_ms = now_ms + 20U;
      ak45_sequence = AK45_SEQUENCE_WAIT_MOTION_COMMAND;
      return;
    }

    ak45_result = AK45_RESULT_BAD_REQUEST;
    return;
  }

  /* Reject another non-disable request while a sequence is running. */
  if (ak45_request != AK45_REQUEST_NONE)
  {
    ak45_request = AK45_REQUEST_NONE;
    ak45_result = AK45_RESULT_BUSY;
  }

  if ((int32_t)(now_ms - ak45_sequence_deadline_ms) < 0)
  {
    return;
  }

  if (ak45_sequence == AK45_SEQUENCE_WAIT_ZERO_COMMAND)
  {
    /*
     * Kp, Kd and torque are all zero.
     * This requests feedback without commanding motor torque.
     */
    if (AK45_SendMITCommand(0.0F,
                            0.0F,
                            0.0F,
                            0.0F,
                            0.0F) != HAL_OK)
    {
      AK45_FinishWithTxError();
      return;
    }

    ak45_sequence_deadline_ms = now_ms + 200U;
    ak45_sequence = AK45_SEQUENCE_WAIT_AUTO_DISABLE;
    return;
  }

  if (ak45_sequence == AK45_SEQUENCE_WAIT_MOTION_COMMAND)
  {
    if (AK45_SendMITCommand(ak45_pending_position_rad,
                            0.0F,
                            10.0F,
                            0.10F,
                            0.0F) != HAL_OK)
    {
      AK45_FinishWithTxError();
      return;
    }

    /* Automatically exit MIT mode after the short position test. */
    ak45_sequence_deadline_ms = now_ms + 800U;
    ak45_sequence = AK45_SEQUENCE_WAIT_AUTO_DISABLE;
    return;
  }

  if (ak45_sequence == AK45_SEQUENCE_WAIT_AUTO_DISABLE)
  {
    if (AK45_SendSpecialCommand(0xFDU) != HAL_OK)
    {
      AK45_FinishWithTxError();
      return;
    }

    ak45_enabled = 0U;
    ak45_sequence = AK45_SEQUENCE_IDLE;
    ak45_motion_unlock = 0U;
    ak45_result = AK45_RESULT_OK;
  }
}
/**
  * @brief Decode one 8-byte AK45-36 MIT reply.
  */
static void AK45_DecodeReply(const uint8_t data[8])
{
  uint32_t position_integer;
  uint32_t velocity_integer;
  uint32_t torque_integer;

  ak45_reply_id = (uint32_t)data[0];

  position_integer = ((uint32_t)data[1] << 8) |
                     (uint32_t)data[2];

  velocity_integer = ((uint32_t)data[3] << 4) |
                     ((uint32_t)data[4] >> 4);

  torque_integer = (((uint32_t)data[4] & 0x0FU) << 8) |
                   (uint32_t)data[5];

  ak45_position_rad = AK45_UIntToFloat(position_integer,
                                       AK45_POSITION_MIN_RAD,
                                       AK45_POSITION_MAX_RAD,
                                       16U);

  ak45_velocity_rad_s = AK45_UIntToFloat(velocity_integer,
                                         AK45_VELOCITY_MIN_RAD_S,
                                         AK45_VELOCITY_MAX_RAD_S,
                                         12U);

  ak45_torque_nm = AK45_UIntToFloat(torque_integer,
                                     AK45_TORQUE_MIN_NM,
                                     AK45_TORQUE_MAX_NM,
                                     12U);

  ak45_temperature_c = (float)data[6] - 40.0F;
  ak45_error_code = (uint32_t)data[7];
  ak45_reply_count++;
}
/**
  * @brief Copy FDCAN health fields into debugger-visible variables.
  */
static void FDCAN1_UpdateDiagnostics(void)
{
  FDCAN_ProtocolStatusTypeDef protocol_status = {0};
  FDCAN_ErrorCountersTypeDef error_counters = {0};

  can_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
  can_last_hal_error = HAL_FDCAN_GetError(&hfdcan1);

  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1,
                                  &protocol_status) == HAL_OK)
  {
    can_protocol_last_error = protocol_status.LastErrorCode;
    can_protocol_activity = protocol_status.Activity;
    can_protocol_error_passive = protocol_status.ErrorPassive;
    can_protocol_warning = protocol_status.Warning;
    can_protocol_bus_off = protocol_status.BusOff;
  }

  if (HAL_FDCAN_GetErrorCounters(&hfdcan1,
                                 &error_counters) == HAL_OK)
  {
    can_tx_error_counter = error_counters.TxErrorCnt;
    can_rx_error_counter = error_counters.RxErrorCnt;
  }
}
/**
  * @brief  FDCAN FIFO0 receive callback.
  * @param  hfdcan FDCAN handle that generated the interrupt.
  * @param  RxFifo0ITs FIFO0 interrupt flags.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
  uint32_t index;
  uint32_t data_bytes;

  if ((hfdcan->Instance != FDCAN1) ||
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
  {
    return;
  }

  /* Clear the work buffer so unused bytes never show stale data. */
  for (index = 0U; index < 8U; index++)
  {
    can_rx_work_data[index] = 0U;
  }

  if (HAL_FDCAN_GetRxMessage(hfdcan,
                             FDCAN_RX_FIFO0,
                             &can_rx_header,
                             can_rx_work_data) != HAL_OK)
  {
    can_rx_error_count++;
    return;
  }

  data_bytes = FDCAN_DlcToBytes(can_rx_header.DataLength);
  can_last_id = can_rx_header.Identifier;
  can_last_id_type = can_rx_header.IdType;
  can_last_dlc = data_bytes;

  for (index = 0U; index < 8U; index++)
  {
    can_last_data[index] = can_rx_work_data[index];
  }

  /*
   * Decode only a complete AK45-36 MIT reply.
   * The arbitration ID and data byte 0 must both identify motor 1.
   */
  if ((can_rx_header.IdType == FDCAN_STANDARD_ID) &&
      (can_rx_header.RxFrameType == FDCAN_DATA_FRAME) &&
      (can_rx_header.Identifier == AK45_CAN_ID) &&
      (data_bytes == 8U) &&
      (can_rx_work_data[0] == (uint8_t)AK45_CAN_ID))
  {
    AK45_DecodeReply(can_rx_work_data);
  }

  /* Increment last so a new count indicates a complete snapshot. */
  can_rx_count++;
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
