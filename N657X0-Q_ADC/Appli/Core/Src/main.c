/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Keyes pressure sensor on PA8 / ADC1_INP5
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * NUCLEO-N657X0-Q Arduino A0 is routed to PA8 / ADC1_INP5 through the
 * board analog conditioning circuit. Therefore, the external A0 range is
 * treated as approximately 0 to 3.3 V.
 *
 * IMPORTANT:
 * - Keyes VCC -> 3V3
 * - Keyes GND -> GND
 * - Keyes S   -> Arduino CN4 A0
 * - Do not apply a signal above 1.8 V directly to the MCU-side PA8 pin.
 *
 * If the signal is connected directly to the MCU-side PA8 and is guaranteed
 * to stay within 0 to 1.8 V, change ADC_EXTERNAL_FULL_SCALE_MV to 1800U.
 */
#define ADC_EXTERNAL_FULL_SCALE_MV   3300U
#define ADC_12BIT_MAX_COUNTS         4095U
#define PRESSURE_ADC_AVERAGE_SAMPLES 16U
#define ADC_POLL_TIMEOUT_MS          20U

/*
 * The STM32N6 calibration routine is optional for this Keyes sensor.
 * Set to 1U to attempt calibration; a failure will be reported but will
 * no longer stop the application from testing normal ADC conversions.
 */
#define ADC_TRY_SELF_CALIBRATION      1U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t adc_raw_val = 0U;
volatile uint32_t sensor_voltage_mv = 0U;
volatile HAL_StatusTypeDef adc_read_status = HAL_OK;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void SystemIsolation_Config(void);
/* USER CODE BEGIN PFP */
static void ADC1_PA8_ForceHardwareInit(void);
static HAL_StatusTypeDef Read_Keyes_Pressure_PA8(uint32_t *raw_value,
                                                uint32_t *input_voltage_mv);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1U, HAL_MAX_DELAY);
  return ch;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

/**
  * @brief  Force-enable the STM32N6 analog supply, ADC12 clock, and PA8
  *         analog GPIO configuration.
  * @note   This is a diagnostic safeguard. The same setup should normally
  *         be present in stm32n6xx_hal_msp.c.
  */
static void ADC1_PA8_ForceHardwareInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Required by the official STM32N6 ADC examples. */
  HAL_PWREx_EnableVddA();

  /* Make the test independent of a missing/incomplete HAL_ADC_MspInit(). */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC12_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t raw_value = 0U;
  uint32_t input_voltage_mv = 0U;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /*
   * STM32N6 ADC1/ADC2 require the VDDA analog domain to be enabled.
   * Calling this here is harmless if HAL_MspInit() already enabled it.
   */
  HAL_PWREx_EnableVddA();
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  SystemIsolation_Config();

  /*
   * Re-assert the critical ADC hardware setup before calibration and reading.
   * This bypasses a missing PA8 analog/ADC12 clock setup in the MSP file.
   */
  ADC1_PA8_ForceHardwareInit();
  /* USER CODE BEGIN 2 */
  printf("\r\n============================================\r\n");
  printf(" STM32N657X0-Q Keyes Pressure Sensor\r\n");
  printf(" ADC input: PA8 / ADC1_INP5 (Arduino A0)\r\n");
  printf(" Wiring: VCC->3V3, GND->GND, S->CN4 A0\r\n");
  printf(" System clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
  printf(" VDDA enabled, ADC12 clock forced, PA8 analog mode forced\r\n");
  printf("============================================\r\n");

  /*
   * Allow the board analog supply and ADC clock to stabilize before
   * running the one-time single-ended calibration.
   */
  HAL_Delay(50U);

#if (ADC_TRY_SELF_CALIBRATION == 1U)
  {
    HAL_StatusTypeDef calibration_status;

    printf("[ADC1] Calibration start...\r\n");

    calibration_status =
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    printf("[ADC1] Calibration result: status=%d, state=0x%08lX, "
           "error=0x%08lX\r\n",
           (int)calibration_status,
           HAL_ADC_GetState(&hadc1),
           HAL_ADC_GetError(&hadc1));

    if (calibration_status == HAL_OK)
    {
      printf("[ADC1] Calibration completed.\r\n");
    }
    else
    {
      /*
       * STM32N6 HAL can return HAL_ERROR from its calibration-offset
       * algorithm while leaving HAL_ADC_GetError() equal to zero.
       * Calibration is not mandatory for this pressure-sensor test, so
       * continue and verify the ordinary PA8 conversion path.
       */
      printf("[ADC1] WARNING: Calibration was skipped after failure.\r\n");
      printf("[ADC1] Continuing with normal PA8/ADC1_INP5 conversions.\r\n");
    }

    /*
     * HAL_ADCEx_Calibration_Start() enables the ADC internally.
     * Return it to a known disabled/idle state before normal sampling.
     */
    {
      HAL_StatusTypeDef stop_status = HAL_ADC_Stop(&hadc1);

      if (stop_status != HAL_OK)
      {
        printf("[ADC1] WARNING: HAL_ADC_Stop after calibration returned "
               "status=%d, error=0x%08lX\r\n",
               (int)stop_status,
               HAL_ADC_GetError(&hadc1));
      }
    }
  }
#else
  printf("[ADC1] Self-calibration disabled for sensor-path test.\r\n");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    adc_read_status = Read_Keyes_Pressure_PA8(&raw_value, &input_voltage_mv);

    if (adc_read_status == HAL_OK)
    {
      adc_raw_val = raw_value;
      sensor_voltage_mv = input_voltage_mv;

      /*
       * Integer-only formatting avoids requiring the _printf_float linker
       * option. Example: 1650 mV is printed as 1.650 V.
       */
      printf("[PA8/A0] RAW: %4lu | Input: %lu.%03lu V\r\n",
             adc_raw_val,
             sensor_voltage_mv / 1000U,
             sensor_voltage_mv % 1000U);
    }
    else
    {
      adc_raw_val = 0U;
      sensor_voltage_mv = 0U;

      printf("[PA8/A0] ADC read failed: status=%d, state=0x%08lX, "
             "error=0x%08lX, CR=0x%08lX, ISR=0x%08lX\r\n",
             (int)adc_read_status,
             HAL_ADC_GetState(&hadc1),
             HAL_ADC_GetError(&hadc1),
             hadc1.Instance->CR,
             hadc1.Instance->ISR);
    }

    HAL_Delay(250U);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_246CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the regular channel to be monitored by WatchDog 2 or 3
  */
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;

  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /*
   * Explicitly authorize the ADC1/ADC2 shared ADC12 peripheral.
   * This line is present in ST's official STM32N6 ADC example.
   */
  HAL_RIF_RISC_SetSlaveSecureAttributes(
      RIF_RISC_PERIPH_INDEX_ADC12,
      RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_NPRIV);

  /* set up GPIO configuration */
  HAL_GPIO_ConfigPinAttributes(GPIOA,GPIO_PIN_8,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOA,GPIO_PIN_10,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOA,GPIO_PIN_11,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_0,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_3,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_6,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_7,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_10,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOB,GPIO_PIN_11,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOC,GPIO_PIN_1,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOE,GPIO_PIN_3,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOE,GPIO_PIN_5,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOE,GPIO_PIN_6,GPIO_PIN_SEC|GPIO_PIN_NPRIV);
  HAL_GPIO_ConfigPinAttributes(GPIOH,GPIO_PIN_9,GPIO_PIN_SEC|GPIO_PIN_NPRIV);

  /* USER CODE BEGIN RIF_Init 1 */

  /* USER CODE END RIF_Init 1 */
  /* USER CODE BEGIN RIF_Init 2 */

  /* USER CODE END RIF_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Read the Keyes pressure sensor on PA8 / ADC1_INP5.
  * @param  raw_value Pointer receiving the averaged 12-bit ADC value.
  * @param  input_voltage_mv Pointer receiving the estimated Arduino A0 voltage.
  * @retval HAL status.
  *
  * @note Connect the Keyes signal to Arduino CN4 A0. On this Nucleo board,
  *       A0 is conditioned before reaching the MCU PA8 ADC input.
  */
static HAL_StatusTypeDef Read_Keyes_Pressure_PA8(uint32_t *raw_value,
                                                 uint32_t *input_voltage_mv)
{
  uint64_t sample_sum = 0U;
  uint32_t sample_index;
  HAL_StatusTypeDef status;

  if ((raw_value == NULL) || (input_voltage_mv == NULL))
  {
    return HAL_ERROR;
  }

  *raw_value = 0U;
  *input_voltage_mv = 0U;

  for (sample_index = 0U;
       sample_index < PRESSURE_ADC_AVERAGE_SAMPLES;
       sample_index++)
  {
    status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return status;
    }

    status = HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return status;
    }

    sample_sum += HAL_ADC_GetValue(&hadc1);

    status = HAL_ADC_Stop(&hadc1);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  /*
   * Rounded average of 16 conversions. The longer sampling time configured
   * in MX_ADC1_Init() and this averaging both help reduce sensor noise.
   */
  *raw_value = (uint32_t)
      ((sample_sum + (PRESSURE_ADC_AVERAGE_SAMPLES / 2U)) /
       PRESSURE_ADC_AVERAGE_SAMPLES);

  /*
   * Convert the averaged ADC count to the estimated voltage at Arduino A0.
   * With ADC_EXTERNAL_FULL_SCALE_MV = 3300, 4095 counts represents 3.3 V
   * at the external A0 connector after accounting for board conditioning.
   */
  *input_voltage_mv =
      ((*raw_value * ADC_EXTERNAL_FULL_SCALE_MV) +
       (ADC_12BIT_MAX_COUNTS / 2U)) /
      ADC_12BIT_MAX_COUNTS;

  return HAL_OK;
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
