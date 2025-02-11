#include <Arduino.h>
#include "CAN_Bus_Arduino.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac3;

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef hlpuart1;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

void SystemClock_Config(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC3_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_LPUART1_UART_Init(void);

uint16_t Read_Throttle_ADC(int);

// Pin Defintions
#define MODE PA_4
#define GEAR_F PC_0
#define GEAR_N PC_1
#define GEAR_R PC_2
#define TRIM_UP_STICKS PC_3
#define TRIM_DN_STICKS PC_4
#define TRIM_UP_WHEEL PC_5
#define TRIM_DN_WHEEL PC_6
#define ACS_UP PC_7
#define ACS_DN PC_8
#define CAM PC_9
#define LIGHTS PC_10
#define TRIM_UP_CTRL PC_11
#define TRIM_DN_CTRL PC_12

// For printf:
// #ifdef __GNUC__
// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
// #else
// #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
// #endif

// PUTCHAR_PROTOTYPE
// {
//   HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//   return ch;
// }


// CAN definitions
using namespace umeb::can;

can_settings_arduino can_cfg = {
  .settings = {
    .BitRate = 500000,
    .FDRate = 1000000,
  },
  .canModule = &fdcan1,
  .mode = ACANFD_STM32_Settings::ModuleMode::NORMAL_FD,
};
CAN_Bus_Arduino bus(can_cfg);

CAN_Message<uint8_t> CCUreadingyay;
CAN_Message<uint16_t> throttleSignal;
CAN_Packet<2> ccu_message(0x17, &CCUreadingyay, &throttleSignal);

//read gear from gear thing
//casseia is spitting some BULL about gear uhh idk what im doing but ok
//gear prob has to be a binary input, assumed using fwee pins
//heads up theres nasty if elses in here and idk if this is the right procedure but you get code
uint8_t Read_Gear(void){
  //gear value will be 00 or 01 or 10 or 11. otherwise it's not valid idk how to handle that
  //these are the pins we are gettin the state from i think
  uint8_t gear_pin_1 = !digitalReadFast(GEAR_N);
  uint8_t gear_pin_2 = !digitalReadFast(GEAR_F);
  uint8_t gear_pin_3 = !digitalReadFast(GEAR_R);
  //lowkey used gpt for this because idk how to combine using binary . shoutout gpt
  //this basically turns the input into like binary ? it just mashes them together
  //also this code is mad gross sorry
  if(gear_pin_1==1 &&gear_pin_2==0&&gear_pin_3==0)
  {
    //assuming neutral
    return 0b00;
  }
  else if(gear_pin_2==1 &&gear_pin_1==0&&gear_pin_3==0)
  {
    //assuming forward
    return 0b01;
  }
  else if(gear_pin_3==1 &&gear_pin_1==0&&gear_pin_2==0)
  {
    //assuming reverse
    return 0b10;
  }
  //can this live outside an else statement ? idk. lol
  else
  {
    //wtf is going on condition
    return 0b11;
  }
  //idk if i need to add another return to handle a situation where no input
}

// Main
void setup() {
  // put your setup code here, to run once:
  SystemClock_Config();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC3_Init();
  // MX_FDCAN1_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  // MX_LPUART1_UART_Init();

  bus.Init();
  bus.makePacketPeriodic(&ccu_message, 500u);

  analogReadResolution(12);

  // Initialize pins
  pin_function(MODE, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));
  pin_function(GEAR_R, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));
  pin_function(GEAR_F, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));
  pin_function(GEAR_N, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));
  pin_function(TRIM_UP_STICKS, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));
  pin_function(TRIM_DN_STICKS, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));
  pin_function(TRIM_UP_WHEEL, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));
  pin_function(TRIM_DN_WHEEL, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));
  pin_function(ACS_UP, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));
  pin_function(ACS_DN, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));

  pin_function(CAM, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
  //pin_function(LIGHTS, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
  pin_function(TRIM_UP_CTRL, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
  pin_function(TRIM_DN_CTRL, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));

  digitalWriteFast(CAM, LOW);
  //digitalWriteFast(LIGHTS, LOW);
  digitalWriteFast(TRIM_UP_CTRL, LOW);
  digitalWriteFast(TRIM_DN_CTRL, LOW);

  delay(1000);
}


void loop() {
  // bus.sendPacketUnsized(&ccu_message);
  //throttle land
  // --------- THROTTLE HANDLING -----------

  // Custom read throttle function due to lack of support for internal opamp connections
  uint16_t throttle1 = Read_Throttle_ADC(1);
  uint16_t throttle2 = Read_Throttle_ADC(2);

  printf("Read values: %d %d\n\r", (uint32_t) throttle1, (uint32_t) throttle2);

  // Going from 12-bit to 12-bit, so no bit shift error

  // ---- ADC to DAC pass through ----
  // This scaling works in theory, but in practice leaves a bit of room for error.
  // Consider adding configuration for each throttle based on its real bounds.

  // Map from 0.5-4.5v (thru voltage divider) to 0-2.5v
  double out = (double) throttle1 / 4096. * 3.3; // Input voltage
  out /= (4.3 / (4.3 + 3.3)); // Voltage divider -- Real throttle voltage
  out = std::clamp<double>(out, 0.5, 4.5);
  out = (out - 0.5) / 4.5;
  out *= 2.4; // Max input voltage for dauphine is 2.5v, slight safety factor
  out *= 4096. / 3.3;

  uint32_t raw_out = (uint32_t) out;

  // analogWrite or dac_write_value() cannot be used due to internal op amp buffering
  if (HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, raw_out) != HAL_OK) {
    /* Setting value Error */
    Error_Handler();
  }
  if (HAL_DAC_Start(&hdac3, DAC_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }


  digitalWrite(CAM, LOW);

  //setting the guys to be read thanks
  uint8_t modeSignal = !digitalReadFast(MODE);
  uint8_t gear = Read_Gear();
  uint8_t acs_up = !digitalReadFast(ACS_UP);
  uint8_t acs_down = !digitalReadFast(ACS_DN);
  int up, down;
  //if else for which throttle to use and also trim (pretty code)
  if (modeSignal == 0) {
    throttleSignal = Read_Throttle_ADC(1);
    up = !digitalReadFast(TRIM_UP_WHEEL);
    down = !digitalReadFast(TRIM_DN_WHEEL);
  }
  else if (modeSignal == 1) {
    throttleSignal = Read_Throttle_ADC(2);
    up = !digitalReadFast(TRIM_UP_STICKS);
    down = !digitalReadFast(TRIM_DN_STICKS);
  }
  if (up && down) {
    up = 0; down = 0;
    digitalWriteFast(TRIM_UP_CTRL,up);
    digitalWriteFast(TRIM_DN_CTRL,down);
  }

  CCUreadingyay = gear|(acs_up<<2)|(acs_down<<3)|(modeSignal<<4);

  // Tick CANBus
  bus.tick();
  delay(10);
}

// Read a throttle opamp channel
// Throttle 1 reads from OPAMP 2 (adc2 ch16)
// Throttle 2 reads from OPAMP 3 (adc2 ch18)
uint16_t Read_Throttle_ADC(int throttle) {
  if (throttle != 1 && throttle != 2)
    return 0;
  
  uint16_t th;

  ADC_ChannelConfTypeDef  channel_conf = {};
  channel_conf.Channel = (throttle == 1) ? ADC_CHANNEL_VOPAMP2 : ADC_CHANNEL_VOPAMP3_ADC2;
  channel_conf.Rank = ADC_REGULAR_RANK_1;
  channel_conf.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  channel_conf.SingleDiff = ADC_SINGLE_ENDED;
  channel_conf.OffsetNumber = ADC_OFFSET_NONE;
  channel_conf.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &channel_conf) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start(&hadc2) != HAL_OK) {
    /* Start Conversion Error */
    Error_Handler();
  }
  if (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK) {
    /* End Of Conversion flag not set on time */
    Error_Handler();
  }
  /* Check if the continuous conversion of regular channel is finished */
  if ((HAL_ADC_GetState(&hadc2) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
    /*##-5- Get the converted value of regular channel  ########################*/
    th = HAL_ADC_GetValue(&hadc2);
  }
  if (HAL_ADC_Stop(&hadc2) != HAL_OK) {
    /* Stop Conversation Error */
    Error_Handler();
  }
  return th;
}


// ------ GENERATED BY STM32CUBEIDE --------

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

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
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */
  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_OPAMP_Lock(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = ENABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */
  if (HAL_OPAMP_Start(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_OPAMP_Lock(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */
  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_OPAMP_Lock(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END OPAMP3_Init 2 */

}



/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}