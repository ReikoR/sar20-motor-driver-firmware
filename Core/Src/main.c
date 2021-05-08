/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "drv8323.h"
#include "foc.h"
#include "pid.h"
#include "windowed_average.h"
#include "eeprom_emul.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) DebugInfo {
  uint16_t rawPositionSensorValue;
  uint16_t mechAngleRaw;
  int16_t deltaPosition;
  uint16_t prevPosition;
  uint16_t position;
  uint16_t driverRegister1;
  uint16_t aDuty;
  uint16_t bDuty;
  uint16_t cDuty;
  uint16_t aVoltage;
  uint16_t bVoltage;
  uint16_t busVoltage;
  uint16_t aVoltageOffset;
  uint16_t bVoltageOffset;
  float speedHz;
  float aI;
  float bI;
  float cI;
  float dI;
  float qI;
  uint16_t nFault;
  uint16_t delimiter;
} DebugInfo;

typedef struct __attribute__((packed)) DebugCommand {
  float speedHz;
  uint8_t calibrateAngle;
  uint8_t checkSum;
} DebugCommand;


enum ControlState {
  Idle,
  CalibratingDriver,
  Ready,
  CalibratingAngle,
  Active
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Clear all flags */
  hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));

  /* Configure DMA Channel data length */
  hdma->Instance->CNDTR = DataLength;

  /* Memory to Peripheral */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Channel destination address */
    hdma->Instance->CPAR = DstAddress;

    /* Configure DMA Channel source address */
    hdma->Instance->CMAR = SrcAddress;
  }
  /* Peripheral to Memory */
  else
  {
    /* Configure DMA Channel source address */
    hdma->Instance->CPAR = SrcAddress;

    /* Configure DMA Channel destination address */
    hdma->Instance->CMAR = DstAddress;
  }
}

void DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Disable the peripheral */
  __HAL_DMA_DISABLE(hdma);

  /* Configure the source, destination address and the data length & clear flags*/
  DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);

  //__HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(hdma, (DMA_IT_TC | DMA_IT_TE));

  /* Enable the Peripheral */
  __HAL_DMA_ENABLE(hdma);
}

void SPI_TransmitReceive_DMA_Setup(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
  /* Set the transaction information */
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
  hspi->TxXferSize  = 1;
  hspi->TxXferCount = Size;
  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
  hspi->RxXferSize  = 1;
  hspi->RxXferCount = Size;

  /* Init field not used in handle to zero */
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;

  DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)hspi->pRxBuffPtr, hspi->RxXferCount);

  /* Enable Rx DMA Request */
  SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);
}

int limit(int value, int min, int max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

enum ControlState controlState = Idle;

uint16_t positionSensorTxData[1] = {0x0000};
uint16_t positionSensorRxData[1];

uint8_t uartRxDataDMA[6] = {0, 0, 0, 0, 0, 0};
uint8_t uartRxData[6] = {0, 0, 0, 0, 0, 0};

uint16_t zeroOffset = 6800;
const int polePairs = 7;
//const int mechCounts = 65536;

uint16_t mechAngleRaw = 0;
const float fElecCounts = 9362.2857f; // 2^16 / 7
const uint16_t elecCounts = 9362;
volatile float fElecAngle = 0.0f;
float busV = 12.0f;

//adcRefV * voltageDividerRatio / adcCounts; busV = adcValue / 4095 * 3.3 * (24.9 + 4.99) / 4.99;
const float ratioAdcToBusV = 0.004835f;

uint16_t adcData[3] = {0, 0, 0};
uint16_t adcOffsets[2] = {1965, 1965};

int isMeasuringCurrentOffsets = 0;
const uint32_t maxAdcOffsetSum = 80000000; // should take about 2s at 20kHz
uint32_t adcOffsetSums[3] = {0, 0, 0};
uint32_t adcOffsetCounts[3] = {0, 0, 0};

// adcRefV / (shuntR * opampGain * adcCounts), gain = 20, shunt = 0.1
const float ratioAdcToPhaseI = 4.0293e-4f;

SinCosResult sc = {.c = 0.0f, .s = 0.0f};

volatile float aV = 0.0f;
volatile float bV = 0.0f;
volatile float cV = 0.0f;

float aI = 0.0f;
float bI = 0.0f;
float cI = 0.0f;

float iqRef = 0.0f;
float idRef = 0.0f;

volatile float angleSetpoint = 0.0f;

volatile float voltage = 0.0f;

SinCosResult scTest;

const float velocity_scale = 1.0f / 65536.0f;
const float kRateHz = 20000.0f;

WindowedAverage speedWindowedAverage;
volatile float speed_Hz = 0.0f;
volatile float speedRef = 0.0f;

uint16_t position = 0;
uint16_t prevPosition = 0;
int16_t deltaPosition = 0;

uint16_t aDuty = 0;
uint16_t bDuty = 0;
uint16_t cDuty = 0;

DqTransformResult dqI = {.d = 0.0f, .q = 0.0f};
InverseDqTransformResult abc = {.a = 0.0f, .b = 0.0f, .c = 0.0f};

PI_Control iqPI = {.pGain = 2.0f, .iGain = 0.2f, .dGain = 0.0f, .maxI = 5.0f, .i = 0.0f};
PI_Control idPI = {.pGain = 2.0f, .iGain = 0.2f, .dGain = 0.0f, .maxI = 5.0f, .i = 0.0f};
PI_Control speedPI = {.pGain = 0.08f, .iGain = 0.00004f, .dGain = 0.0f, .maxI = 5.0f, .i = 0.0f};

float iqPIout = 0.0f;
float idPIout = 0.0f;

int isAngleSensorActive = 0;
int isCalibratingAngle = 0;
float angleCalibrationCurrent = 0.5f;
volatile float angleCalibrationSetpoint = 0.0f;
SinCosResult scAngleCalibration = {.c = 0.0f, .s = 0.0f};
float angleCalibrationDirection = 1.0f;
int angleAtZeroCounter = 0;

volatile DebugInfo debugInfo = {
    .rawPositionSensorValue = 0,
    .mechAngleRaw = 0,
    .driverRegister1 = 0,
    .aDuty = 0,
    .bDuty = 0,
    .cDuty = 0,
    .aVoltage = 0,
    .bVoltage = 0,
    .busVoltage = 0,
    .aVoltageOffset = 0,
    .bVoltageOffset = 0,
    .deltaPosition = 0,
    .delimiter = 0xAAAA
};

volatile DebugCommand debugCommand = {
    .speedHz = 0.0f,
    .calibrateAngle = 0,
    .checkSum = 0
};

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc != &hadc1) {
    return;
  }

  DEBUG1_GPIO_Port->BSRR = DEBUG1_Pin;

  if (isMeasuringCurrentOffsets) {
    adcOffsetSums[0] += adcData[0];
    adcOffsetSums[1] += adcData[2];
    adcOffsetCounts[0] += 1;
    adcOffsetCounts[1] += 1;

    if (adcOffsetSums[0] > maxAdcOffsetSum || adcOffsetSums[1] > maxAdcOffsetSum) {
      isMeasuringCurrentOffsets = 0;

      adcOffsets[0] = adcOffsetSums[0] / adcOffsetCounts[0];
      adcOffsets[1] = adcOffsetSums[1] / adcOffsetCounts[1];
    }

    return;
  }

  if (controlState < Ready) {
    return;
  }

  mechAngleRaw = positionSensorRxData[0];
  prevPosition = position;
  position = mechAngleRaw - zeroOffset;
  deltaPosition = position - prevPosition;
  WindowedAverage_Add(&speedWindowedAverage, deltaPosition);
  speed_Hz = 0.96f * speed_Hz + 0.04f * (float)speedWindowedAverage.total_ * velocity_scale * kRateHz / (float)speedWindowedAverage.size_;

  if (controlState < CalibratingAngle) {
    return;
  }

  fElecAngle = (float)(position % elecCounts);
  fElecAngle = fElecAngle / fElecCounts * k2Pi;

  if (fElecAngle < 0.0f) fElecAngle += k2Pi;

  SinCos(fElecAngle, &sc);

  //SinCos(angleSetpoint, &scTest);
  //InverseDqTransform(&scTest, 3.8f, 0.0f, &abc);

  //angleSetpoint += 0.01f;

  //if (angleSetpoint > k2Pi) angleSetpoint -= k2Pi;

  aI = (float)((int32_t)adcData[0] - (int32_t)adcOffsets[0]) * ratioAdcToPhaseI;
  bI = (float)((int32_t)adcData[2] - (int32_t)adcOffsets[1]) * ratioAdcToPhaseI;
  cI = -aI - bI;

  busV = (float)adcData[1] * ratioAdcToBusV;

  //InverseDqTransform(&sc, 0.0f, 0.8f, &abc); // voltage

  DqTransform(&sc, aI, bI, cI, &dqI); // current

  //iqRef = 0.2f;
  if (controlState == CalibratingAngle) {
    idRef = angleCalibrationCurrent;
    iqRef = 0.0f;

    angleCalibrationSetpoint += angleCalibrationDirection * 0.01f;

    if (mechAngleRaw > elecCounts) {
      angleCalibrationDirection = -1.0f;
      angleAtZeroCounter = 0;
    } else {
      if (angleCalibrationSetpoint > 0.1f) {
        angleCalibrationDirection = -1.0f;
      } else {
        angleCalibrationSetpoint = 0.0f;
      }
    }

    if (mechAngleRaw < elecCounts && angleCalibrationSetpoint == 0.0f) {
      angleAtZeroCounter++;

      if (angleAtZeroCounter == 10000) {
        zeroOffset = mechAngleRaw;
        isCalibratingAngle = 0;
      }
    }

    if (angleCalibrationSetpoint > k2Pi) {
      angleCalibrationSetpoint -= k2Pi;
    } else if (angleCalibrationSetpoint < 0) {
      angleCalibrationSetpoint += k2Pi;
    }

    SinCos(angleCalibrationSetpoint, &scAngleCalibration);
    DqTransform(&scAngleCalibration, aI, bI, cI, &dqI);
  } else if (controlState == Active) {
    DqTransform(&sc, aI, bI, cI, &dqI); // current
    idRef = 0.0f;
    iqRef = PI_Control_update(&speedPI, speedRef, speed_Hz);
  }

  iqPIout = PI_Control_update(&iqPI, iqRef, dqI.q);
  idPIout = PI_Control_update(&idPI, idRef, dqI.d);

  if (controlState == CalibratingAngle) {
    InverseDqTransform(&scAngleCalibration, idPIout, iqPIout, &abc);
  } else if (controlState == Active) {
    InverseDqTransform(&sc, idPIout, iqPIout, &abc); // current
  }

  // Voltage to PWM
  aV = 0.5f + abc.a / busV;
  bV = 0.5f + abc.b / busV;
  cV = 0.5f + abc.c / busV;

  aDuty = limit((int)(aV * 4000.0f), 0, 4000);
  bDuty = limit((int)(bV * 4000.0f), 0, 4000);
  cDuty = limit((int)(cV * 4000.0f), 0, 4000);

  htim1.Instance->CCR1 = cDuty;
  htim1.Instance->CCR2 = bDuty;
  htim1.Instance->CCR3 = aDuty;

  // For finding angle sensor zero offset
  /*TIM1->CCR1 = 1800;
  TIM1->CCR2 = 1800;
  TIM1->CCR3 = 2400;*/

  /*TIM1->CCR1 = 2000;
  TIM1->CCR2 = 2000;
  TIM1->CCR3 = 2000;*/

  /*IM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;*/

  DEBUG1_GPIO_Port->BRR = DEBUG1_Pin;
}

void startAngleSensor() {
  if (isAngleSensorActive) {
    return;
  }

  isAngleSensorActive = 1;

  /*DRV8323_writeRegister(
      &hspi3,
      DRV_CS_GPIO_Port,
      DRV_CS_Pin,
      DRV8323_RegisterAddress_CSA_control,
      DRV8323_CSA_control_bidirectional_mode |
      DRV8323_CSA_control_gain_20 |
      DRV8323_CSA_control_sense_overcurrent_level_1V
  );*/

  // Change SPI frequency to 10 MHz for angle sensor
  MODIFY_REG(hspi3.Instance->CR1, SPI_CR1_BR_Msk, SPI_BAUDRATEPRESCALER_16);

  // Change SPI mode from 1 to 0
  MODIFY_REG(hspi3.Instance->CR1, SPI_CR1_CPHA_Msk, SPI_PHASE_1EDGE);

  SPI_TransmitReceive_DMA_Setup(&hspi3, (uint32_t)positionSensorTxData, (uint32_t)positionSensorRxData, 1);
  HAL_DMA_Start(&hdma_tim4_ch2, positionSensorTxData, (uint32_t)&SPI3->DR, 1);
}

void setControlState(enum ControlState newControlState) {
  if (controlState == newControlState) {
    return;
  }

  switch (newControlState) {
  case Idle:
    break;
  case CalibratingDriver:
    /*DRV8323_writeRegister(
        &hspi3,
        DRV_CS_GPIO_Port,
        DRV_CS_Pin,
        DRV8323_RegisterAddress_CSA_control,
        DRV8323_CSA_control_bidirectional_mode |
        DRV8323_CSA_control_gain_20 |
        DRV8323_CSA_control_CAL_A_enabled |
        DRV8323_CSA_control_CAL_B_enabled |
        DRV8323_CSA_control_CAL_C_enabled |
        DRV8323_CSA_control_sense_overcurrent_level_1V
    );*/

    isMeasuringCurrentOffsets = 1;

    break;
  case Ready:
    startAngleSensor();
    break;
  case CalibratingAngle:
    isCalibratingAngle = 1;
    startAngleSensor();
    break;
  case Active:
    startAngleSensor();
    break;
  }

  controlState = newControlState;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  WindowedAverage_Init(&speedWindowedAverage);
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
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  __HAL_SPI_ENABLE(&hspi3);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);

  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET); // enable DRV8323

  HAL_Delay(1); // SPI ready after enable < 1ms

  DRV8323_writeRegister(
      &hspi3,
      DRV_CS_GPIO_Port,
      DRV_CS_Pin,
      DRV8323_RegisterAddress_driver_control,
      DRV8323_driver_control_PWM_mode_3x
  );

  /*uint16_t driverControlRegisterValue = DRV8323_readRegister(
      &hspi3,
      DRV_CS_GPIO_Port,
      DRV_CS_Pin,
      DRV8323_RegisterAddress_driver_control
  );*/
  EE_Status ee_status = EE_OK;

  HAL_FLASH_Unlock();

  ee_status = EE_Init(EE_FORCED_ERASE);
  if(ee_status != EE_OK) {Error_Handler();}

  EE_ReadVariable16bits(1, &zeroOffset);

  HAL_FLASH_Lock();

  htim4.Instance->CCR1 = 32;
  htim4.Instance->CCR2 = 64;

  HAL_ADC_Start_DMA(&hadc2, adcData + 2, 1);

  HAL_ADC_Start_DMA(&hadc1, adcData, 2);
  HAL_ADC_Start_IT(&hadc1);

  HAL_UART_Receive_DMA(&huart1, &uartRxDataDMA, sizeof(uartRxDataDMA));

  // sets offset before timer is started to align TIM4 CH2 DMA request that triggers SPI3 data transfer,
  // should happen after the ADC sampling is done and before ADC sequence end interrupt
  htim4.Instance->CNT = 4000;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 3999;
  TIM1->CCR5 = 1;

  __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);

  setControlState(CalibratingDriver);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(100);

    if (controlState == CalibratingDriver && isMeasuringCurrentOffsets == 0) {
      // Current offset calibration has ended
      setControlState(Ready);
    } else if (controlState == CalibratingAngle && isCalibratingAngle == 0) {
      // Angle calibration has ended
      HAL_FLASH_Unlock();
      EE_WriteVariable16bits(1, zeroOffset);
      HAL_FLASH_Lock();
      setControlState(Ready);
    }

    uint32_t uartRxDMACounter = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    uint32_t firstByteIndex = sizeof(uartRxData) - uartRxDMACounter;
    uint8_t calculatedCheckSum = 0;

    for (uint32_t i = 0; i < sizeof(uartRxData); i++) {
      uint32_t byteIndex = firstByteIndex + i;

      if (byteIndex >= sizeof(uartRxData)) {
        byteIndex -= sizeof(uartRxData);
      }

      uartRxData[i] = uartRxDataDMA[byteIndex];

      if (i != sizeof(uartRxData) - 1) {
        calculatedCheckSum += uartRxData[i];
      }
    }

    if (calculatedCheckSum == uartRxData[sizeof(uartRxData) - 1]) {
      uint8_t prevCalibrateAngle = debugCommand.calibrateAngle;

      memcpy(&debugCommand, uartRxData, sizeof(uartRxData));

      speedRef = debugCommand.speedHz;

      if (prevCalibrateAngle == 0 && debugCommand.calibrateAngle == 1) {
        setControlState(CalibratingAngle);
      } else if (controlState == Ready && speedRef != 0.0f) {
        setControlState(Active);
      }
    }

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    debugInfo.driverRegister1 = zeroOffset;
    debugInfo.rawPositionSensorValue = positionSensorRxData[0];
    debugInfo.mechAngleRaw = mechAngleRaw;
    debugInfo.deltaPosition = deltaPosition;
    debugInfo.prevPosition = prevPosition;
    debugInfo.position = position;
    debugInfo.aDuty = aDuty;
    debugInfo.bDuty = bDuty;
    debugInfo.cDuty = cDuty;
    debugInfo.aVoltage = adcData[0];
    debugInfo.bVoltage = adcData[2];
    debugInfo.busVoltage = adcData[1];
    debugInfo.aVoltageOffset = adcOffsets[0];
    debugInfo.bVoltageOffset = adcOffsets[1];
    debugInfo.speedHz = speed_Hz;
    debugInfo.aI = aI;
    debugInfo.bI = bI;
    debugInfo.cI = cI;
    debugInfo.dI = dqI.d;
    debugInfo.qI = dqI.q;
    debugInfo.nFault = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);

    HAL_UART_Transmit(&huart1, &debugInfo, sizeof(debugInfo), 0xffffff);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
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
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim1.Init.Period = 4000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC5REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 3999;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 7999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 4;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 31999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|DEBUG1_Pin|DEBUG2_Pin|DEBUG3_Pin
                          |DEBUG4_Pin|DRV_EN_Pin|DRV_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin DEBUG1_Pin DEBUG2_Pin DEBUG3_Pin
                           DEBUG4_Pin DRV_EN_Pin DRV_CS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|DEBUG1_Pin|DEBUG2_Pin|DEBUG3_Pin
                          |DEBUG4_Pin|DRV_EN_Pin|DRV_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : nFAULT_Pin */
  GPIO_InitStruct.Pin = nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(nFAULT_GPIO_Port, &GPIO_InitStruct);

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
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(40);
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
