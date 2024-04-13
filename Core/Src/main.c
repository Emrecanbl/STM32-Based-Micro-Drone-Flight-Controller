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
#include "nRF24L01P.h"
#include "dwt_delay.h"
#include "MPU_6050.h"

void NRFSendData(char * nrfID, char * nrfData);
void NRF_main_init(void);

SensorData_t SensorData;
uint32_t adc_values[4];
float AngleX,AngleY,AngleZ;
nRF24L01P myNRF;
char RXBuffer[32];
char TXBuffer[32];
char regStatus;
float Sn = 0.0030;
uint32_t T ;
uint8_t Adc_Val1[24];
uint8_t Adc_Val[8];

float DR_Roll,DR_Pitch,DR_Yaw;
float DR_Roll_Angle,DR_Pitch_Angle;
float ER_Roll,ER_Pitch,ER_Yaw;
float ER_Roll_Angle,ER_Pitch_Angle;
float IN_Roll,IN_Pitch,IN_Yaw;
float PE_ER_Roll,PE_ER_Pitch,PE_ER_Yaw;
float PE_ER_Roll_Angle,PE_ER_Pitch_Angle;
float PIR_Roll,PIR_Pitch,PIR_Yaw;
float PIR_Roll_Angle,PIR_Pitch_Angle;
float PID_return[]={0,0,0};

float PRateRoll=0;float PRatePitch=0;float PRateYaw=0;
float IRateRoll=0;float IRatePitch=0;float IRateYaw=0;
float DRateRoll=0;float DRatePitch=0;float DRateYaw=0;

float P_Angle_Roll=0;float P_Angle_Pitch=0;
float I_Angle_Roll=0;float I_Angle_Pitch=0;
float D_Angle_Roll=0;float D_Angle_Pitch=0;
/*float PRateRoll=0.4;float PRatePitch=0.4;float PRateYaw=0;
float IRateRoll=0;float IRatePitch=0;float IRateYaw=0;
float DRateRoll=0.0003;float DRatePitch=0.0003;float DRateYaw=0;

float P_Angle_Roll=4;float P_Angle_Pitch=4;
float I_Angle_Roll=0.5;float I_Angle_Pitch=0.5;
float D_Angle_Roll=6;float D_Angle_Pitch=6;
*/
float K_Angle_Roll =0;
float KU_Angle_Roll =4;
float K_Angle_Pitch =0;
float KU_Angle_Pitch =4;
float Kalman_Output[]={0,0};
float Kalman_Angles[]={0,0};

uint8_t M1,M2,M3,M4;
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Motor_Inıt();
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_Delay(100);
  DWT_Init();
  MPU_6050_Init();
  HAL_Delay(1000);
  NRF_main_init();
  MPU_6050_Gyroscope_Cali(&SensorData);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
  __HAL_TIM_SET_COUNTER(&htim2,0);
  NRFSendData("00001", "OK!");
	 if((myNRF.RXIRQ==1) && (!myNRF.Busy))
	      {
	      myNRF.RXIRQ==0;
	      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      }
	 for (int i = 0;i<24;i++)
	  {
	     	Adc_Val1[i] = myNRF.RX_Buffer[i];
	     	HAL_Delay(1);

	    }
	 for (int j = 1;j<9;j++){
		 Adc_Val[j-1] = (Adc_Val1[j*3-3]-48)*100 + (Adc_Val1[j*3-2]-48)*10  + Adc_Val1[j]-48;
		 HAL_Delay(1);
	 }
	 HAL_Delay(1);
	 Get_IMU_Data();
	 Kalman_Angles_Cal(&SensorData);
	 Control_Motor(Adc_Val[2],Adc_Val[7],Adc_Val[4],0,0,Adc_Val[0],&SensorData);
	   T = __HAL_TIM_GET_COUNTER(&htim2);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 256;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Csn_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Csn_Pin CE_Pin */
  GPIO_InitStruct.Pin = Csn_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_15)
	{
		if(HAL_nRF24L01P_IRQ_Handler(&myNRF) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_nRF24L01P_ReadRegister(&myNRF, nRF_STATUS, &regStatus);
	}
}
void NRFSendData(char * nrfID, char * nrfData)
{
	HAL_nRF24L01P_SetPTXAddress(&myNRF, (uint8_t *) nrfID);
	if(HAL_nRF24L01P_TransmitPacketACK(&myNRF, (uint8_t *) nrfData, nRF_DATA_PIPE_0) != HAL_OK)
	{
		//Error
	}
}

void NRF_main_init(void)
{
	/* ---- myNRF24L01+ Definitions ---- */
	myNRF.hspi = &hspi1;
	myNRF.CRC_Width = nRF_CRC_WIDTH_BYTE;
	myNRF.ADDR_Width = nRF_ADDR_WIDTH_5;
	myNRF.Data_Rate = nRF_DATA_RATE_2MBPS;
	myNRF.TX_Power = nRF_TX_PWR_M18dBm;
	myNRF.State = nRF_STATE_RX;

	myNRF.RF_Channel = 1;
	myNRF.PayloadWidth = nRF_RXPW_32BYTES;
	myNRF.RetransmitCount = nRF_RETX_DISABLED;// nRF_RETX_COUNT_15;
	myNRF.RetransmitDelay = nRF_RETX_DELAY_250uS; //nRF_RETX_DELAY_4000uS;

	myNRF.RX_Address = (uint8_t *)"00000";
	myNRF.TX_Address = (uint8_t *)"00001";

	myNRF.RX_Buffer = RXBuffer;
	myNRF.TX_Buffer = TXBuffer;

	myNRF.nRF_nSS_GPIO_PORT = GPIOA;
	myNRF.nRF_nSS_GPIO_PIN = GPIO_PIN_3;

	myNRF.nRF_CE_GPIO_PORT = GPIOA;
	myNRF.nRF_CE_GPIO_PIN = GPIO_PIN_4;

	//ekleme
	myNRF.RXIRQ = 0;
	myNRF.TXIRQ = 0;
	myNRF.MaxReIRQ = 0;
	/* ---- myNRF24L01+ Definitions ---- */


	if(HAL_nRF24L01P_Init(&myNRF) != HAL_OK)
	{
		Error_Handler();
	}
}
void Motor_control(uint8_t m1,uint8_t m2,uint8_t m3,uint8_t m4){
	 TIM1->CCR1 = m2;
	 TIM1->CCR2 = m3;
	 TIM1->CCR3 = m1;
	 TIM1->CCR4 = m4;
}

void Motor_Inıt(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}
void Get_IMU_Data(){
	MPU_6050_Accelerometer_Read(&SensorData);
	MPU_6050_Angle(&SensorData);
	MPU_6050_Gyroscope_Read(&SensorData);
	 MPU_6050_Temp_Read();
}

void PID_equation(float Error,float P,float I,float D,float PrevError,float PrevItern){
	float Pterm= P*Error;
	float Iterm = PrevItern+I*(Error+PrevError)*Sn/2;
	if(Iterm>50){Iterm=50;}
	else if(Iterm<=-50){Iterm=-50;}
	float Dterm=D*(Error-PrevError)/Sn;
	float PIDout=Pterm+Iterm+Dterm;
	if(PIDout>50){PIDout=50;}
	else if(PIDout<=-50){PIDout=-50;}
	PID_return[0]=PIDout;
	PID_return[1]=Error;
	PID_return[2]=Iterm;
}
void PID_Reset(){
	 PE_ER_Roll=0,PE_ER_Pitch=0,PE_ER_Yaw=0;
	 PIR_Roll=0,PIR_Pitch=0,PIR_Yaw=0;
	 PE_ER_Roll_Angle=0,PE_ER_Pitch_Angle=0;
	 PIR_Roll_Angle=0,PIR_Pitch_Angle=0;
}

void Control_Motor(uint8_t Trottle,uint8_t Roll,uint8_t Pitch,uint8_t Roll_Angle,uint8_t Pitch_Angle,uint8_t Yaw,SensorData_t *SensorData){
	ER_Roll_Angle=-Roll_Angle-K_Angle_Roll;
	ER_Pitch_Angle=-Pitch_Angle-K_Angle_Pitch;
	PID_equation(ER_Roll_Angle,P_Angle_Roll,I_Angle_Roll,D_Angle_Roll,PE_ER_Roll_Angle,PIR_Roll_Angle);
	Roll=PID_return[0];
	PE_ER_Roll_Angle=PID_return[1];
	PIR_Roll_Angle=PID_return[2];
	PID_equation(ER_Pitch_Angle,P_Angle_Pitch,I_Angle_Pitch,D_Angle_Pitch,PE_ER_Pitch_Angle,PIR_Pitch_Angle);
	Pitch=PID_return[0];
	PE_ER_Pitch_Angle=PID_return[1];
	PIR_Pitch_Angle=PID_return[2];
	ER_Roll=Roll-(-SensorData->GYRO_XOUT);
	ER_Pitch=Pitch-(-SensorData->GYRO_YOUT);
	ER_Yaw=Yaw-(-SensorData->GYRO_ZOUT);
	PID_equation(ER_Roll,PRateRoll,IRateRoll,DRateRoll,PE_ER_Roll,PIR_Roll);
	IN_Roll=PID_return[0];
	PE_ER_Roll=PID_return[1];
	PIR_Roll=PID_return[2];
	PID_equation(ER_Pitch,PRatePitch,IRatePitch,DRatePitch,PE_ER_Pitch,PIR_Pitch);
	IN_Pitch=PID_return[0];
	PE_ER_Pitch=PID_return[1];
	PIR_Pitch=PID_return[2];
	PID_equation(ER_Yaw,PRateYaw,IRateYaw,DRateYaw,PE_ER_Yaw,PIR_Yaw);
	IN_Yaw=PID_return[0];
	PE_ER_Yaw=PID_return[1];
	PIR_Yaw=PID_return[2];

	if(Trottle>=255){Trottle=255;}

	M1=Trottle-IN_Roll-IN_Pitch-IN_Yaw;
	M2=Trottle-IN_Roll+IN_Pitch+IN_Yaw;
	M3=Trottle+IN_Roll+IN_Pitch-IN_Yaw;
	M4=Trottle+IN_Roll-IN_Pitch+IN_Yaw;

	Motor_control(M1,M2,M3,M4);
}

void Kalman_filter(float Kalman_state,float Kalman_Uncertanity,float Kalman_Input,float Kalman_Mesurement){
	Kalman_state=Kalman_state+Sn*Kalman_Input;
	Kalman_Uncertanity=Kalman_Uncertanity+Sn*Sn*4*4;
	float Kalman_Gain= Kalman_Uncertanity*1/(1*Kalman_Uncertanity+3*3);
	Kalman_state=Kalman_state+Kalman_Gain*(Kalman_Mesurement-Kalman_state);
	Kalman_Uncertanity=(1-Kalman_Gain)*Kalman_Uncertanity;
	Kalman_Output[0]=Kalman_state;
	Kalman_Output[1]=Kalman_Uncertanity;
}

void Kalman_Angles_Cal(SensorData_t *SensorData){
	Kalman_filter( K_Angle_Roll, KU_Angle_Roll,SensorData->GYRO_YOUT,SensorData->Angle_Y);
	K_Angle_Roll=Kalman_Output[0];
	KU_Angle_Roll=Kalman_Output[1];
	Kalman_filter( K_Angle_Pitch, KU_Angle_Pitch,SensorData->GYRO_XOUT,SensorData->Angle_X);
	K_Angle_Pitch=Kalman_Output[0];
	KU_Angle_Pitch=Kalman_Output[1];
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
