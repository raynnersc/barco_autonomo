/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "L293D_STM32F103.h"
#include "API_STM32F103_SERVOMOTOR_MG90D.h"
//#include "HMC5883L.h"
#include "QMC5883.h"
#include "I2Cdev.h"
#include "Sail_Algorithms.h"
#include "Bluetooth_BLE_V4.2_JDY-18.h"
#include <string.h>
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Beacon b1;
Beacon b2;
Beacon b3;
float boat_x;
float boat_y;
QMC_t compass;
Compass_config config;
float boat_angle;
float target_angle;
float rudder_angle;
uint8_t duty_cylce_dc_motor;

//Variáveis para o módulo bluetooth
extern Status_t status;
static char buffer[700];
uint8_t caractere[1];
static uint8_t buffer_index = 0;
ListDevices_t listModules;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void GetBeaconBTData(Beacon *b1, Beacon *b2, Beacon *b3, ListDevices_t listModules);
void GetCompassData();
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  //Configura o módulo Bluetooth
  status = INSTRUCTION;
  //memset(buffer,0,strlen(buffer));
  //buffer_index = 0;
  setupBLE(&huart1, &huart2);
  while(status != FINISH);

  status = INSTRUCTION;
  //memset(buffer,0,strlen(buffer));
  //buffer_index = 0;
  setBaud(BAUD_9600);
  while(status != FINISH);

  status = INSTRUCTION;
  //memset(buffer,0,strlen(buffer));
  //buffer_index = 0;
  setRole(MASTER);
  while(status != FINISH);

  b1.x = UTM_B1_X;
  b1.y = UTM_B1_Y;
  b1.d = 0;
  b2.x = UTM_B2_X;
  b2.y = UTM_B2_Y;
  b2.d = 0;
  b3.x = UTM_B3_X;
  b3.y = UTM_B3_Y ;
  b3.d = 0;

  compass.ADDR_Control_RegisterA = ADDR_REG_A;
  compass.ADDR_Control_RegisterB = ADDR_REG_B;
  compass.ADDR_Mode_Register = ADDR_REG_MODE;
  compass.ADDR_Status_Register = ADDR_REG_STATUS;

  config.gain = _4_0;
  config.meas_mode = Normal;
  config.op_mode = Continuous_meas;
  config.output_rate = 75;
  config.samples_num = eight;

  QMC_init(&compass,&hi2c2,&config);

  SetupServo(SERVO_PWM_GPIO_Port, SERVO_PWM_Pin);
  Setup_DC_Motor(&htim2, TIM_CHANNEL_2);

  SetDirection_DC_Motor(Forward);
  SetDutyCycle_DC_Motor(&htim2, TIM_CHANNEL_2, 100);

  //HAL_Delay(2000);
  //SetDirection_DC_Motor(Back);
  //HAL_Delay(1000);
  //SetDirection_DC_Motor(Forward);
  HAL_Delay(10000);

  //SetDirection_DC_Motor(Forward);
  //SetDutyCycle_DC_Motor(&htim2, TIM_CHANNEL_2, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  status = INSTRUCTION;
	  	  //memset(buffer,0,strlen(buffer));
	  	  //buffer_index = 0;
	  	  setMasterScanForSlaves();
		  while(status != FINISH);

	  	  status = SCAN;
//		  memset(buffer,0,strlen(buffer));
//		  buffer_index = 0;
		  while(status != FINISH);
		  listModules = masterScanForSlaves(buffer);
		  memset(buffer,0,strlen(buffer));

		  if(listModules.nbOfDevices != 0)
			  GetBeaconBTData(&b1, &b2, &b3, listModules);

	  	  b1.d = BeaconDistance(b1.power, b1.RSSI);
	  	  b2.d = BeaconDistance(b2.power, b2.RSSI);
	  	  b3.d = BeaconDistance(b3.power, b3.RSSI);

	  	  BoatPosition(&b1, &b2, &b3, &boat_x, &boat_y);

	  	  GetCompassData();

	  	  boat_angle = AngleFromGeoNorth(&compass);

	  	  target_angle = TargetAngleFromGeoNorth(&b2, boat_x, boat_y);

	  	  rudder_angle = RudderAngle(boat_angle, target_angle);

	  	  SetPosition(rudder_angle);

	  	  duty_cylce_dc_motor = DCMotorDutyCycle(boat_angle, target_angle);

	  	  SetDutyCycle_DC_Motor(&htim2, TIM_CHANNEL_2, 100);

	  	  HAL_Delay(100);
	  	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIR_LATCH_Pin|DIR_EN_Pin|DIR_SER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_CLK_GPIO_Port, DIR_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DIR_LATCH_Pin DIR_EN_Pin DIR_SER_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DIR_LATCH_Pin|DIR_EN_Pin|DIR_SER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PWM_SERVO_Pin */
  GPIO_InitStruct.Pin = PWM_SERVO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWM_SERVO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_CLK_Pin */
  GPIO_InitStruct.Pin = DIR_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_TIM3_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void GetBeaconBTData(Beacon *b1, Beacon *b2, Beacon *b3, ListDevices_t listModules){
	for(uint8_t i=0; i<listModules.nbOfDevices; i++){
		if(strstr(listModules.devices[i].name,"PSE2022_B1") != NULL){
			b1->RSSI = listModules.devices[i].signalStrength;
		}
		else if(strstr(listModules.devices[i].name,"PSE2022_B2") != NULL){
			b2->RSSI = listModules.devices[i].signalStrength;
		}
		else if(strstr(listModules.devices[i].name,"PSE2022_B3") != NULL){
			b3->RSSI = listModules.devices[i].signalStrength;
		}
	}
	b1->power = BEACON_POWER;
	b2->power = BEACON_POWER;
	b3->power = BEACON_POWER;
}

void GetCompassData(){
	//HMC5883L_measurement(&x_field, &y_field, &z_field);
	//QMC_init(&compass,&hi2c2,&config);
	QMC_read(&compass);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(status != FINISH){
		buffer[buffer_index] = (char) *caractere;
		buffer_index++;
		char *check = strstr(buffer, "OK");
		char *check2 = strstr(buffer, "STOP:SCAN");
		if(((check != NULL)&&(status == INSTRUCTION)) || ((check2 != NULL)&&(status == SCAN))){
			strcat(buffer,"\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t *) buffer, strlen(buffer), HAL_MAX_DELAY);
			if(status != SCAN){
				memset(buffer,0,strlen(buffer));
			}
			else{
				sendToLogger("Inquired \r\n");
			}
			status = FINISH;
			buffer_index = 0;
		}
		//free(check);
		//free(check2);
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
