/*
 * L293D_STM32F103.c
 *
 *  Created on: Dec 13, 2022
 *      Author: danil
 */

#include "L293D_STM32F103.h"
#include "main.h"

TIM_OC_InitTypeDef sConfigOC_DC_Motor = {0};

void Setup_DC_Motor(TIM_HandleTypeDef *htimx, uint32_t Channel){
	HAL_TIM_PWM_Start(htimx, Channel);
	SetDirection_DC_Motor(Forward);
	SetDutyCycle_DC_Motor(htimx, Channel, 0);
}

void SetDirection_DC_Motor(Direction dir_code){

  GPIO_PinState bit_to_send = GPIO_PIN_RESET;

  //Enviar código de controle da direção:
  HAL_GPIO_WritePin(DIR_EN_GPIO_Port, DIR_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_LATCH_GPIO_Port, DIR_LATCH_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_CLK_GPIO_Port, DIR_CLK_Pin, GPIO_PIN_RESET);

  unsigned char comparator = 0x80;

  for (int i = 0; i<8; i++){
	  if (dir_code & comparator){
		  bit_to_send = GPIO_PIN_SET;
	  }
	  else{
		  bit_to_send = GPIO_PIN_RESET;
	  }

	  comparator = comparator >> 1;

	  HAL_GPIO_WritePin(DIR_SER_GPIO_Port, DIR_SER_Pin, bit_to_send);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(DIR_CLK_GPIO_Port, DIR_CLK_Pin, GPIO_PIN_SET);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(DIR_CLK_GPIO_Port, DIR_CLK_Pin, GPIO_PIN_RESET);
	  HAL_Delay(1);
  }

  HAL_GPIO_WritePin(DIR_LATCH_GPIO_Port, DIR_LATCH_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(DIR_LATCH_GPIO_Port, DIR_LATCH_Pin, GPIO_PIN_RESET);

}

void SetDutyCycle_DC_Motor(TIM_HandleTypeDef *htimx, uint32_t Channel, uint8_t duty_cycle){

	sConfigOC_DC_Motor.Pulse = (duty_cycle*htimx->Init.Period)/100;
	sConfigOC_DC_Motor.OCMode = TIM_OCMODE_PWM1;
	sConfigOC_DC_Motor.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC_DC_Motor.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_Stop(htimx, Channel);
	HAL_TIM_PWM_Init(htimx);
	HAL_TIM_PWM_ConfigChannel(htimx, &sConfigOC_DC_Motor, Channel);
	HAL_TIM_PWM_Start(htimx, Channel);
}
