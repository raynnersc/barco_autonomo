/*
 * L293D_STM32F103.h
 *
 *  Created on: Dec 13, 2022
 *      Author: danil
 */

#ifndef INC_L293D_STM32F103_H_
#define INC_L293D_STM32F103_H_

#include <API_STM32F103_SERVOMOTOR_MG90D.h>

typedef enum{

	Stop = 0x00,
	Forward = 0x10,
	Back = 0x02

} Direction;

void Setup_DC_Motor(TIM_HandleTypeDef *htimx, uint32_t Channel);
void SetDirection_DC_Motor(Direction dir_code);
void SetDutyCycle_DC_Motor(TIM_HandleTypeDef *htimx, uint32_t Channel, uint8_t duty_cycle);

#endif /* INC_L293D_STM32F103_H_ */
