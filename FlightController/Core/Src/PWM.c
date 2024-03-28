/*
 * PWM.c
 *
 *  Created on: Mar 11, 2024
 *      Author: MAN-MADE
 *
 *      Здесь определяется реализация генерации ШИМ сигнала на выходах контроллера
 */

#include"main.h"

void setPWM(TIM_HandleTypeDef tim, uint32_t ch,uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&tim, &sConfigOC, ch);
	HAL_TIM_PWM_Stop(&tim, ch);
	HAL_TIM_PWM_Start(&tim, ch);
}

int16_t convertPWM(int16_t command)
{
	int16_t k = (CH1_PWM_MAX - CH1_PWM_MID)/(ROLL_RATE);

	return k*command + CH1_PWM_MID;
}
