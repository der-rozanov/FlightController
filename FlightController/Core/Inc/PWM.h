/*
 * PWM.h
 *
 *  Created on: Mar 11, 2024
 *      Author: MAN-MADE
 *
 *      Здесь объявляются прототипы функций для работы с ШИМ
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

void setPWM(TIM_HandleTypeDef tim, uint32_t ch,uint16_t value);
int16_t convertPWM(int16_t command);

#endif /* INC_PWM_H_ */
