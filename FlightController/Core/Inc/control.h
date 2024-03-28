/*
 * control.h
 *
 *  Created on: Mar 17, 2024
 *      Author: MAN-MADE
 *
 *      здесь объявляются прототипы функций ПИД контроллера
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

struct PID
{
	int16_t kp;
	int16_t kd;
	int16_t ki;
	float ff;

	int16_t error;
	int16_t sum_error;
	int16_t old_error;
};

int16_t updatePID(struct PID ch,float desire, float data, int32_t time);

#endif /* INC_CONTROL_H_ */
