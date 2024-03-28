/*
 * control.c
 *
 *  Created on: Mar 17, 2024
 *      Author: MAN-MADE
 *
 *      десь определяются функции работы ПИД контроллера
 */

#include"main.h"

int16_t updatePID(struct PID ch,float desire, float data, int32_t time)
{
	ch.error = desire - data;

	ch.sum_error += ch.error;

	int16_t command = ch.ff*(ch.kp*ch.error); //+ (float)Kd*(x_error - x_old_error)/((float)pidCycleTime/1000000) + Ki*x_sum_error;

	ch.old_error = ch.error;

	return command;
}
