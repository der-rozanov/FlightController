/*
 * config.h
 *
 *  Created on: Mar 11, 2024
 *      Author: MAN-MADE
 *
 *      Основной конфигурационный файл с настройками полетного контроллера
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/*Рейты*/
#define ROLL_RATE 180
#define PITCH_RATE 180
#define YAW_RATE 90
/*-Рейты*/

/*Границы и центры ПВМ выходов*/
#define CH1_PWM_MAX 2500
#define CH1_PWM_MID 1500
#define CH1_PWM_MIN 500

#define CH2_PWM_MAX 2500
#define CH2_PWM_MID 1500
#define CH2_PWM_MIN 500

#define CH3_PWM_MAX 2500
#define CH3_PWM_MID 1500
#define CH3_PWM_MIN 500
/*-Границы и центры ПВМ выходов*/

/*Значения пидов*/
#define ROLL_KP 1
#define ROLL_KD 1
#define ROLL_KI 1
#define ROLL_FF 0.5

#define PITCH_KP 1
#define PITCH_KD 1
#define PITCH_KI 1
#define PITCH_FF 0.5

#define YAW_KP 1
#define YAW_KD 1
#define YAW_KI 1
#define YAW_FF 0.5
/*-Значения пидов*/

#endif /* INC_CONFIG_H_ */
