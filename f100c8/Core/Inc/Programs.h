/*
 * Programs.h
 *
 *  Created on: Jan 17, 2024
 *      Author: Ahmet
 */

#ifndef INC_PROGRAMS_H_
#define INC_PROGRAMS_H_

#include "main.h"

typedef struct Motor{
	TIM_HandleTypeDef Tımer;
	uint32_t *Channel;
	uint16_t *Duty;

};


typedef struct{

     uint16_t MotorSayi;
	 uint16_t MotorDuty;
	 uint16_t TempDuty;

}Control;

typedef struct prg{

	uint8_t time;
	uint8_t temp;
	uint8_t speed;
	uint16_t motor;

}prg;

// Üç programı döndürecek ve seçilen program sayısını gösterecek yeni yapı tanımı
typedef struct {
    prg program1;
    prg program2;
    prg program3;
    uint8_t selectedProgramsCount;
} prgSelection;

void program(uint16_t time);

void f1(int time,int valuduty, int motor,int temp);
prgSelection prgSel();

void standartprogram (prg program,uint8_t pid);


#endif /* INC_PROGRAMS_H_ */
