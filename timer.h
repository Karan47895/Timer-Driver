/*
 * timer.h
 *
 *  Created on: Aug 16, 2024
 *      Author: Karan Patel
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_
#include "stm32f407xx.h"

typedef struct {
	uint32_t Delayinms;
	uint32_t Mode;
}TIMER_Config;


typedef struct{
	TIMER_RegDef_t *pTIMERx;
	TIMER_Config Config;
}TIMER_handle_t;


// Modes for TIMER
#define TIMER_Mode_Output_Compare 2
#define TIMER_Mode_Normal         3
#define TIMER_Mode_Input_Capture  4

void TIMER_PeriClockControl(TIMER_RegDef_t *pTIMERx,uint8_t EnorDi);
void TIMER_Init(TIMER_handle_t *handler);
void TIMER_Enable(TIMER_RegDef_t *pTIMERx,uint8_t EnorDi);
void TIMER_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void TIMER_Enable_In_Interrupt(TIMER_RegDef_t *pTIMERx);

#endif /* INC_TIMER_H_ */
