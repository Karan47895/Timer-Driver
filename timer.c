/*
 * timer.c
 *
 *  Created on: Aug 16, 2024
 *      Author: Karan Patel
 */

#include "timer.h"

void TIMER_Enable(TIMER_RegDef_t *pTIMERx,uint8_t EnorDi){
	if(EnorDi==ENABLE) pTIMERx->CR1 = (1<<0);
	else pTIMERx->CR1 &=~(1<<0);
}


void TIMER_PeriClockControl(TIMER_RegDef_t *pTIMERx,uint8_t EnorDi){
	if(EnorDi==ENABLE){
	if(pTIMERx==TIM1){
       TIM1_PCLK_EN();
	}
	else if(pTIMERx==TIM2){
		TIM2_PCLK_EN();
	}
	else if(pTIMERx==TIM3){
		TIM3_PCLK_EN();
	}
	else if(pTIMERx==TIM4){
		TIM4_PCLK_EN();
	}
	else if(pTIMERx==TIM5){
		TIM5_PCLK_EN();
	}
	else if(pTIMERx==TIM6){
		TIM6_PCLK_EN();
	}
	else if(pTIMERx==TIM7){
		TIM7_PCLK_EN();
	}
	else if(pTIMERx==TIM8){
		TIM8_PCLK_EN();
	}
	else if(pTIMERx==TIM9){
		TIM9_PCLK_EN();
	}
	else if(pTIMERx==TIM10){
		TIM10_PCLK_EN();
	}
	else if(pTIMERx==TIM11){
		TIM11_PCLK_EN();
	}
	else if(pTIMERx==TIM12){
		TIM12_PCLK_EN();
	}
	else if(pTIMERx==TIM13){
		TIM13_PCLK_EN();
	}
	else if(pTIMERx==TIM14){
		TIM14_PCLK_EN();
	}
}
	else if(EnorDi==DISABLE){
	if(pTIMERx==TIM1){
       TIM1_PCLK_DI();
	}
	else if(pTIMERx==TIM2){
		TIM2_PCLK_DI();
	}
	else if(pTIMERx==TIM3){
		TIM3_PCLK_DI();
	}
	else if(pTIMERx==TIM4){
		TIM4_PCLK_DI();
	}
	else if(pTIMERx==TIM5){
		TIM5_PCLK_DI();
	}
	else if(pTIMERx==TIM6){
		TIM6_PCLK_DI();
	}
	else if(pTIMERx==TIM7){
		TIM7_PCLK_DI();
	}
	else if(pTIMERx==TIM8){
		TIM8_PCLK_DI();
	}
	else if(pTIMERx==TIM9){
		TIM9_PCLK_DI();
	}
	else if(pTIMERx==TIM10){
		TIM10_PCLK_DI();
	}
	else if(pTIMERx==TIM11){
		TIM11_PCLK_DI();
	}
	else if(pTIMERx==TIM12){
		TIM12_PCLK_DI();
	}
	else if(pTIMERx==TIM13){
		TIM13_PCLK_DI();
	}
	else if(pTIMERx==TIM14){
		TIM14_PCLK_DI();
	}
}
}



void TIMER_Init(TIMER_handle_t *handler) {
    // Enable the clock access to the peripheral.
    TIMER_PeriClockControl(handler->pTIMERx, ENABLE);

    // Set the prescaler value. Assuming system clock is 16 MHz
    handler->pTIMERx->PSC = 16 - 1;  // 1 us tick

    if(handler->Config.Mode == TIMER_Mode_Output_Compare) {
        // Set output compare mode based on your requirement.
        // Example: toggle output
        handler->pTIMERx->CCMR1 |= (1<<4) | (1<<5);  // Set OC1M bits for toggle
        // Enable the channel 1 in compare mode
        handler->pTIMERx->CCER |= (1<<0);  // Enable CC1
        // Set the auto-reload value
        handler->pTIMERx->ARR = handler->Config.Delayinms;
    }
    else if(handler->Config.Mode == TIMER_Mode_Input_Capture) {
        // Set channel 1 as input, mapped on TI1
        handler->pTIMERx->CCMR1 |= (1<<0);  // Set CC1S bits for input mode
        // Set input capture to capture on the rising edge
        handler->pTIMERx->CCER |= (1<<0);  // Enable capture on rising edge

    }
    else {
        // Set the auto-reload value
        handler->pTIMERx->ARR = handler->Config.Delayinms;
    }

    // Clear counter
    handler->pTIMERx->CNT = 0;


    // Enable the timer
    TIMER_Enable(handler->pTIMERx, ENABLE);
    //TIMER_Enable_In_Interrupt(TIM3);
}



void TIMER_Enable_In_Interrupt(TIMER_RegDef_t *pTIMERx){
	//Enable interrrupt for given timer
	pTIMERx->DIER |=(1<<0);
	//Enable the NVIC priorty in the code .
}


void TIMER_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){
	 if(EnorDi==ENABLE){
		 if(IRQNumber<=31){
			//Program ISER0 register.
           *NVIC_ISER0|=(1<<IRQNumber);
		 }
		 else if(IRQNumber>31 &&IRQNumber<64){
			 //Program ISER1 register.
		  *NVIC_ISER1|=(1<<(IRQNumber)%32);
		 }
		 else if(IRQNumber>=64 &&IRQNumber<96){
			 //Program ISER2 register
			 *NVIC_ISER2|=(1<<(IRQNumber)%64);
		 }
	 }
	 else {
		 if(IRQNumber<=31){
			 *NVIC_ICER0|=(1<<IRQNumber);
		   }
         else if(IRQNumber>31 &&IRQNumber<64){
        	 *NVIC_ICER1|=(1<<(IRQNumber)%32);
		   }
		 else if(IRQNumber>=64 &&IRQNumber<96){
			 *NVIC_ICER2|=(1<<(IRQNumber)%64);
		   }
	 }
}


