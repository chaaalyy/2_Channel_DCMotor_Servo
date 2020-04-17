/*
 * stepDirReceiver.cpp
 *
 *  Created on: 12.04.2020
 *      Author: Karl Hönemann
 */

#include <stepDirReceiver.h>

stepDirReceiver::stepDirReceiver(TIM_HandleTypeDef* stepTimer, GPIO_TypeDef* dirPort, uint16_t dirPin, int32_t* extVar) {
    STEP_TIMER = stepTimer;

    DIR_GPIO_PORT = dirPort;
    DIR_GPIO_PIN = dirPin;

    STP_CNT_OF = 0;
    STP_PULSES = 0;
    STP_LAST_VALUE = 0;

    stepPosition = extVar;

    InitHardware();
}

stepDirReceiver::~stepDirReceiver() {
}

void stepDirReceiver::InitHardware() {
   	STEP_TIMER->Instance->ARR = 0xFFFF;
   	HAL_TIM_Base_Start (STEP_TIMER);
}

void stepDirReceiver::SetStepDirPosition()
{
	uint16_t timCounterTemp = STEP_TIMER->Instance->CNT;
	STP_PULSES= timCounterTemp;
	if (STP_PULSES != STP_LAST_VALUE)
	{
		STP_PULSES &= 0xC000;
		STP_LAST_VALUE &= 0xC000;
		if (STP_PULSES == 0 && STP_LAST_VALUE == 0xC000){
			STP_CNT_OF++;
		}
		if (STP_PULSES == 0xC000 && STP_LAST_VALUE == 0){
			STP_CNT_OF--;
		}
	}

	if (STP_CNT_OF < 0)
		*stepPosition = ((STP_CNT_OF + 1) * (0xFFFF +1)) - ((0xFFFF + 1) - timCounterTemp);
	else
		*stepPosition = (STP_CNT_OF * (0xFFFF +1)) + timCounterTemp;

	STP_LAST_VALUE = timCounterTemp;
}

void stepDirReceiver::ToggleCounterDirection() { //triggered by ExtI !!!
	  if (EXTI->PR & DIR_GPIO_PIN) {                       //EXTI DIR_GPIO_PIN pending?
	    EXTI->PR |= DIR_GPIO_PIN;                          // clear pending interrupt bit

	    if (DIR_GPIO_PORT->IDR & DIR_GPIO_PIN) {   	//returns true if Pin is HIGH)
			STEP_TIMER->Instance->CR1 &= ~TIM_CR1_DIR; //clear bits from mask to set dir upwards
	    }

	    else {
	    	STEP_TIMER->Instance->CR1 |=  TIM_CR1_DIR; //set bits from mask to set dir downwards
	    }
	  }
}
