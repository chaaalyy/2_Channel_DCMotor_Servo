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

    STP_CNT_OF = 0; // int16_t: Stores number of overflows/underflows
    // STP_PULSES is no longer used with the new logic
    STP_LAST_VALUE = 0; // uint32_t, but will effectively store uint16_t from timer

    stepPosition = extVar; // int32_t*

    InitHardware();
}

stepDirReceiver::~stepDirReceiver() {
}

void stepDirReceiver::InitHardware() {
   	STEP_TIMER->Instance->ARR = 0xFFFF; // Set timer period to max for full 16-bit range
   	HAL_TIM_Base_Start (STEP_TIMER);
}

void stepDirReceiver::SetStepDirPosition()
{
	uint16_t timCounterTemp = STEP_TIMER->Instance->CNT; // Raw 16-bit timer value

    // Corrected overflow/underflow detection for a 16-bit timer (0x0000 to 0xFFFF).
    // This logic assumes the timer changes by less than 0x8000 (32768) in one sample period.
    // An overflow is detected if the last value was in the upper quadrant (e.g., >0xC000)
    // and the current value is in the lower quadrant (e.g., <0x4000), indicating a wrap-around from 0xFFFF to 0.
    // Cast STP_LAST_VALUE to uint16_t for comparison as it stores the previous timer count.
    if ((uint16_t)STP_LAST_VALUE > 0xC000 && timCounterTemp < 0x4000) {
        STP_CNT_OF++; // Increment overflow counter
    }
    // An underflow is detected if the last value was in the lower quadrant (e.g., <0x4000)
    // and the current value is in the upper quadrant (e.g., >0xC000), indicating a wrap-around from 0 to 0xFFFF.
    else if ((uint16_t)STP_LAST_VALUE < 0x4000 && timCounterTemp > 0xC000) {
        STP_CNT_OF--; // Decrement overflow counter
    }

    // Update last known value with the current raw timer value for the next cycle.
    // Though STP_LAST_VALUE is uint32_t, we are assigning a uint16_t to it.
    // This is acceptable as it's only used to store the previous 16-bit timer state.
    STP_LAST_VALUE = timCounterTemp;

    // Calculate current absolute step position.
    // STP_CNT_OF (int16_t) stores the number of full 16-bit overflows/underflows.
    // timCounterTemp (uint16_t) is the current 16-bit timer count.
    // 65536 (which is 0xFFFF + 1) is used as the multiplier for the overflow count.
    // The target stepPosition is int32_t.
    *stepPosition = (int32_t)(((int32_t)STP_CNT_OF * 65536L) + timCounterTemp);
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
