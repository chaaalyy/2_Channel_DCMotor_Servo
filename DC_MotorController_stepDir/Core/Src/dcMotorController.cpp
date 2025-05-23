/*
 * dcMotorController.cpp
 *
 *  Created on: Apr 7, 2020
 *      Author: Karl Hönemann
 */

#include "dcMotorController.h"
#include "stdlib.h"


dcMotor::dcMotor(TIM_HandleTypeDef* pwmTimer, uint32_t pwmChannel, TIM_HandleTypeDef* encTimer, GPIO_TypeDef* dirPort, uint16_t dirPin, bool* errorFlag)
{
	PWM_TIMER      = pwmTimer;
	PWM_CHANNEL    = pwmChannel;
	ENC_TIMER		= encTimer;
	ENC_CNT_OF		= 0;
	ENC_LAST_VALUE = 0; // Stores the previous raw 16-bit timer value
	// ENC_PULSES is no longer used with the new logic
	DIR_GPIO_PORT  = dirPort;
	DIR_GPIO_PIN   = dirPin;
	ERROR_GAP	   = ERROR_TOLERANCE;
	ERROR_FLAG	   = errorFlag;
	pid_param_kp	= PID_KP;
	pid_param_ki	= PID_KI;
	pid_param_kd	= PID_KD;
	targetPosition = 0;
	currentPosition = 0; // This is a double
	// dutyLimit corresponds to the timer's Auto-Reload Register (ARR) value.
	// Setting to 999 means the timer counts 0-999 (1000 steps),
	// consistent with TIM2 initialization (htim2.Init.Period = 1000-1) in main.c.
	dutyLimit = 999;
	duty = 0;

	pidSystem = new PID (&currentPosition, &duty, &targetPosition, pid_param_kp, pid_param_ki, pid_param_kd, DIRECT);
	InitPID();
	InitHardware();
};

dcMotor::~dcMotor() {
	delete pidSystem;
}

void dcMotor::InitHardware()
{
	/* set up timers */
    ENC_TIMER->Instance->ARR = 0xFFFF; //max. to ensure correct of & uf check with 16 and 32bit timer counters
   	HAL_TIM_Encoder_Start (ENC_TIMER, TIM_CHANNEL_ALL);

   	StopMotor();

   	PWM_TIMER->Instance->ARR = dutyLimit;
   	HAL_TIM_PWM_Start (PWM_TIMER, PWM_CHANNEL);
}

void dcMotor::InitPID()
{
	pidSystem->SetOutputLimits((dutyLimit*(-1)), dutyLimit);
	pidSystem->SetSampleTime(1);
	pidSystem->SetMode(AUTOMATIC);
}

void dcMotor::SetCurrentPosition()
{
    uint16_t tempEncValue = ENC_TIMER->Instance->CNT; // Raw 16-bit timer value

    // Corrected overflow/underflow detection for a 16-bit timer (0x0000 to 0xFFFF).
    // This logic assumes the timer changes by less than 0x8000 (32768) in one sample period,
    // which is a common assumption for encoder readings.
    // An overflow is detected if the last value was in the upper quadrant (e.g., >0xC000)
    // and the current value is in the lower quadrant (e.g., <0x4000), indicating a wrap-around from 0xFFFF to 0.
    if (ENC_LAST_VALUE > 0xC000 && tempEncValue < 0x4000) {
        ENC_CNT_OF++; // Increment overflow counter
    }
    // An underflow is detected if the last value was in the lower quadrant (e.g., <0x4000)
    // and the current value is in the upper quadrant (e.g., >0xC000), indicating a wrap-around from 0 to 0xFFFF.
    else if (ENC_LAST_VALUE < 0x4000 && tempEncValue > 0xC000) {
        ENC_CNT_OF--; // Decrement overflow counter
    }

    ENC_LAST_VALUE = tempEncValue; // Update last known value with the current raw timer value for the next cycle.

    // Calculate current absolute position.
    // ENC_CNT_OF (int16_t) stores the number of full 16-bit overflows/underflows.
    // tempEncValue (uint16_t) is the current 16-bit timer count within the latest 0xFFFF cycle.
    // 65536.0 (which is 0xFFFF + 1) is used as the multiplier for the overflow count.
    // currentPosition is a double, so explicit casting of ENC_CNT_OF to int32_t first, then to double for multiplication,
    // ensures the multiplication is done with appropriate precision.
    currentPosition = (double)(((int32_t)ENC_CNT_OF * 65536.0) + (double)tempEncValue);
}

void dcMotor::SetTargetPosition(int32_t* value)
{
	targetPosition = *value;
}

void dcMotor::ExecuteController()
{
	SetCurrentPosition();
	if (currentPosition != targetPosition)
	{
		if ((abs(targetPosition - currentPosition)) > ERROR_GAP) {
			duty = 0;
			StopMotor();
			*ERROR_FLAG = true;
			return;
		}
		pidSystem->Compute();

		if (duty > 0) HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_RESET);

		switch(PWM_CHANNEL) {
		  case TIM_CHANNEL_1: PWM_TIMER->Instance->CCR1 = abs(duty); break;
		  case TIM_CHANNEL_2: PWM_TIMER->Instance->CCR2 = abs(duty); break;
		  case TIM_CHANNEL_3: PWM_TIMER->Instance->CCR3 = abs(duty); break;
		  case TIM_CHANNEL_4: PWM_TIMER->Instance->CCR4 = abs(duty); break;
		  default: break;
		}
	}
}

void dcMotor::StopMotor()
{
   	switch(PWM_CHANNEL) {
   		case TIM_CHANNEL_1: PWM_TIMER->Instance->CCR1 = 0; break;
	    case TIM_CHANNEL_2: PWM_TIMER->Instance->CCR2 = 0; break;
	    case TIM_CHANNEL_3: PWM_TIMER->Instance->CCR3 = 0; break;
	  	case TIM_CHANNEL_4: PWM_TIMER->Instance->CCR4 = 0; break;}

   	targetPosition = currentPosition;
   	HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_RESET);
}
