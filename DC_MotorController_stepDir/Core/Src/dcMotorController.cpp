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
	ENC_LAST_VALUE = 0;
	ENC_PULSES 	= 0;
	DIR_GPIO_PORT  = dirPort;
	DIR_GPIO_PIN   = dirPin;
	ERROR_GAP	   = ERROR_TOLERANCE;
	ERROR_FLAG	   = errorFlag;
	pid_param_kp	= PID_KP;
	pid_param_ki	= PID_KI;
	pid_param_kd	= PID_KD;
	targetPosition = 0;
	currentPosition = 0;
	dutyLimit = 1024;
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
	    uint16_t tempEncValue = ENC_TIMER->Instance->CNT;
		ENC_PULSES = tempEncValue;
		if (ENC_PULSES != ENC_LAST_VALUE)
		{
			ENC_PULSES &= 0x0000C000;
			ENC_LAST_VALUE &= 0x0000C000;
			if (ENC_PULSES == 0 && ENC_LAST_VALUE == 0xC000){
				ENC_CNT_OF++;
			}
			if (ENC_PULSES == 0xC000 && ENC_LAST_VALUE == 0){
				ENC_CNT_OF--;
			}
			ENC_LAST_VALUE = tempEncValue;
		}
		if (ENC_CNT_OF < 0)
			currentPosition = ((ENC_CNT_OF + 1) * (0xFFFF +1)) - ((0xFFFF + 1) - tempEncValue);
		else
			currentPosition = (ENC_CNT_OF * (0xFFFF +1)) + tempEncValue;
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

