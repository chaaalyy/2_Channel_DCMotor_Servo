/*
 * motorTask.c
*
 *  Created on: Apr 8, 2020
 *      Author: Karl Hönemann
 */

#include "motorTask.h"
#include "dcMotorController.h"
#include "stepDirReceiver.h"

extern TIM_HandleTypeDef htim1; //Encoder Mot2
extern TIM_HandleTypeDef htim2; //PWM Mot 2 (Channel 1 - PA15) & Mot 1 (Channel 3 - PB10)
extern TIM_HandleTypeDef htim3; //Step signal 1 - ExtClk TI1FP1 mode - PA6
extern TIM_HandleTypeDef htim4; //Step signal 2 - ExtClk TI1FP1 mode - PB6
extern TIM_HandleTypeDef htim5; //Encoder Mot1

// DIR input Pins = PA7 (Sig 1) & PB5 (Sig 2) via ExtI line
// config jumpers : JP1 = PB7 (blue button side) & JP2 = PB2 (reset button side). If low (==gnd), JP is set !

static int32_t extSetPosMot1 = 0;
static int32_t extSetPosMot2 = 0;
static bool singleSourceFlag = false;
static bool motorErrorFlag = false;

static volatile int16_t debDuty1 = 0;
static volatile int16_t debDuty2 = 0;
static volatile int32_t debCurrPos1 = 0;
static volatile int32_t debCurrPos2 = 0;

dcMotor* mot1 = nullptr;
dcMotor* mot2 = nullptr;

stepDirReceiver* sdr1 = nullptr;
stepDirReceiver* sdr2 = nullptr;

void motorInit()
{
	mot1 = new dcMotor (&htim2, TIM_CHANNEL_3, &htim5, GPIOB, GPIO_PIN_4, &motorErrorFlag);
	mot2 = new dcMotor (&htim2, TIM_CHANNEL_1, &htim1, GPIOA, GPIO_PIN_4, &motorErrorFlag);
	setupStepDirSource(GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_2);
}

void motorRoutine()
{
	sdr1->SetStepDirPosition();
	if (!singleSourceFlag)
		sdr2->SetStepDirPosition();
	setTargetPositions();
	checkError(&motorErrorFlag);
	mot1->ExecuteController();
	mot2->ExecuteController();

	/* for debug only */
	debDuty1=mot1->duty;
	debCurrPos1 = mot1->currentPosition;
	debDuty2=mot2->duty;
	debCurrPos2 = mot2->currentPosition;
}

void setTargetPositions(void)
{
	mot1->SetTargetPosition(&extSetPosMot1);
	if (!singleSourceFlag)
		mot2->SetTargetPosition(&extSetPosMot2);
	else
		mot2->SetTargetPosition(&extSetPosMot1);

}

void toggleCounterDirections(void)
{
	sdr1->ToggleCounterDirection();
	if (!singleSourceFlag)
			sdr2->ToggleCounterDirection();
}

void setupStepDirSource(GPIO_TypeDef* JP1_Port, uint16_t JP1_Pin, GPIO_TypeDef* JP2_Port, uint16_t JP2_Pin)
{
    if (JP1_Port->IDR & JP1_Pin) {
    	singleSourceFlag = true;
    	if (JP2_Port->IDR & JP2_Pin) { // (jp NOT set!) use sig 1 or 2 for both
    		sdr1 = new stepDirReceiver (&htim3, GPIOA, GPIO_PIN_7, &extSetPosMot1); // sig 1
    	}
    	else {
    		sdr1 = new stepDirReceiver (&htim4, GPIOB, GPIO_PIN_5, &extSetPosMot1); // sig 2
     	}
    }
    else {
    	singleSourceFlag = false;
    	if (JP2_Port->IDR & JP2_Pin) { // normal (jp NOT set !)
        	sdr1 = new stepDirReceiver (&htim3, GPIOA, GPIO_PIN_7, &extSetPosMot1);
        	sdr2 = new stepDirReceiver (&htim4, GPIOB, GPIO_PIN_5, &extSetPosMot2);
    	}
    	else { // inverse channels (jp set)
        	sdr1 = new stepDirReceiver (&htim3, GPIOA, GPIO_PIN_7, &extSetPosMot2);
        	sdr2 = new stepDirReceiver (&htim4, GPIOB, GPIO_PIN_5, &extSetPosMot1);
    	}
    }
}

void checkError(bool* errorFlag)
{
	if (!motorErrorFlag)
		return;
	else {
		mot1->StopMotor();
		mot2->StopMotor();
		// LD2_GPIO_Port and LD2_Pin are defined in main.h (e.g., GPIOA, GPIO_PIN_5)
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Turn on LD2 (User LED on PA5)
		while(1) {
			asm("nop");
		}
	}
}
