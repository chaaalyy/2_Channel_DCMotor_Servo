/*
 * motorTask.h
 *
 *  Created on: Apr 8, 2020
 *      Author: Karl Hönemann
 */

#ifndef INC_MOTORTASK_H_
#define INC_MOTORTASK_H_

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "stdbool.h"

void motorInit();
void motorRoutine();

void setTargetPositions();
void toggleCounterDirections();
void setupStepDirSource(GPIO_TypeDef* JP1_Port, uint16_t JP1_Pin, GPIO_TypeDef* JP2_Port, uint16_t JP2_Pin);

void checkError(bool* motorErrorFlag);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTORTASK_H_ */
