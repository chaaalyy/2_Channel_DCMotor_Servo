/*
 * stepDirReceiver.h
 *
 *  Created on: 12.04.2020
 *      Author: Karl Hönemann
 */

#ifndef INC_STEPDIRRECEIVER_H_
#define INC_STEPDIRRECEIVER_H_

#include "stm32f4xx_hal.h"

class stepDirReceiver {
public:
	int32_t* stepPosition;

	stepDirReceiver(TIM_HandleTypeDef*, GPIO_TypeDef*, uint16_t, int32_t*);
	virtual ~stepDirReceiver();

    void SetStepDirPosition();
	void ToggleCounterDirection();

private:
    // reference to step timer and its channel
    TIM_HandleTypeDef * STEP_TIMER;

    // reference to direction control pin
    GPIO_TypeDef * DIR_GPIO_PORT;
    uint16_t DIR_GPIO_PIN;

    // reference to encoder counter variables
    int16_t STP_CNT_OF;
    uint32_t STP_PULSES;
    uint32_t STP_LAST_VALUE;

    void InitHardware();
};


#endif /* INC_STEPDIRRECEIVER_H_ */

