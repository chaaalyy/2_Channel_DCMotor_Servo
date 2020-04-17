/*
 * dcMotorController.h
 *
 *  Created on: Apr 7, 2020
 *      Author: Karl Hönemann
 */

#ifndef INC_DCMOTORCONTROLLER_H_
#define INC_DCMOTORCONTROLLER_H_

#include "stm32f4xx_hal.h"
#include "PID_v1.h"

#define PID_KP        9.3295         	/* Proportional 33295 */
#define PID_KI        0.03236      	/* Integral 06472 */
#define PID_KD        0.51168       /* Derivative 051168 */

#define ERROR_TOLERANCE 1120

class dcMotor {

public:

	dcMotor(TIM_HandleTypeDef*, uint32_t, TIM_HandleTypeDef*, GPIO_TypeDef*, uint16_t, bool*);

	virtual ~dcMotor();

    // PID parameters for pwm duty cycle control
    double pid_param_kp;
    double pid_param_ki;
    double pid_param_kd;
    double duty;
    int16_t dutyLimit;

    // where do we want to go?
    double     targetPosition;

    // where are we?
    double     currentPosition;

    void SetCurrentPosition();
    void SetTargetPosition(int32_t* value);
    void ExecuteController();
    void StopMotor();

    PID* pidSystem;

private:

    void InitHardware();
    void InitPID();

    // reference to pwm timer and its channel
    TIM_HandleTypeDef * PWM_TIMER;
    uint32_t  PWM_CHANNEL;

    // reference to stepper direction control pin
    GPIO_TypeDef * DIR_GPIO_PORT;
    uint16_t DIR_GPIO_PIN;

    // reference to encoder timer
    TIM_HandleTypeDef * ENC_TIMER;

    // reference to encoder counter variables
    int16_t ENC_CNT_OF;
    uint16_t ENC_PULSES;
    uint16_t ENC_LAST_VALUE;

    uint16_t ERROR_GAP;
    bool* ERROR_FLAG;
};


#endif /* INC_DCMOTORCONTROLLER_H_ */
