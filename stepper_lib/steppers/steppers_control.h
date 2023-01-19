/*
 * steppers_control.h
 *
 *      Author: Valerii Chernov
 */

#ifndef SRC_STEPPER_LIB_STEPPERS_STEPPERS_CONTROL_H_
#define SRC_STEPPER_LIB_STEPPERS_STEPPERS_CONTROL_H_

#include "../config.h"
#include "../stepper/stepper.h"

/* prototypes for all functions  */

void steppersInitTimer(TIM_HandleTypeDef* TMR_Handle);
void steppersSetSpeed(StepperState* motor, float speed);
void stepperTimerOverflowISR(TIM_HandleTypeDef* htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

#endif /* SRC_STEPPER_LIB_STEPPERS_STEPPERS_CONTROL_H_ */
