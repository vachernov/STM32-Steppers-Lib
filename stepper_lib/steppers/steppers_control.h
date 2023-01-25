/*
 * steppers_control.h
 *
 *      Author: Valerii
 */

#ifndef SRC_STEPPER_LIB_STEPPERS_STEPPERS_CONTROL_H_
#define SRC_STEPPER_LIB_STEPPERS_STEPPERS_CONTROL_H_

#include "../config.h"
#include "../stepper/stepper.h"

/* Prototypes For All Functions  */

void addStepperPointer(StepperState* motor, uint8_t index);
StepperState* getStepperPointer(uint8_t index);

void steppersSetSpeed(StepperState* motor, float speed);
void steppersSetPosition(StepperState* motor, int32_t position);
void steppersSpeedControl(StepperState* motor, float speed);

void steppersSetIdle(StepperState* motor);

void updateKinematicsParams(StepperState* motor);

void steppersInitTimer(TIM_HandleTypeDef* TMR_Handle);
void steppersTimerOverflowISR(TIM_HandleTypeDef* htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

#endif /* SRC_STEPPER_LIB_STEPPERS_STEPPERS_CONTROL_H_ */
