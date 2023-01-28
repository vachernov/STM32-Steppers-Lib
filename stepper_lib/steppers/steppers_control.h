/*
 * steppers_control.h
 *
 *      Author: Valerii Chernov
 */

#ifndef SRC_STEPPER_LIB_STEPPERS_STEPPERS_CONTROL_H_
#define SRC_STEPPER_LIB_STEPPERS_STEPPERS_CONTROL_H_

#include "../config.h"
#include "../stepper/stepper.h"

/* Prototypes For All Functions  */

void addStepperPointer(StepperState* motor, uint8_t index);
StepperState* getStepperPointer(uint8_t index);

void steppersSetSpeed(StepperState* motor, float speed_in_steps_per_sec);
void steppersSetPosition(StepperState* motor, int32_t position);
void steppersSpeedControl(StepperState* motor, float speed_in_steps_per_sec);
void steppersPositionControl(StepperState* motor, int32_t position);

void steppersSetIdle(StepperState* motor);

void updateKinematicsParams(StepperState* motor);

void steppersInitSignalTimer(TIM_HandleTypeDef* TMR_Handle);
void steppersSignalTimerOverflowISR(TIM_HandleTypeDef* htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

#endif /* SRC_STEPPER_LIB_STEPPERS_STEPPERS_CONTROL_H_ */
