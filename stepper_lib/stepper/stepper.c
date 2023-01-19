/*
 * stepper.c
 *
 *      Author: Valerii Chernov
 */

#include "main.h"
#include "stepper.h"
#include "../config.h"

// ToDo : implement HAL initialization for pins
StepperState stepperInit(GpioPin STEP_Pin, GpioPin DIR_Pin, GpioPin EN_Pin){
	/*  configure hardware  */
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// Step
	GPIO_InitStruct.Pin = STEP_Pin.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(STEP_Pin.port, &GPIO_InitStruct);

	// DIR
	GPIO_InitStruct.Pin = DIR_Pin.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DIR_Pin.port, &GPIO_InitStruct);

	// EN
	GPIO_InitStruct.Pin = EN_Pin.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(EN_Pin.port, &GPIO_InitStruct);

	/*  configure identity  */

	StepperState motor;

	motor.steps = 0;
	motor.angle = 0;
	motor.home = 0;
	motor.goal = 0;
	motor.v_goal = 0;
	motor.ticks = 0;
	motor.ticks_max = 0;
	motor.v_cur = 0;
	motor.a_cur = 0;
	motor.resolution = STEPPER_DEFAULT_RES;
	motor.v_max = STEPPER_DEFAULT_V_M;
	motor.a_max = STEPPER_DEFAULT_A_M;
	motor.dir = 0;

	motor.step_pin = STEP_Pin;
	motor.dir_pin = DIR_Pin;
	motor.en_pin = EN_Pin;

	motor.dir_positive = 1;
	motor.dir_inverse = 0;

	motor.en_on = 1;
	motor.en_off = 0;

	motor.id = 0;

	return motor; 
}

void doStep(StepperState* motor){
//	HAL_GPIO_WritePin( (motor -> step_pin).port, (motor -> step_pin).pin, GPIO_PIN_SET);
	HAL_GPIO_TogglePin( (motor -> step_pin).port, (motor -> step_pin).pin);
    motor -> steps += motor -> dir;
    motor -> angle += (motor -> dir)*(motor -> resolution);
}
void releaseStep(StepperState* motor){
	HAL_GPIO_WritePin( (motor -> step_pin).port, (motor -> step_pin).pin, GPIO_PIN_RESET);
}

void setDir(StepperState* motor, int8_t d){
    motor -> dir = d;
    HAL_GPIO_WritePin((motor -> dir_pin).port, (motor -> dir_pin).pin, d == 1 ? (motor -> dir_positive) : (motor -> dir_inverse) );
}

void setVel(StepperState* motor, float velocity){
	if ( abs(velocity) < (motor -> v_max) ){
		motor -> v_goal = velocity;
	}else{
		motor -> v_goal = motor -> v_max;
	}

	setDir( motor, (velocity > 0) ? (motor -> dir_positive) : (motor -> dir_inverse) );

	if ((motor -> dir) == 0){
		motor -> status = HOLD;
	}
}

void setAcc(StepperState* motor, float acceleration){
	if ( abs(acceleration) < (motor -> a_max) ){
		motor -> a_cur = acceleration;
	}else{
		motor -> a_cur = motor -> a_max;
	}

	setDir( motor, (acceleration > 0) ? (motor -> dir_positive) : (motor -> dir_inverse) );
}

void updateStep2Sec(StepperState* motor){
	motor -> step2sec = (360. / (motor -> resolution)) * (motor -> resolution);
}

// usage: limitAcceleration(&motor_A, 100);
void limitAcceleration(StepperState* motor, uint32_t a){ // steps/s^2
	if (a > 0){
		motor -> a_max = a;
		setAcc(motor, a);
		printf(" --> acceleration limit for ID %u has been set to %lu \r\n", (motor -> id), (motor -> a_max));
	}
}

void limitSpeed(StepperState* motor, uint32_t v){ // steps/s
	if (v > 0){
		motor -> v_max = v;
		setVel(motor, v);
		printf(" --> speed limit for ID %u has been set to %lu \r\n", (motor -> id), (motor -> v_max));
	}
}

void saveHome(StepperState* motor){
    motor -> home = motor -> steps;
}

void setHome(StepperState* motor, int32_t position){ // steps
    motor -> home = position;
}

void setGoal(StepperState* motor, int32_t position){ // steps
	setDir(motor, (position > (motor -> steps)) ? 1 : -1);
    motor -> goal = position;
}

void setGoalRel(StepperState* motor, int32_t delta)
{
    setDir(motor, delta < 0 ? -1 : 1);
    motor -> goal = (motor -> steps) + delta;
}