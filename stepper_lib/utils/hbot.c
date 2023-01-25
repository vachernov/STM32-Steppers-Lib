/*
 * hbot.c
 *
 *      Author: Valerii
 */

#include "main.h"
#include "math.h"
#include "hbot.h"
//#include "../config.h"
#include "../steppers/steppers_control.h"

/* Transform configuration space {q_1, q_2} \in C to task space {x, y} \in T
 * | x |       | -1   1 | | q_1 |
 * |   | = r_d |        | |     |
 * | y |       | -1  -1 | | q_2 |
 *
 *   z   = r_d q_3
 */

void directSpaceTransform(HBotState* machine, int32_t q_1, int32_t q_2, int32_t q_3){
	machine -> x = (machine -> r_d) * ( -q_1 + q_2 );
	machine -> y = (machine -> r_d) * ( -q_1 - q_2 );
	machine -> z = (machine -> r_d) * q_3;
}

/* Transform task space {x, y} \in T to configuration space {q_1, q_2} \in C
 * | q_1 |    1           | -1  -1 | | x |
 * |     | = --- r_d^{-1} |        | |   |
 * | q_2 |    2           |  1  -1 | | y |
 *
 *   q_3   =  r_d^{-1} z
 */

//pow( (motor -> v_cur), 2)

void inverseSpaceTransform(HBotState* machine, int32_t x, int32_t y, int32_t z){
	machine -> q_1 = (machine -> r_d_inv) * ( -x - y ) / 2;
	machine -> q_2 = (machine -> r_d_inv) * (  x - y ) / 2;
	machine -> q_3 = (machine -> r_d_inv) * z;
}

//extern TIM_HandleTypeDef htim2;
HBotState hbotInit(StepperState* motorX, StepperState* motorY, StepperState* motorZ,
			  	   GpioPin switchXmin,  GpioPin switchXmax,
				   GpioPin switchYmin,  GpioPin switchYmax,
				   GpioPin switchZmin,  GpioPin switchZmax,
				   TIM_HandleTypeDef* TMR_Handle){
	HBotState machine;

	/* Steppers */

	machine.motor_x_id = HBOT_DEFAULT_X_ID;
	machine.motor_y_id = HBOT_DEFAULT_Y_ID;
	machine.motor_z_id = HBOT_DEFAULT_Z_ID;

	machine.x = HBOT_DEFAULT_X_POS;
	machine.y = HBOT_DEFAULT_Y_POS;
	machine.z = HBOT_DEFAULT_Z_POS;

	addStepperPointer(motorX, machine.motor_x_id);
	addStepperPointer(motorY, machine.motor_y_id);
	addStepperPointer(motorZ, machine.motor_z_id);

	steppersInitTimer(TMR_Handle);

	/* Limit switches */
	machine.x_min_limit = switchXmin;
	machine.x_max_limit = switchXmax;
	machine.y_min_limit = switchYmin;
	machine.y_max_limit = switchYmax;
	machine.z_min_limit = switchZmin;
	machine.z_max_limit = switchZmax;

	//  configure hardware
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// X_min
	GPIO_InitStruct.Pin = switchXmin.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // or GPIO_MODE_IT_FALLING
	GPIO_InitStruct.Pull = GPIO_NOPULL;          // or GPIO_PULLUP
	HAL_GPIO_Init(switchXmin.port, &GPIO_InitStruct);

	// X_max
	GPIO_InitStruct.Pin = switchXmax.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // or GPIO_MODE_IT_FALLING
	GPIO_InitStruct.Pull = GPIO_NOPULL;          // or GPIO_PULLUP
	HAL_GPIO_Init(switchXmax.port, &GPIO_InitStruct);

	// Y_min
	GPIO_InitStruct.Pin = switchYmin.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // or GPIO_MODE_IT_FALLING
	GPIO_InitStruct.Pull = GPIO_NOPULL;          // or GPIO_PULLUP
	HAL_GPIO_Init(switchYmin.port, &GPIO_InitStruct);

	// Y_max
	GPIO_InitStruct.Pin = switchYmax.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // or GPIO_MODE_IT_FALLING
	GPIO_InitStruct.Pull = GPIO_NOPULL;          // or GPIO_PULLUP
	HAL_GPIO_Init(switchYmax.port, &GPIO_InitStruct);

	// Z_min
	GPIO_InitStruct.Pin = switchZmin.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // or GPIO_MODE_IT_FALLING
	GPIO_InitStruct.Pull = GPIO_NOPULL;          // or GPIO_PULLUP
	HAL_GPIO_Init(switchZmin.port, &GPIO_InitStruct);

	// Z_max
	GPIO_InitStruct.Pin = switchZmax.pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // or GPIO_MODE_IT_FALLING
	GPIO_InitStruct.Pull = GPIO_NOPULL;          // or GPIO_PULLUP
	HAL_GPIO_Init(switchZmax.port, &GPIO_InitStruct);

	/* Other */
	machine.r_d = HBOT_DEFAULT_RD;
	machine.r_d_inv = (1.0/HBOT_DEFAULT_RD);

	machine.is_initialized = 0; // have to call findHome

	return machine;
}

/*
 *  Assume XY switches placed in X_MIN, Y_MIN
 */
void hbotFindHome(HBotState* machine){
	StepperState* motor_x = getStepperPointer(machine -> motor_x_id);
	StepperState* motor_y = getStepperPointer(machine -> motor_x_id);
	StepperState* motor_z = getStepperPointer(machine -> motor_x_id);

	steppersSpeedControl(motor_x, HBOT_DEFAULT_X_VEL);
	steppersSpeedControl(motor_y, HBOT_DEFAULT_Y_VEL);
	steppersSpeedControl(motor_z, HBOT_DEFAULT_Z_VEL);

	uint8_t isNotDone = 1;
	uint8_t isXMinReached = 0;
	uint8_t isXMaxReached = 0;
	uint8_t isYMinReached = 0;
	uint8_t isYMaxReached = 0;
	uint8_t isZMinReached = 0;
	uint8_t isZMaxReached = 0;

	while (isNotDone){
		if( HAL_GPIO_ReadPin( (machine -> x_min_limit).port, (machine -> x_min_limit).pin) ){
			isXMinReached = 1;
			steppersSpeedControl(motor_x, -HBOT_DEFAULT_X_VEL);
			saveMinLimit(motor_x);
		}

		if( HAL_GPIO_ReadPin( (machine -> x_max_limit).port, (machine -> x_max_limit).pin) ){
			isXMaxReached = 1;
			steppersSpeedControl(motor_x, 0.0);
			saveMaxLimit(motor_x);
		}

		if( HAL_GPIO_ReadPin( (machine -> y_min_limit).port, (machine -> y_min_limit).pin) ){
			isYMinReached = 1;
			steppersSpeedControl(motor_y, -HBOT_DEFAULT_Z_VEL);
			saveMinLimit(motor_y);
		}

		if( HAL_GPIO_ReadPin( (machine -> y_max_limit).port, (machine -> y_max_limit).pin) ){
			isYMaxReached = 1;
			steppersSpeedControl(motor_y, 0.0);
			saveMaxLimit(motor_y);
		}

		if( HAL_GPIO_ReadPin( (machine -> z_min_limit).port, (machine -> z_min_limit).pin) ){
			isZMinReached = 1;
			steppersSpeedControl(motor_z, -HBOT_DEFAULT_Z_VEL);
			saveMinLimit(motor_z);
		}

		if( HAL_GPIO_ReadPin( (machine -> z_max_limit).port, (machine -> z_max_limit).pin) ){
			isZMaxReached = 1;
			steppersSpeedControl(motor_z, 0.0);
			saveMaxLimit(motor_z);
		}

		if( isXMinReached && isXMaxReached && isYMinReached && isYMaxReached && isZMinReached && isZMaxReached ){
			isNotDone = 0;
		}

		HAL_Delay(HBOT_DEFAULT_DT_INIT);
	}

	calculateHome(motor_x);
	calculateHome(motor_y);
	calculateHome(motor_z);

	machine -> is_initialized = 1;
}

void hbotGoToHome(HBotState* machine){
	if (machine -> is_initialized){
		StepperState* motor_x = getStepperPointer(machine -> motor_x_id);
		StepperState* motor_y = getStepperPointer(machine -> motor_x_id);
		StepperState* motor_z = getStepperPointer(machine -> motor_x_id);

		steppersSetPosition(motor_x, motor_x -> home);
		steppersSetPosition(motor_y, motor_y -> home);
		steppersSetPosition(motor_z, motor_z -> home);
	}
}

void hbotGoToPoint(HBotState* machine, int32_t x, int32_t y, int32_t z){
	if (machine -> is_initialized){
		StepperState* motor_x = getStepperPointer(machine -> motor_x_id);
		StepperState* motor_y = getStepperPointer(machine -> motor_x_id);
		StepperState* motor_z = getStepperPointer(machine -> motor_x_id);

		inverseSpaceTransform(machine, x, y, z);

		steppersSetPosition(motor_x, machine -> q_1);
		steppersSetPosition(motor_y, machine -> q_2);
		steppersSetPosition(motor_z, machine -> q_3);
	}
}

void hbotSetSpeed(HBotState* machine, int32_t v_x, int32_t v_y, int32_t v_z){
	if (machine -> is_initialized){
		StepperState* motor_x = getStepperPointer(machine -> motor_x_id);
		StepperState* motor_y = getStepperPointer(machine -> motor_x_id);
		StepperState* motor_z = getStepperPointer(machine -> motor_x_id);

		inverseSpaceTransform(machine, v_x, v_y, v_z);

		steppersSpeedControl(motor_x, machine -> q_1);
		steppersSpeedControl(motor_y, machine -> q_2);
		steppersSpeedControl(motor_z, machine -> q_3);
	}
}

uint8_t hbotGetXMinSwitchState(HBotState* machine){
	return HAL_GPIO_ReadPin( (machine -> x_min_limit).port, (machine -> x_min_limit).pin);
}

uint8_t hbotGetXMaxSwitchState(HBotState* machine){
	return HAL_GPIO_ReadPin( (machine -> x_max_limit).port, (machine -> x_max_limit).pin);
}

uint8_t hbotGetYMinSwitchState(HBotState* machine){
	return HAL_GPIO_ReadPin( (machine -> y_min_limit).port, (machine -> y_min_limit).pin);
}

uint8_t hbotGetYMaxSwitchState(HBotState* machine){
	return HAL_GPIO_ReadPin( (machine -> y_max_limit).port, (machine -> y_max_limit).pin);
}

uint8_t hbotGetZMinSwitchState(HBotState* machine){
	return HAL_GPIO_ReadPin( (machine -> z_min_limit).port, (machine -> z_min_limit).pin);
}

uint8_t hbotGetZMaxSwitchState(HBotState* machine){
	return HAL_GPIO_ReadPin( (machine -> z_max_limit).port, (machine -> z_max_limit).pin);
}

/*
 * Calculete L1-norm pose vector error
 */
uint32_t hbotGetL1PositionError(HBotState* machine){
	if (machine -> is_initialized){
		StepperState* motor_x = getStepperPointer(machine -> motor_x_id);
		StepperState* motor_y = getStepperPointer(machine -> motor_x_id);
		StepperState* motor_z = getStepperPointer(machine -> motor_x_id);

		uint32_t err = abs( getGoalErr(motor_x) );

		if (err < abs( getGoalErr(motor_y) ) ){
			err = abs( getGoalErr(motor_y) );
		}

		if (err < abs( getGoalErr(motor_z) ) ){
			err = abs( getGoalErr(motor_z) );
		}

		return err;
	}else{
		return -1;
	}
}



