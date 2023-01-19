/*
 * steppers_control.c
 *
 *      Author: Valerii Chernov
 */

#include "main.h"
#include "steppers_control.h"
#include "math.h"

StepperState *steppers_ptr[STEPPER_UNITS];

StepperState* MotorPointer;

void addStepperPointer(StepperState* motor, int index){
	steppers_ptr[index] = motor;
	motor -> id = index;

	MotorPointer = motor;
}

void steppersInitTimer(TIM_HandleTypeDef* TMR_Handle){
	TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    uint32_t ARR_Value = (STEPPER_TIMER_CLK * 10.0 * STEPPER_TIME_BASE);

    printf(" ... ARR = %u \r\n", ARR_Value);

	TMR_Handle -> Instance               = STEPPER_TIMER;
	TMR_Handle -> Init.Prescaler         = 99; //99
	TMR_Handle -> Init.CounterMode       = TIM_COUNTERMODE_UP;
	TMR_Handle -> Init.Period            = ARR_Value-1;
	TMR_Handle -> Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	TMR_Handle -> Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	HAL_TIM_Base_Init(TMR_Handle);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(TMR_Handle, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

	HAL_TIMEx_MasterConfigSynchronization(TMR_Handle, &sMasterConfig);
	HAL_TIM_Base_Start_IT(TMR_Handle);
}

void steppersSetSpeed(StepperState* motor, float speed){
	//limitSpeed(motor, speed);
	setVel(motor, speed);

	uint32_t Total_Steps = (360. / (motor -> resolution));

	motor -> ticks_max = (STEPPER_TIMER_CLK)/(STEPPER_TIME_BASE * Total_Steps * (motor -> v_goal));

//	printf("... per 2pi: %lu | max: %lu ", Total_Steps, (motor -> ticks_max));
}

void steppersSetPosition(StepperState* motor, int32_t position){
	setGoal(motor, position);
	motor -> status = POSITION_CONTROL;
}

void steppersSpeedControl(StepperState* motor, float speed){
	steppersSetSpeed(motor, speed);

	motor -> status = VELOCITY_CONTROL;
}

void updateKinematicsParams(StepperState* motor){

//	(motor -> v_cur)
//	(motor -> v_max)

	int32_t err = (motor -> goal) - (motor -> steps);
	uint8_t status = motor -> status;

	if (status == VELOCITY_CONTROL){
		int32_t err = (motor -> v_cur) - (motor -> v_goal);

		if ( err > STEPPER_SPEED_TOLERANCE ){
			motor -> v_cur += (motor -> a_cur)*(motor -> step2sec);
			steppersSetSpeed( motor, (motor -> v_cur) );
		}
		if ( err < STEPPER_SPEED_TOLERANCE ){
			motor -> v_cur -= (motor -> a_cur)*(motor -> step2sec);
			steppersSetSpeed( motor, (motor -> v_cur) );
		}
	}

	if (status == POSITION_CONTROL){
		int32_t err = (motor -> steps) - (motor -> goal);

		float limit = 200; //0.5 * pow( (motor -> v_cur), 2) * ctan( motor -> a_max);
		if (abs(err) < limit){
			motor -> a_cur = motor -> a_max;
			motor -> v_cur -= (motor -> a_cur)*(motor -> step2sec);
			steppersSetSpeed( motor, (motor -> v_cur) );
		}
	}
}

void stepperTimerOverflowISR(TIM_HandleTypeDef* htim){
	uint8_t i = 0;

	if(htim -> Instance == STEPPER_TIMER){

		for(i = 0; i < STEPPER_UNITS; i++){
			StepperState* stepper_i = steppers_ptr[i];

			if ( ((stepper_i -> status) != HOLD )  && ((stepper_i -> status) != IDLE ) ){
				if( (stepper_i -> ticks)  >= (stepper_i -> ticks_max) ) {
					HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

					doStep( steppers_ptr[i] );
					stepper_i -> ticks = 0;

					updateKinematicsParams(stepper_i);
				}else{
					stepper_i -> ticks++;
				}
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	stepperTimerOverflowISR(htim);
}
