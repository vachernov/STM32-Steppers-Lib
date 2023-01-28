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

void addStepperPointer(StepperState* motor, uint8_t index){
	steppers_ptr[index] = motor; // may be  ptr[i] = &var[i];
	motor -> id = index;
	motor -> status = IDLE;

	MotorPointer = motor;
	// usage:
	// printf("Value of var[%d] = %d\n", i, *ptr[i] );
}

StepperState* getStepperPointer(uint8_t index){
	return steppers_ptr[index];
}
/*                F_tim_clk
 *  F_tim =  ----------------------
 *            (PSC + 1)*(ARR + 1)
 *
 *            PSC = Prescaler - 16 bit ( 0 ... 65535 )
 *            ARR = Auto Reload Register - 16 bit ( 0 ... 65535 )
 */
void steppersInitSignalTimer(TIM_HandleTypeDef* TMR_Handle){
	TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    uint32_t ARR_Value = STEPPER_TIMER_CLK / (STEPPER_TIMER_FREQ * STEPPER_TIMER_PSC);

    printf(" ... ARR = %u \r\n", ARR_Value);

	TMR_Handle -> Instance               = STEPPER_TIMER;
	TMR_Handle -> Init.Prescaler         = STEPPER_TIMER_PSC - 1; // 99
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

void steppersSetSpeed(StepperState* motor, float speed_in_steps_per_sec){
	//limitSpeed(motor, speed);
	setGoalVel(motor, speed_in_steps_per_sec);

//	uint32_t Total_Steps = (360. / (motor -> resolution));

//	if (abs(speed_in_steps_per_sec) > STEPPER_SPEED_TOLERANCE){
//		motor -> ticks_max = STEPPER_TIMER_FREQ / (2*(motor -> v_goal)); // dive by 2 !
//		motor -> step2sec = 1.0/speed_in_steps_per_sec;
//	}else{
//		motor -> ticks_max = STEPPER_TIMER_FREQ / (2*STEPPER_SPEED_TOLERANCE); // dive by 2 !
//		motor -> step2sec = 1.0/STEPPER_SPEED_TOLERANCE;
//	}

	if (abs(motor -> v_cur) < STEPPER_SPEED_TOLERANCE){
		motor -> ticks_max = STEPPER_TIMER_FREQ / (2*STEPPER_SPEED_TOLERANCE); // dive by 2 !
		motor -> step2sec = 1.0/STEPPER_SPEED_TOLERANCE;
	}

//	printf("... per 2pi: %lu | max: %lu ", Total_Steps, (motor -> ticks_max));
}

void steppersUpdateSpeed(StepperState* motor, float speed_in_steps_per_sec){
	setCurVel(motor, speed_in_steps_per_sec);

	if (abs(speed_in_steps_per_sec) > STEPPER_SPEED_TOLERANCE){
		motor -> ticks_max = STEPPER_TIMER_FREQ / (2*abs(motor -> v_cur)); // dive by 2 !
		motor -> step2sec = 1.0/abs(speed_in_steps_per_sec);
	}else{
		motor -> ticks_max = STEPPER_TIMER_FREQ / (2*STEPPER_SPEED_TOLERANCE); // dive by 2 !
		motor -> step2sec = 1.0/STEPPER_SPEED_TOLERANCE;
	}
}

void steppersSetPosition(StepperState* motor, int32_t position){
	setGoal(motor, position);
}

void steppersPositionControl(StepperState* motor, int32_t position){
	steppersSetPosition(motor, position);

	//ensure velocity sign is right
	if (((motor -> steps) - (motor -> goal)) < 0){
		steppersSetSpeed(motor, abs( (motor -> v_goal) ));

	}else{
		steppersSetSpeed(motor, -abs( (motor -> v_goal) ));
	}

	motor -> status = POSITION_CONTROL;
}

void steppersSpeedControl(StepperState* motor, float speed_in_steps_per_sec){
	steppersSetSpeed(motor, speed_in_steps_per_sec);
	motor -> status = VELOCITY_CONTROL;
}

void steppersSetIdle(StepperState* motor){
	motor -> status = IDLE;
}

int8_t sign(float x){
	if (x > 0){
		return 1;
	}else{
		if (x < 0){
			return -1;
		}else{
			return 0;
		}
	}
}

void updateKinematicsParams(StepperState* motor){
	uint8_t status = motor -> status;

	if (status == VELOCITY_CONTROL){
		float err = (motor -> v_cur) - (motor -> v_goal);

		if ( abs(err) > STEPPER_SPEED_TOLERANCE ){
			float dVel;
			if (err > 0) {
				dVel = -((motor -> a_max)*(motor -> step2sec));
			}else{
				dVel =   (motor -> a_max)*(motor -> step2sec);
			}
//			dVel = -sign(err)*(motor -> a_max)*(motor -> step2sec);

			if (abs(dVel) > abs(err)){
				dVel = err;
			}

//				printf(" .... dVel : %f \r\n", dVel);

			motor -> v_cur += dVel;
//			motor -> v_cur +=  -sign(err);

			steppersUpdateSpeed(motor, motor -> v_cur);
		}

		if ( ( abs(motor -> v_cur) < (STEPPER_SPEED_TOLERANCE + 1) ) && ( abs(motor -> v_goal) < (STEPPER_SPEED_TOLERANCE + 1) ) ){
			motor -> status = HOLD;
		}
	}

	if (status == POSITION_CONTROL){
		int32_t err = (motor -> steps) - (motor -> goal);

		if (abs(err) < STEPPER_POSITION_TOLERANCE){
			steppersSetSpeed(motor, 0.0);
			steppersUpdateSpeed(motor, 0.0);
			motor -> status = HOLD;
		}else{
			// S = (v_0^2 + v^2)/(2*a)
			float limit = 0.5 * pow((motor -> v_cur), 2) / ( motor -> a_max );
			if (abs(err) < limit){
				//deceleration
//				motor -> a_cur = sign(err)*(motor -> a_max);
				// motor -> v_cur += (motor -> a_cur)*(motor -> step2sec);
//				motor -> v_cur = (motor -> v_goal)*(abs(err)/limit);

				motor -> v_cur += sign(err)*(motor -> a_max)*(motor -> step2sec);

				steppersUpdateSpeed(motor, motor -> v_cur);
			}else{
				// acceleration
				float err = (motor -> v_cur) - (motor -> v_goal);

				if ( abs(err) > STEPPER_SPEED_TOLERANCE ){
					motor -> v_cur += -sign(err)*(motor -> a_max)*(motor -> step2sec);

					steppersUpdateSpeed(motor, motor -> v_cur);
				}
			}
		}
	}
}

void steppersSignalTimerOverflowISR(TIM_HandleTypeDef* htim){
	uint8_t i = 0;

	if(htim -> Instance == STEPPER_TIMER){

		for(i = 0; i < STEPPER_UNITS; i++){
			StepperState* stepper_i = steppers_ptr[i];

			if ( ((stepper_i -> status) != HOLD )  && ((stepper_i -> status) != IDLE ) ){
				if( (stepper_i -> ticks)  >= (stepper_i -> ticks_max) ) {
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
	// LED blinks
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	steppersSignalTimerOverflowISR(htim);
}
