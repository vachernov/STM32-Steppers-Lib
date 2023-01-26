/*
 * stepper.h
 *
 *      Author: Valerii Chernov
 */

#ifndef SRC_STEPPER_LIB_STEPPER_STEPPER_H_
#define SRC_STEPPER_LIB_STEPPER_STEPPER_H_

#include "../config.h"

#define abs(x) ((x) > 0 ? (x) : -(x))

typedef enum{
	IDLE = 0,
	HOLD, // 1
	POSITION_CONTROL, // 2
	VELOCITY_CONTROL // 3
}StepperStatus;

typedef struct{
	/*  state-space  */
	int32_t steps;     // current steps
	float angle;       // current angle (deg)
	
	uint32_t ticks;    // timer purposed stuff
	uint32_t ticks_max;

	int32_t home;      // home position (in steps)
	int32_t limit_min; // minimum allowed steps
	int32_t limit_max; // maximum allowed steps

	int32_t goal;      // goal position (in steps)
	float v_goal;    // target velocity (steps/s)

	float v_cur;     // current velocity (steps/s)
	float a_cur;     // current acceleration (steps/s^2)

	float v_max;     // velocity limit (steps/s)
	float a_max;     // acceleration limit (steps/s^2)

	float resolution;  // degs in 1 step
	float step2sec;    // constant
	int8_t dir;        // direction of rotation (+- 1)

	/*  driver connection  */

	GpioPin step_pin;
	GpioPin dir_pin;
	GpioPin en_pin;

	/*  polarity settings  */

	int8_t dir_positive;
	int8_t dir_inverse;

	uint8_t en_on;
	uint8_t en_off;

	/*  finite automata stuf  */

	uint8_t id;
	uint8_t status;
}StepperState;

StepperState stepperInit(GpioPin STEP_Pin, GpioPin DIR_Pin, GpioPin EN_Pin);

void doStep(StepperState* motor);
void releaseStep(StepperState* motor);
void setVel(StepperState* motor, float velocity);
void setAcc(StepperState* motor, float acceleration);
void updateStep2Sec(StepperState* motor);

void setDirectionPolarity(StepperState* motor, int8_t a); // Set pin state corresponding to positive direction
void setStepPinPolarity(StepperState* motor, uint8_t a);  // HIGH -> positive pulses, LOW -> negative pulses
void limitAcceleration(StepperState* motor, float a);
void limitSpeed(StepperState* motor, float v);

void setHome(StepperState* motor, int32_t position);
void saveHome(StepperState* motor);
void calculateHome(StepperState* motor);

void setMinLimit(StepperState* motor, int32_t minSteps);
void setMaxLimit(StepperState* motor, int32_t maxSteps);
void saveMinLimit(StepperState* motor);
void saveMaxLimit(StepperState* motor);

void setGoal(StepperState* motor, int32_t position);
int32_t getGoalErr(StepperState* motor);

#endif /* SRC_STEPPER_LIB_STEPPER_STEPPER_H_ */
