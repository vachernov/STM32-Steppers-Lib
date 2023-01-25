/*
 * hbot.h
 *
 *      Author: Valerii
 */

#ifndef SRC_STEPPER_LIB_UTILS_HBOT_H_
#define SRC_STEPPER_LIB_UTILS_HBOT_H_

#include "stm32f4xx.h"
#include "../config.h"
#include "../steppers/steppers_control.h"

#define HBOT_DEFAULT_RD  0.75 // this have to be changed

#define HBOT_DEFAULT_X_ID  1
#define HBOT_DEFAULT_Y_ID  2
#define HBOT_DEFAULT_Z_ID  3

#define HBOT_DEFAULT_X_POS 0
#define HBOT_DEFAULT_Y_POS 0
#define HBOT_DEFAULT_Z_POS 0

#define HBOT_DEFAULT_X_VEL -15
#define HBOT_DEFAULT_Y_VEL -15
#define HBOT_DEFAULT_Z_VEL -10.5

#define HBOT_DEFAULT_DT_INIT 50

typedef struct{
	/* Stepper indexes */
	uint8_t motor_x_id;
	uint8_t motor_y_id;
	uint8_t motor_z_id;

	/* Task space vector */
	int32_t x;
	int32_t y;
	int32_t z;

	/* Configuration space vector */
	int32_t q_1;
    int32_t q_2;
    int32_t q_3;

    /* Machine Limit Switches */
    GpioPin x_min_limit;
    GpioPin x_max_limit;
    GpioPin y_min_limit;
    GpioPin y_max_limit;
    GpioPin z_min_limit;
    GpioPin z_max_limit;

    /* --- Private Variable Zone (Please mind it) --- */

    /* Kinematics constants */
    float r_d;
    float r_d_inv;

    uint8_t is_initialized;
}HBotState;

/* Prototypes For All Functions  */

void directSpaceTransform(HBotState* machine, int32_t q_1, int32_t q_2, int32_t q_3);
void directSpaceTransform(HBotState* machine, int32_t x, int32_t y, int32_t z);

HBotState hbotInit(StepperState* motorX, StepperState* motorY, StepperState* motorZ,
			  	   GpioPin switchXmin,  GpioPin switchXmax,
				   GpioPin switchYmin,  GpioPin switchYmax,
				   GpioPin switchZmin,  GpioPin switchZmax,
				   TIM_HandleTypeDef* TMR_Handle);

void hbotFindHome(HBotState* machine);
void hbotGoToHome(HBotState* machine);

void hbotGoToPoint(HBotState* machine, int32_t x, int32_t y, int32_t z);
void hbotSetSpeed(HBotState* machine, int32_t v_x, int32_t v_y, int32_t v_z);

uint8_t hbotGetXMinSwitchState(HBotState* machine);
uint8_t hbotGetXMaxSwitchState(HBotState* machine);
uint8_t hbotGetYMinSwitchState(HBotState* machine);
uint8_t hbotGetYMaxSwitchState(HBotState* machine);
uint8_t hbotGetZMinSwitchState(HBotState* machine);
uint8_t hbotGetZMaxSwitchState(HBotState* machine);

uint32_t hbotGetL1PositionError(HBotState* machine);

#endif /* SRC_STEPPER_LIB_UTILS_HBOT_H_ */
