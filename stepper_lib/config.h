/*
 * config.h
 *
 *      Author: Valerii
 */

#ifndef SRC_STEPPER_LIB_CONFIG_H_
#define SRC_STEPPER_LIB_CONFIG_H_

#include "stm32f4xx.h"

#define STEPPER_SPEED_TOLERANCE     3.5
#define STEPPER_POSITION_TOLERANCE  2

#define STEPPER_DEFAULT_RES  1.8
#define STEPPER_DEFAULT_V_M  0.225
#define STEPPER_DEFAULT_A_M  10

#define STEPPER_DEFAULT_LIM_MIN -10000
#define STEPPER_DEFAULT_LIM_MAX  10000

#define STEPPER_UNITS 1

// Stepper Timer Base Options
#define STEPPER_TIMER      TIM2
#define STEPPER_TIMER_CLK  84
#define STEPPER_TIME_BASE  1

typedef struct{
    GPIO_TypeDef* port;
    uint16_t       pin;
}GpioPin;

typedef struct{
	GPIO_TypeDef * IN_GPIO[4];
	uint16_t        IN_PIN[4];
	uint16_t    STEPS_PER_REV;
}StepperConfig;

const StepperConfig SetupConfig[STEPPER_UNITS];

void createGpioPin(GpioPin* temp, GPIO_TypeDef* port, uint16_t pin);

#endif /* SRC_STEPPER_LIB_CONFIG_H_ */
