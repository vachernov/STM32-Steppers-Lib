/*
 * config.c
 *
 *      Author: Valerii Chernov
 */

#include "config.h"

// has to be rewritten
const StepperConfig SetupConfig[STEPPER_UNITS] =
{
	{
		{GPIOA, GPIOA, GPIOA, GPIOA},
		{GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8},
		2038
	}
};

void createGpioPin(GpioPin* temp, GPIO_TypeDef* port, uint16_t pin){
	temp -> port = port;
	temp -> pin = pin;
}
