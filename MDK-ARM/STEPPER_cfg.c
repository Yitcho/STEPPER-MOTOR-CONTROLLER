

#include "STEPPER.h"

const STEPPER_CfgType STEPPER_CfgParam[STEPPER_UNITS] =
{
	// Stepper Motor 1 Configurations
    {
      {GPIOE, GPIOE, GPIOD, GPIOD},
    	{GPIO_PIN_7, GPIO_PIN_15, GPIO_PIN_10, GPIO_PIN_11},
		2038,
		STEPPER_UNIPOLAR,
		WAVE_DRIVE
	}
};
