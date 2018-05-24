#ifndef ADIS_ACCEL_H
#define ADIS_ACCEL_H

#include "stm32f7xx_hal.h"

#define ADIS_CONFIG_REG_MASK	0x400
#define ADIS_CONFIG_AXIS_BIT	11

typedef enum
{
	ADIS_AXIS_X = 0,
	ADIS_AXIS_Y
} enADISAxis;

void ADIS_Accel_Init(void);
void ADIS_Accel_SetTestMode(uint8_t test);

uint16_t ADIS_Accel_Read(enADISAxis axis);

#endif