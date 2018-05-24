#include "adis_accel.h"
#include "spi.h"

void ADIS_Accel_Init(void)
{
	HAL_GPIO_WritePin(ADIS_TCS_GPIO_Port, ADIS_TCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(ADIS_ST_GPIO_Port, ADIS_ST_Pin, GPIO_PIN_RESET);
}

void ADIS_Accel_SetTestMode(uint8_t test)
{
	if(test)
	{
			HAL_GPIO_WritePin(ADIS_ST_GPIO_Port, ADIS_ST_Pin, GPIO_PIN_SET);
	}
	else
	{
			HAL_GPIO_WritePin(ADIS_ST_GPIO_Port, ADIS_ST_Pin, GPIO_PIN_RESET);
	}
}

uint16_t ADIS_Accel_Read(enADISAxis axis)
{
	 uint16_t configReg = 0;
	 uint16_t resultAccel = 0;
	
	 if(axis == ADIS_AXIS_X)
	 {
			configReg = ADIS_CONFIG_REG_MASK;
	 }
	 else
	 {
		  configReg = (ADIS_CONFIG_REG_MASK |(1 << ADIS_CONFIG_AXIS_BIT));
	 }
	 HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_RESET);	
	 
	 HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&configReg, (uint8_t*)&resultAccel, 1, 10);
	 
	 HAL_GPIO_WritePin(ADIS_CS_GPIO_Port, ADIS_CS_Pin, GPIO_PIN_SET);	
	 
	 return resultAccel;
}