#include "altera_conf.h"
#include "m2_modem.h"
#include "FreeRTOS.h"
#include "task.h"

extern const uint8_t altera_conf_image[];
extern const uint32_t altera_conf_image_length;

extern SPI_HandleTypeDef hspi1;

void AlteraConf_Init(void)
{
	/*
		��������� SPI 8 ��� , CPHA = 0, CPOL = 0
	*/
}

HAL_StatusTypeDef  AlteraConf_ConfigureFPGA(void)
{
		HAL_StatusTypeDef errorcode = HAL_OK;
		ALTCONF_NCONFIG_RESET;
		vTaskDelay(1);
		
		if(ALTCONF_NSTATUS != GPIO_PIN_RESET)
		{
				return HAL_ERROR;
		}
			
		ALTCONF_NCONFIG_SET;
		
	/*
	���� ���������� � ������������
	�������� ������� ������
	*/
		while(ALTCONF_NSTATUS == GPIO_PIN_RESET);
		
		errorcode = HAL_SPI_Transmit(&M2_SPI, (uint8_t *)altera_conf_image, altera_conf_image_length, 2000);
		
		if(errorcode != HAL_OK)
		{
				return HAL_ERROR;
		}
		
		if(ALTCONF_CONF_DONE != GPIO_PIN_SET)
		{
				return HAL_ERROR;
		}
		
		errorcode = HAL_SPI_Transmit(&M2_SPI, (uint8_t *)altera_conf_image, 1, 10);
		
		if(errorcode != HAL_OK)
		{
				return HAL_ERROR;
		}
		
		vTaskDelay(1);
}