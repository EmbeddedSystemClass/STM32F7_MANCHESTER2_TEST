#include "altera_conf.h"
#include "m2_modem.h"
#include "FreeRTOS.h"
#include "task.h"

extern const unsigned char altera_conf_image[];
extern const unsigned long altera_conf_image_length;

extern SPI_HandleTypeDef hspi1;

void AlteraConf_Init(void)
{
	/*
		Настройка SPI 8 бит , CPHA = 0, CPOL = 0
	*/
}

HAL_StatusTypeDef  AlteraConf_ConfigureFPGA(void)
{
		HAL_StatusTypeDef errorcode = HAL_OK;
		HAL_GPIO_WritePin(nCONFIG_GPIO_Port, nCONFIG_Pin, GPIO_PIN_RESET);
		vTaskDelay(1);
		
		if(HAL_GPIO_ReadPin(nSTATUS_GPIO_Port, nSTATUS_Pin) != GPIO_PIN_RESET)
		{
				return HAL_ERROR;
		}
			
		HAL_GPIO_WritePin(nCONFIG_GPIO_Port, nCONFIG_Pin, GPIO_PIN_SET);
		
	/*
	Ждем готовности к конфигурации
	Добавить таймаут ошибки
	*/
		while(HAL_GPIO_ReadPin(nSTATUS_GPIO_Port, nSTATUS_Pin) == GPIO_PIN_RESET);
		
		errorcode = HAL_SPI_Transmit(&M2_SPI, (uint8_t *)altera_conf_image, altera_conf_image_length, 10);
		
		if(errorcode != HAL_OK)
		{
				return HAL_ERROR;
		}
		
		
		
		errorcode = HAL_SPI_Transmit(&M2_SPI, (uint8_t *)altera_conf_image, 1, 10);
		
		if(errorcode != HAL_OK)
		{
				return HAL_ERROR;
		}
}