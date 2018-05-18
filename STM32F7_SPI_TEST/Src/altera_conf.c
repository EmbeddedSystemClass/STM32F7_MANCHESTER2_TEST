#include "altera_conf.h"
#include "m2_modem.h"
#include "FreeRTOS.h"
#include "task.h"

extern const uint8_t altera_conf_image[];
extern const uint32_t altera_conf_image_length;

extern SPI_HandleTypeDef M2_SPI;

void AlteraConf_Init(void)
{
	/*
		Настройка SPI 8 бит , CPHA = 0, CPOL = 0
	*/

  M2_SPI.Init.FirstBit = SPI_FIRSTBIT_LSB;

  if (HAL_SPI_Init(&M2_SPI) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }	
}

HAL_StatusTypeDef  AlteraConf_ConfigureFPGA(void)
{
		HAL_StatusTypeDef errorcode = HAL_OK;
		uint32_t tickstart = 0U;
		uint32_t altera_conf_image_cnt = altera_conf_image_length;
		uint8_t *altera_conf_image_ptr =(uint8_t*) altera_conf_image;
		ALTCONF_NCONFIG_RESET;
		vTaskDelay(1);
		
		if(ALTCONF_NSTATUS != GPIO_PIN_RESET)
		{
				return HAL_ERROR;
		}
			
		ALTCONF_NCONFIG_SET;
		
	/*
	Ждем готовности к конфигурации
	Добавить таймаут ошибки
	*/
		tickstart = HAL_GetTick();
		
		while(ALTCONF_NSTATUS == GPIO_PIN_RESET)
		{
				if ((HAL_GetTick() - tickstart) >=  ALTCONF_NSTATUS_TIMEOUT)
				{
						return HAL_TIMEOUT;
				}				
		}
		
		/*
			Передача пакета по HAL_SPI_Transmit ограничена 65536, по этому передаем частями
		*/
		
		while(altera_conf_image_cnt)
		{
			if(altera_conf_image_cnt > 0xFFFF)
			{
					errorcode = HAL_SPI_Transmit(&M2_SPI, altera_conf_image_ptr, 0xFFFF, 2000);
					altera_conf_image_cnt -= 0xFFFF;
					altera_conf_image_ptr += 0xFFFF;
			}
			else
			{
					errorcode = HAL_SPI_Transmit(&M2_SPI, altera_conf_image_ptr, altera_conf_image_cnt, 2000);
					altera_conf_image_cnt = 0x0;	
			}
			
			if(errorcode != HAL_OK)
			{
					return HAL_ERROR;
			}
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
		
		HAL_GPIO_WritePin(FPGA_RST_GPIO_Port, FPGA_RST_Pin, GPIO_PIN_SET);
		
		return HAL_OK;
}