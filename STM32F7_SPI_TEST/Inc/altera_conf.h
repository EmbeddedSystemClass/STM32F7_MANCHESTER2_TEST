#ifndef ALTERA_CONF_H
#define ALTERA_CONF_H
#include "stm32f7xx_hal.h"


#define ALTCONF_WAIT_USERMODE			1//mS

#define ALTCONF_NCONFIG_SET		HAL_GPIO_WritePin(nCONFIG_GPIO_Port, nCONFIG_Pin, GPIO_PIN_SET)
#define ALTCONF_NCONFIG_RESET	HAL_GPIO_WritePin(nCONFIG_GPIO_Port, nCONFIG_Pin, GPIO_PIN_RESET)
#define ALTCONF_NSTATUS				HAL_GPIO_ReadPin(nSTATUS_GPIO_Port, nSTATUS_Pin)
#define ALTCONF_CONF_DONE		HAL_GPIO_ReadPin(CONF_DONE_GPIO_Port, CONF_DONE_Pin)



void AlteraConf_Init(void);
HAL_StatusTypeDef AlteraConf_ConfigureFPGA(void);

#endif