#ifndef ALTERA_CONF_H
#define ALTERA_CONF_H
#include "stm32f7xx_hal.h"


#define ALTCONF_WAIT_USERMODE			1//mS

void AlteraConf_Init(void);
HAL_StatusTypeDef AlteraConf_ConfigureFPGA(void);

#endif