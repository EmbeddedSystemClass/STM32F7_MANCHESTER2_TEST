#ifndef M2_MODEM_H
#define M2_MODEM_H
#include "stm32f7xx_hal.h"

#define READY_IF0			HAL_GPIO_ReadPin(READY_0_GPIO_Port, READY_0_Pin);
#define READY_IF1			HAL_GPIO_ReadPin(READY_1_GPIO_Port, READY_1_Pin);

#define LOAD_RESET	  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);
#define LOAD_SET	    HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);

#define STROB_RESET	  HAL_GPIO_WritePin(STROB_GPIO_Port, STROB_Pin, GPIO_PIN_RESET);
#define STROB_SET	    HAL_GPIO_WritePin(STROB_GPIO_Port, STROB_Pin, GPIO_PIN_SET);

typedef enum
{
	M2_DEVICE_IF_0_TX 			=1,
	M2_DEVICE_IF_0_RX 			=2,
	M2_DEVICE_IF_1_TX 			=3,
	M2_DEVICE_IF_1_RX 			=4,
	M2_DEVICE_IF_0_SET_ADDR =5,
	M2_DEVICE_IF_1_SET_ADDR =6,	
	M2_DEVICE_CRC_REG 			=7,	
} enM2DeviceCS;


void 		 M2_Modem_SelectDevice(enM2DeviceCS dev);
void		 M2_Modem_SendBuf(enM2DeviceCS dev, uint16_t *buf, uint16_t len);
uint16_t M2_Modem_ReceiveBuf(enM2DeviceCS dev, uint8_t *buf, uint16_t len, uint16_t timeout);
uint16_t crc16_CCITT(const uint8_t* data_p, uint8_t length);

#endif