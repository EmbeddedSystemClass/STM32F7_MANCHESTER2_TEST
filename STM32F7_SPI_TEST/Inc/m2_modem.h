#ifndef M2_MODEM_H
#define M2_MODEM_H
#include "stm32f7xx_hal.h"

#define M2_IF_RX 	M2_DEVICE_IF_1_RX//M2_DEVICE_IF_0_RX
#define M2_IF_TX	M2_DEVICE_IF_1_TX//M2_DEVICE_IF_0_TX

#define READY_IF0			HAL_GPIO_ReadPin(READY_0_GPIO_Port, READY_0_Pin);
#define READY_IF1			HAL_GPIO_ReadPin(READY_1_GPIO_Port, READY_1_Pin);

#define LOAD_RESET	  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);
#define LOAD_SET	    HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);

#define STROB_RESET	  HAL_GPIO_WritePin(STROB_GPIO_Port, STROB_Pin, GPIO_PIN_RESET);
#define STROB_SET	    HAL_GPIO_WritePin(STROB_GPIO_Port, STROB_Pin, GPIO_PIN_SET);

#define  M2_SPI	 hspi1

typedef enum
{
	M2_DEVICE_IF_0_TX 			=1,
	M2_DEVICE_IF_0_RX 			=2,
	M2_DEVICE_IF_1_TX 			=3,
	M2_DEVICE_IF_1_RX 			=4,
	M2_DEVICE_IF_0_SET_ADDR =5,
	M2_DEVICE_IF_1_SET_ADDR =6,	
	M2_DEVICE_CRC_REG 			=7,
	M2_DEVICE_SET_REG 			=8,		
} enM2DeviceCS;

typedef enum
{
	M2_ECHO_OFF = 0,
	M2_ECHO_ON,
}enM2Echo;

#pragma pack(push, 1)
typedef struct
{
	uint8_t flags;
	uint16_t m2Word;
} stM2Word;

typedef union
{
	stM2Word word;
	uint8_t  buf[3];
} unM2Word;
#pragma pack(pop)

void 		 M2_Modem_Init(void);
void 		 M2_Modem_SelectDevice(enM2DeviceCS dev);
void		 M2_Modem_SendBuf(enM2DeviceCS dev, uint16_t *buf, uint16_t len);
int8_t 	 M2_Modem_ReceiveBuf(enM2DeviceCS dev, unM2Word *wordBuf, uint16_t len, uint16_t *recv, uint16_t timeout);
void 		 M2_Modem_RecvAndSendEcho(enM2DeviceCS devRx, enM2DeviceCS devTx);
int8_t 	 M2_Modem_SendAndRecvEcho(enM2DeviceCS devTx, enM2DeviceCS devRx, uint16_t *sndBuf, uint16_t sndLen, uint16_t *rcvBuf, uint16_t *rcvLen, uint32_t timeout);
void 		 M2_Modem_EchoState(enM2Echo state);
void		 M2_Modem_SetControlReg(uint8_t reg);


uint16_t crc16_CCITT(const uint8_t* data_p, uint8_t length);

#endif