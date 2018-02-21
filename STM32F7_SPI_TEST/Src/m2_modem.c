#include "m2_modem.h"
#include "cmsis_os.h"


extern SPI_HandleTypeDef hspi1;



static enM2Echo echoState;

static void delay_us(uint32_t time);
uint8_t M2_Modem_RxFIFO_NotEmpty(enM2DeviceCS dev);
void M2_Modem_Task(void const * argument);

static void delay_us(uint32_t time)
{
		while (time--);
}

#define M2_ADDR 	0x7860
void M2_Modem_Init(void)
{
  	uint16_t  m2Addr = M2_ADDR;
//		M2_Modem_SendBuf(M2_DEVICE_IF_0_SET_ADDR, &m2Addr, 1);
		
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }	
	
		osThreadDef(M2_Task, M2_Modem_Task, osPriorityNormal, 0, 256);
		osThreadCreate(osThread(M2_Task), NULL);		
}

uint8_t M2_Modem_RxFIFO_NotEmpty(enM2DeviceCS dev)
{
		switch(dev)
		{
			case M2_DEVICE_IF_0_RX:
			{
					return  HAL_GPIO_ReadPin(READY_0_GPIO_Port, READY_0_Pin);
			}
			break;
			
			case M2_DEVICE_IF_1_RX:
			{
					return  HAL_GPIO_ReadPin(READY_1_GPIO_Port, READY_1_Pin);
			}
			break;

			default:
			{
					return 0;
			}
			break;			
		}
}

void 		 M2_Modem_SelectDevice(enM2DeviceCS dev)
{
	LOAD_RESET;
	STROB_RESET;
	uint8_t devNum =(uint8_t)dev;
	HAL_SPI_Transmit(&M2_SPI, &devNum, 1, 10);
	LOAD_SET;
}

void		 M2_Modem_SendBuf(enM2DeviceCS dev, uint16_t *buf, uint16_t len)
{
	uint16_t bufCnt=0;
	M2_Modem_SelectDevice(dev);
	
	for(bufCnt = 0; bufCnt < (len*2);  bufCnt++)
	{
			HAL_SPI_Transmit(&M2_SPI, &(((uint8_t*)buf)[bufCnt]), 1, 10);
			delay_us(5);
			STROB_SET;
			delay_us(5);
			STROB_RESET;			
	}
	
	LOAD_RESET;
}

//uint16_t M2_Modem_ReceiveBuf(enM2DeviceCS dev, uint8_t *buf, uint16_t len, uint16_t timeout)
//{
//	uint16_t bufCnt=0;
//	M2_Modem_SelectDevice(dev);
//	
//	for(bufCnt = 0; bufCnt < len;  bufCnt++)
//	{
//			STROB_SET;
//			delay_us(5);
//			STROB_RESET;	
//		
//			HAL_SPI_Receive(&M2_SPI, &buf[bufCnt], 1, 10);
//		
//	}
//	
//	LOAD_RESET;
//	
//	return 0;
//}



int8_t M2_Modem_ReceiveBuf(enM2DeviceCS dev, unM2Word *wordBuf, uint16_t len, uint16_t *recv, uint16_t timeout)
{
		uint32_t tickstart = 0U;
		uint16_t recvCnt = 0;
		tickstart = HAL_GetTick();
		M2_Modem_SelectDevice(dev);
		
	 
		while(recvCnt < len)
		{
				if(M2_Modem_RxFIFO_NotEmpty(dev))
				{
						
						tickstart = HAL_GetTick();
						uint8_t byteCnt = 0;
					
						for(byteCnt = 0; byteCnt < 3; byteCnt ++)
						{
								STROB_SET;
								delay_us(5);
								STROB_RESET;	
							
								HAL_SPI_Receive(&M2_SPI, &wordBuf[recvCnt].buf[byteCnt], 1, 10);							
						}
																	
						recvCnt ++;
				}
			
				if ((timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >=  timeout))
				{
						LOAD_RESET;
						*recv = recvCnt;
						return HAL_TIMEOUT;
				}
		} 
		
		LOAD_RESET;
		*recv = recvCnt;
		return HAL_OK;
}

void 		 M2_Modem_FlushRxFIFO(enM2DeviceCS dev)
{
		M2_Modem_SelectDevice(dev);
		LOAD_RESET;
}

#define M2_RECV_TIMEOUT					100
#define M2_MAX_RECV_MAX_LEN			100

unM2Word rcvWordBuf[M2_MAX_RECV_MAX_LEN];
void 		 M2_Modem_RecvAndSendEcho(enM2DeviceCS devRx, enM2DeviceCS devTx)
{
		
		uint16_t sndBuf[M2_MAX_RECV_MAX_LEN];
		uint16_t rcvLen;
		int8_t ret;
		ret = M2_Modem_ReceiveBuf(devRx, rcvWordBuf, M2_MAX_RECV_MAX_LEN, &rcvLen, M2_RECV_TIMEOUT);
		if((ret == HAL_OK) || (ret == HAL_TIMEOUT))
		{
				uint16_t cnt = 0;
				
				for(cnt = 0; cnt < rcvLen; cnt++)
				{
						sndBuf[cnt] = rcvWordBuf[cnt].word.m2Word;
				}
				M2_Modem_SendBuf(devTx, sndBuf, rcvLen);
		}	
}


int8_t 		 M2_Modem_SendAndRecvEcho(enM2DeviceCS devTx, enM2DeviceCS devRx, uint16_t *sndBuf, uint16_t sndLen, uint16_t *rcvBuf, uint16_t *rcvLen, uint32_t timeout)
{
//		unM2Word rcvWordBuf[32];
		uint16_t rcvWordLen = 0;

		int8_t	 err;
	
		M2_Modem_FlushRxFIFO(devRx);
		M2_Modem_SendBuf(devTx, sndBuf, sndLen);
	
		err = M2_Modem_ReceiveBuf(devRx, rcvWordBuf, sndLen, &rcvWordLen, timeout);
	
		if((err == HAL_OK) || (err == HAL_TIMEOUT) )
		{
				uint16_t cnt = 0;
				
				*rcvLen = rcvWordLen;
				for(cnt = 0; cnt < rcvWordLen; cnt++)
				{
						rcvBuf[cnt] = rcvWordBuf[cnt].word.m2Word;
				}
				
				return 0;
		}
		else
		{
				return -1;
		}
}

void 		 M2_Modem_EchoState(enM2Echo state)
{
		echoState = state;
}

void M2_Modem_Task(void const * argument)
{
	while(1)
	{
		 if((echoState == M2_ECHO_ON) && M2_Modem_RxFIFO_NotEmpty(M2_DEVICE_IF_0_RX))
		 {
				M2_Modem_RecvAndSendEcho(M2_DEVICE_IF_0_RX, M2_DEVICE_IF_0_TX);
		 }
	}
}

uint16_t crc16_CCITT(const uint8_t* data_p, uint8_t length)
{
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (length--)
		{
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}