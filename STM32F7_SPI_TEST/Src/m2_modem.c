#include "m2_modem.h"
extern SPI_HandleTypeDef hspi1;

#define  M2_SPI	 hspi1

static void delay_us(uint32_t time);

static void delay_us(uint32_t time)
{
		while (time--);
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
			STROB_SET;
			delay_us(5);
			STROB_RESET;			
	}
	
	LOAD_RESET;
}

uint16_t M2_Modem_ReceiveBuf(enM2DeviceCS dev, uint8_t *buf, uint16_t len, uint16_t timeout)
{
	uint16_t bufCnt=0;
	M2_Modem_SelectDevice(dev);
	
	for(bufCnt = 0; bufCnt < len;  bufCnt++)
	{
			STROB_SET;
			delay_us(5);
			STROB_RESET;	
		
			HAL_SPI_Receive(&M2_SPI, &buf[bufCnt], 1, 10);
		
	}
	
	LOAD_RESET;
	
	return 0;
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