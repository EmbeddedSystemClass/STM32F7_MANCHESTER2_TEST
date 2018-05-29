/*
    FreeRTOS V7.3.0 - Copyright (C) 2012 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/
#include "stm32f7xx.h"
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

#include "m2_modem.h"

extern enM2DeviceCS rxIface;
extern enM2DeviceCS txIface;

/*
 * Implements the echo-three-parameters command.
 */
static portBASE_TYPE prvThreeParameterEchoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/*
 * Implements the echo-parameters command.
 */
static portBASE_TYPE prvMultiParameterEchoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

static portBASE_TYPE prvM2SendWord( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvM2SendEcho( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvM2SendEchoCRC( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvM2EchoMode( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvM2Interface( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvM2ControlReg( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );



/* Structure that defines the "echo_3_parameters" command line command.  This
takes exactly three parameters that the command simply echos back one at a
time. */
static const CLI_Command_Definition_t prvThreeParameterEchoCommandDefinition =
{
	( const int8_t * const ) "echo-3-parameters",
	( const int8_t * const ) "echo-3-parameters <param1> <param2> <param3>:\r\n Expects three parameters, echos each in turn\r\n\r\n",
	prvThreeParameterEchoCommand, /* The function to run. */
	3 /* Three parameters are expected, which can take any value. */
};

/* Structure that defines the "echo_parameters" command line command.  This
takes a variable number of parameters that the command simply echos back one at
a time. */
static const CLI_Command_Definition_t prvMultiParameterEchoCommandDefinition =
{
	( const int8_t * const ) "echo-parameters",
	( const int8_t * const ) "echo-parameters <...>:\r\n Take variable number of parameters, echos each in turn\r\n\r\n",
	prvMultiParameterEchoCommand, /* The function to run. */
	-1 /* The user can enter any number of commands. */
};

static const CLI_Command_Definition_t prvM2SendWordDefinition =
{
	( const int8_t * const ) "m2-send-word",
	( const int8_t * const ) "m2-send-word <word>:\r\nSend word \r\n\r\n",
	prvM2SendWord, /* The function to run. */
	1 /* Three parameters are expected, which can take any value. */
};

static const CLI_Command_Definition_t prvM2EchoModeDefinition =
{
	( const int8_t * const ) "m2-echo-mode",
	( const int8_t * const ) "m2-echo-mode <on/off>:\r\n Expects 1 parameter\r\n\r\n",
	prvM2EchoMode, /* The function to run. */
	1 /* Three parameters are expected, which can take any value. */
};

static const CLI_Command_Definition_t prvM2InterfaceDefinition =
{
	( const int8_t * const ) "m2-interface",
	( const int8_t * const ) "m2-interface <0/1>:\r\n Expects 1 parameter\r\n\r\n",
	prvM2Interface, /* The function to run. */
	1 /* Three parameters are expected, which can take any value. */
};

static const CLI_Command_Definition_t prvM2ControlRegDefinition =
{
	( const int8_t * const ) "m2-set-control-reg",
	( const int8_t * const ) "m2-set-control-reg <byte>:\r\n Expects 1 parameter\r\n\r\n",
	prvM2ControlReg, /* The function to run. */
	1 /* Three parameters are expected, which can take any value. */
};


static const CLI_Command_Definition_t prvM2SendEchoDefinition =
{
	( const int8_t * const ) "m2-send-echo",
	( const int8_t * const ) "m2-send-echo <...>:\r\n Take variable number of parameters, echos each in turn\r\n\r\n",
	prvM2SendEcho, /* The function to run. */
	-1 /* The user can enter any number of commands. */
};

static const CLI_Command_Definition_t prvM2SendEchoCRCDefinition =
{
	( const int8_t * const ) "m2-send-echo-crc",
	( const int8_t * const ) "m2-send-echo-crc <...>:\r\n Take variable number of parameters, echos each in turn\r\n\r\n",
	prvM2SendEchoCRC, /* The function to run. */
	-1 /* The user can enter any number of commands. */
};

/*-----------------------------------------------------------*/

void vRegisterCLICommands( void )
{
	/* Register all the command line commands defined immediately above. */

	FreeRTOS_CLIRegisterCommand( &prvThreeParameterEchoCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &prvMultiParameterEchoCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &prvM2SendWordDefinition );
	FreeRTOS_CLIRegisterCommand( &prvM2SendEchoDefinition );
	FreeRTOS_CLIRegisterCommand( &prvM2SendEchoCRCDefinition );
	FreeRTOS_CLIRegisterCommand( &prvM2EchoModeDefinition );
	FreeRTOS_CLIRegisterCommand( &prvM2InterfaceDefinition );
	FreeRTOS_CLIRegisterCommand( &prvM2ControlRegDefinition );

}

/*-----------------------------------------------------------*/

static portBASE_TYPE prvThreeParameterEchoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
int8_t *pcParameterString;
portBASE_TYPE xParameterStringLength, xReturn;
static portBASE_TYPE xParameterNumber = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( xParameterNumber == 0 )
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( ( char * ) pcWriteBuffer, "The three parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		xParameterNumber = 1L;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										xParameterNumber,		/* Return the next parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

		/* Sanity check something was returned. */
		configASSERT( pcParameterString );

		/* Return the parameter string. */
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		sprintf( ( char * ) pcWriteBuffer, "%d: ", xParameterNumber );
		strncat( ( char * ) pcWriteBuffer, ( const char * ) pcParameterString, xParameterStringLength );
		strncat( ( char * ) pcWriteBuffer, "\r\n", strlen( "\r\n" ) );

		/* If this is the last of the three parameters then there are no more
		strings to return after this one. */
		if( xParameterNumber == 3L )
		{
			/* If this is the last of the three parameters then there are no more
			strings to return after this one. */
			xReturn = pdFALSE;
			xParameterNumber = 0L;
		}
		else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			xParameterNumber++;
		}
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

static portBASE_TYPE prvMultiParameterEchoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
int8_t *pcParameterString;
portBASE_TYPE xParameterStringLength, xReturn;
static portBASE_TYPE xParameterNumber = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( xParameterNumber == 0 )
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( ( char * ) pcWriteBuffer, "The parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		xParameterNumber = 1L;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										xParameterNumber,		/* Return the next parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

		if( pcParameterString != NULL )
		{
			/* Return the parameter string. */
			memset( pcWriteBuffer, 0x00, xWriteBufferLen );
			sprintf( ( char * ) pcWriteBuffer, "%d: ", xParameterNumber );
			strncat( ( char * ) pcWriteBuffer, ( const char * ) pcParameterString, xParameterStringLength );
			strncat( ( char * ) pcWriteBuffer, "\r\n", strlen( "\r\n" ) );

			/* There might be more parameters to return after this one. */
			xReturn = pdTRUE;
			xParameterNumber++;
		}
		else
		{
			/* No more parameters were found.  Make sure the write buffer does
			not contain a valid string. */
			pcWriteBuffer[ 0 ] = 0x00;

			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			xParameterNumber = 0;
		}
	}

	return xReturn;
}


static portBASE_TYPE prvM2SendWord( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
int8_t *pcParameterString;
portBASE_TYPE xParameterStringLength, xReturn;
static portBASE_TYPE xParameterNumber = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( xParameterNumber == 0 )
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( ( char * ) pcWriteBuffer, "The three parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		xParameterNumber = 1L;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										xParameterNumber,		/* Return the next parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

		/* Sanity check something was returned. */
		configASSERT( pcParameterString );

		/* Return the parameter string. */
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		sprintf( ( char * ) pcWriteBuffer, "%d: ", xParameterNumber );
		strncat( ( char * ) pcWriteBuffer, ( const char * ) pcParameterString, xParameterStringLength );
		strncat( ( char * ) pcWriteBuffer, "\r\n", strlen( "\r\n" ) );

		/* If this is the last of the three parameters then there are no more
		strings to return after this one. */

			/* If this is the last of the three parameters then there are no more
			strings to return after this one. */
			xReturn = pdFALSE;
			xParameterNumber = 0L;
	}

	return xReturn;
}


static portBASE_TYPE prvM2SendEcho( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
int8_t *pcParameterString;
portBASE_TYPE xParameterStringLength, xReturn;
static portBASE_TYPE xParameterNumber = 0;
	
static uint16_t m2WordCnt = 0;
static uint16_t m2WordBuf[1000];
uint8_t 	tempBuf[32];
int8_t ret;
uint16_t rcvLen;
uint16_t cnt;


	if( xParameterNumber == 0 )
	{
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		m2WordCnt = 0;
		xParameterNumber = 1L;
		xReturn = pdPASS;
	}
	else
	{
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										xParameterNumber,		/* Return the next parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		
		if( pcParameterString != NULL )
		{
			
			int cnv;
			uint16_t wordTmp;
			cnv = sscanf(pcParameterString,"%x",&wordTmp); 
			
			if((xParameterStringLength == 4) && (cnv == 1))
			{				
					m2WordBuf[m2WordCnt] = wordTmp;
				  m2WordCnt++;
					/* There might be more parameters to return after this one. */
					xReturn = pdTRUE;
					xParameterNumber++;
			}
			else
			{
				 sprintf( ( char * ) pcWriteBuffer, "Param error\r\n");
				 xReturn = pdFALSE;
				 pcParameterString = NULL;
				 xParameterNumber = 0;
			}


		}
		else //No more parameters were found.
		{

			//pcWriteBuffer[ 0 ] = 0x00;
			memset( pcWriteBuffer, 0x00, xWriteBufferLen );
			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			xParameterNumber = 0;
			
			/*
			Execute M2_Modem_SendAndRecvEcho
			*/
			

			
			ret = M2_Modem_SendAndRecvEcho(txIface, rxIface, m2WordBuf, m2WordCnt,  m2WordBuf, &rcvLen, 400);
			
			if(ret == 0)
			{				
					for(cnt = 0; cnt < rcvLen; cnt++)
					{
							sprintf( tempBuf, "%X ", m2WordBuf[cnt]);
							strncat( ( char * ) pcWriteBuffer, tempBuf, strlen(tempBuf) );
					}
					strncat( ( char * ) pcWriteBuffer, "\r\n", strlen( "\r\n" ) );
			}
			else
			{
					sprintf( ( char * ) pcWriteBuffer, "Echo error\r\n");
			}
		}
	}

	return xReturn;
}


static portBASE_TYPE prvM2SendEchoCRC( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
int8_t *pcParameterString;
portBASE_TYPE xParameterStringLength, xReturn;
static portBASE_TYPE xParameterNumber = 0;
	
static uint16_t m2WordCnt = 0;
static uint16_t m2WordBuf[1000];
uint8_t 	tempBuf[32];
int8_t ret;
uint16_t rcvLen;
uint16_t cnt;
uint16_t sndCRC = 0;


	if( xParameterNumber == 0 )
	{
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		m2WordCnt = 0;
		xParameterNumber = 1L;
		xReturn = pdPASS;
	}
	else
	{
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										xParameterNumber,		/* Return the next parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		
		if( pcParameterString != NULL )
		{
			
			int cnv;
			uint16_t wordTmp;
			cnv = sscanf(pcParameterString,"%x",&wordTmp); 
			
			if((xParameterStringLength == 4) && (cnv == 1))
			{				
					m2WordBuf[m2WordCnt] = wordTmp;
				  m2WordCnt++;
					/* There might be more parameters to return after this one. */
					xReturn = pdTRUE;
					xParameterNumber++;
			}
			else
			{
				 sprintf( ( char * ) pcWriteBuffer, "Param error\r\n");
				 xReturn = pdFALSE;
				 pcParameterString = NULL;
				 xParameterNumber = 0;
			}


		}
		else //No more parameters were found.
		{

			//pcWriteBuffer[ 0 ] = 0x00;
			memset( pcWriteBuffer, 0x00, xWriteBufferLen );
			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			xParameterNumber = 0;
			
			/*
			Execute M2_Modem_SendAndRecvEcho
			*/
			

			
			ret = M2_Modem_SendAndRecvEchoCRC(txIface, rxIface, m2WordBuf, m2WordCnt, &sndCRC, m2WordBuf, &rcvLen, 400);
			
			if(ret == 0)
			{	
					sprintf( tempBuf, "CRC = %X \r\n", sndCRC);
					strncat( ( char * ) pcWriteBuffer, tempBuf, strlen(tempBuf) );
				
					for(cnt = 0; cnt < rcvLen; cnt++)
					{
							sprintf( tempBuf, "%X ", m2WordBuf[cnt]);
							strncat( ( char * ) pcWriteBuffer, tempBuf, strlen(tempBuf) );
					}
					strncat( ( char * ) pcWriteBuffer, "\r\n", strlen( "\r\n" ) );
			}
			else
			{
					sprintf( ( char * ) pcWriteBuffer, "Echo error\r\n");
			}
		}
	}

	return xReturn;
}


static portBASE_TYPE prvM2EchoMode( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
int8_t *pcParameterString;
portBASE_TYPE xParameterStringLength, xReturn;
static portBASE_TYPE xParameterNumber = 0;


	if( xParameterNumber == 0 )
	{
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		xParameterNumber = 1L;
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										xParameterNumber,		/* Return the next parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		
		if(strcmp(pcParameterString, "on") == 0)
		{
				M2_Modem_EchoState(M2_ECHO_ON);
				sprintf( ( char * ) pcWriteBuffer, "Echo ON\r\n");
		}
		else if(strcmp(pcParameterString, "off") == 0)
		{
				M2_Modem_EchoState(M2_ECHO_OFF);
				sprintf( ( char * ) pcWriteBuffer, "Echo OFF\r\n");
		}
		else
		{
				sprintf( ( char * ) pcWriteBuffer, "Param error\r\n");
		}


		xReturn = pdFALSE;
		xParameterNumber = 0L;
	}

	return xReturn;
}

static portBASE_TYPE prvM2Interface( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
int8_t *pcParameterString;
portBASE_TYPE xParameterStringLength, xReturn;
static portBASE_TYPE xParameterNumber = 0;


	if( xParameterNumber == 0 )
	{
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		xParameterNumber = 1L;
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										xParameterNumber,		/* Return the next parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		
		if(strcmp(pcParameterString, "0") == 0)
		{
				M2_Modem_SetInterface(M2_IF_0);
				sprintf( ( char * ) pcWriteBuffer, "IF 0\r\n");
		}
		else if(strcmp(pcParameterString, "1") == 0)
		{
				M2_Modem_SetInterface(M2_IF_1);
				sprintf( ( char * ) pcWriteBuffer, "IF 1\r\n");
		}
		else
		{
				sprintf( ( char * ) pcWriteBuffer, "Param error\r\n");
		}


		xReturn = pdFALSE;
		xParameterNumber = 0L;
	}

	return xReturn;
}


static portBASE_TYPE prvM2ControlReg( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
int8_t *pcParameterString;
portBASE_TYPE xParameterStringLength, xReturn;
static portBASE_TYPE xParameterNumber = 0;


	if( xParameterNumber == 0 )
	{
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		xParameterNumber = 1L;
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										xParameterNumber,		/* Return the next parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		
		if( pcParameterString != NULL )
		{
			
			int cnv;
			uint8_t byteTmp;
			cnv = sscanf(pcParameterString,"%x",&byteTmp); 
			
			if((xParameterStringLength == 2) && (cnv == 1))
			{				
					/* There might be more parameters to return after this one. */
					M2_Modem_SetControlReg(byteTmp);
					sprintf( ( char * ) pcWriteBuffer, "Control reg =  %x\r\n", byteTmp);
					xReturn = pdTRUE;

			}
			else
			{
				 sprintf( ( char * ) pcWriteBuffer, "Param error\r\n");
				 xReturn = pdFALSE;
				 pcParameterString = NULL;
				 xParameterNumber = 0;
			}
		}



		xReturn = pdFALSE;
		xParameterNumber = 0L;
	}

	return xReturn;
}
