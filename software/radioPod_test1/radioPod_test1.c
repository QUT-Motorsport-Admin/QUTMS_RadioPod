/*
 * radioPod_test1.c
 *
 * Created: 02-Feb-16 6:30:39 PM
 *  Author: ant
 */ 


#define F_CPU 32000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "usart_driver.h"

int init(void);

USART_data_t BLUETOOTH_usart_data;
USART_data_t GPS_usart_data;

int main(void)
{
	init();
	
    while(1)
    {
		/*
        _delay_ms(333);
		PORTK_OUT = 4;
		_delay_ms(333);
		PORTK_OUT = 8;
		_delay_ms(333);
		PORTK_OUT = 2;
		*/
		
		if(USART_RXBufferData_Available(&GPS_usart_data))
		{
			unsigned char temp_byte =  USART_RXBuffer_GetByte(&GPS_usart_data);
			USART_TXBuffer_PutByte(&BLUETOOTH_usart_data, temp_byte);
			if(temp_byte == '\n') PORTK_OUT ^= 2;
		}
    }
}

ISR(USARTF0_RXC_vect)		//receive interrupt routine for the bluetooth module
{
	USART_RXComplete(&BLUETOOTH_usart_data);
}

ISR(USARTF0_DRE_vect)		//transmit interrupt routine for the bluetooth module
{
	USART_DataRegEmpty(&BLUETOOTH_usart_data);
}

ISR(USARTD1_RXC_vect)		//receive interrupt routine for the GPS module
{
	USART_RXComplete(&GPS_usart_data);
}

ISR(USARTD1_DRE_vect)		//transmit interrupt routine for the GPS module
{
	USART_DataRegEmpty(&GPS_usart_data);
}