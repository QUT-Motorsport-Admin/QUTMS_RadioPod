#include <avr/io.h>
#include <stdbool.h>
#include "usart_driver.h"

USART_data_t BLUETOOTH_usart_data;		//struct to manage the bluetooth usart
USART_data_t GPS_usart_data;			//struct to manage the GPS usart



int init(void)
{
	int error = 128;

	///////////////////////////////////////////////////////////////////////////////////////////////
	//set up clock options
	OSC.XOSCCTRL = 0b11011011;				// set for 16MHz xtal, with longest startup time
	OSC.CTRL |= 8;							// enable the external oscillator
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));	// wait for oscillator to be ready
	OSC.PLLCTRL =  0b11000010;				// PLL source is external OSC, multiple is 2x (32MHz)
	OSC.CTRL |= 16;							// enable the PLL unit
	while(!(OSC.STATUS & OSC_PLLRDY_bm));	// wait for PLL unit to stabilise

	CCP = CCP_IOREG_gc;						// disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;			// switch clock source to the PLL

	///////////////////////////////////////////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////////////////////////////////////////////
	//Set GPIO

	PORTF_DIR |= 9;		//set PORTF pin 3 as output for serial 1 TX and PF) which is the CTS line
	PORTF_DIR &= ~4;	//set PORTF pin 2 as an input for serial 1 RX
	
	PORTD_DIR |= 128;	//set PORTD pin 7 as output for serial 2 TX
	PORTD_DIR &= ~64;	//set PORTD pin 6 as an input for serial 2 RX
	
	PORTF_OUT &= ~1;	//set pin PF0 low so the CTS line is enabled for sending
	///////////////////////////////////////////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////////////////////////////////////////////
	//configure ADC
	ADCA_CTRLB |= 0b00000000;		//12-bit right hand result mode
	ADCA_CH0_MUXCTRL |= 0b01010000;	//set the MUX to input 10 position (PIN7 on qfp100)
	ADCA_CH0_CTRL = 0b00000001;		//single ended conversion on channel A
	ADCA_REFCTRL |= 0b00110000;		//enable the reference on AREF_B
	ADCA_PRESCALER |= 2;			//turn on /16 mode for ADC clock
	ADCA_CTRLA |= 1;				//enable ADC module A
	///////////////////////////////////////////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////////////////////////////////////////////
	//configure USART for bluetooth
	/* Use USARTF0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&BLUETOOTH_usart_data, &USARTF0, USART_DREINTLVL_LO_gc);
	/* USARTF0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(BLUETOOTH_usart_data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(BLUETOOTH_usart_data.usart, USART_RXCINTLVL_LO_gc);

	USART_Baudrate_Set(&USARTF0, 524 , 11);			//115200, from -5 scale and 524

	/* Enable both RX and TX. */
	USART_Rx_Enable(BLUETOOTH_usart_data.usart);
	USART_Tx_Enable(BLUETOOTH_usart_data.usart);
	///////////////////////////////////////////////////////////////////////////////////////////////
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	//configure USART for GPS
	/* Use USARTD1 and initialize buffers. */
	USART_InterruptDriver_Initialize(&GPS_usart_data, &USARTD1, USART_DREINTLVL_LO_gc);
	/* USARTD1, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(GPS_usart_data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(GPS_usart_data.usart, USART_RXCINTLVL_LO_gc);

	USART_Baudrate_Set(&USARTD1, 207 , 1);			//115200, from -5 scale and 524

	/* Enable both RX and TX. */
	USART_Rx_Enable(GPS_usart_data.usart);
	USART_Tx_Enable(GPS_usart_data.usart);
	///////////////////////////////////////////////////////////////////////////////////////////////
	
	

	PORTK_DIR |= 14;	//bits 1, 2, 3

	PORTK_OUT |= 2;
	
	
	ADCA_CTRLA |= 4;				//START THE FIRST CONVERSION
	
	unsigned int i, j;
	
	for (i = 0; i < 6500; i++)		//delay to let systems settle (approx one second)
	{
		for (j = 0; j < 650; j++);
	}
	
	/////////////////////////////////////////////
	/* Enable global interrupts. */
	PMIC.CTRL |= PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm;	//turn on all level of interrupts
	sei();
	/////////////////////////////////////////////
	
	return error;
}