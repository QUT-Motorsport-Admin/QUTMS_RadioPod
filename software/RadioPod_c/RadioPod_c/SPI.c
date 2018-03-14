#include "SPI.h"

void SPI_init()
{
	SPIC.CTRL = (1<<SPI_ENABLE_bp)|SPI_PRESCALER_DIV64_gc|(1<<SPI_MASTER_bp)|SPI_MODE_0_gc;	//enable SPI with ENABLE, MSB first, master mode, reads on leading edge, speed fosc/128 (due to 2x speed not set).
	SPIC.DATA = 0x00;	//ensure the data register has nothing in it.
	SPIC.INTCTRL = 0x00;
	
	//MCUCR &= ~(1<<SPIPS);		//make sure we are getting spi not on the _A spi pins.
	//SPCR = (0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR0)|(0<<SPR1);  //set interrupt, enable SPI, MSB mode, set as master, low is idle, read on leading edge, Set to speed(fosc/64) SPR0=1, SPR1=0
	//SPSR = (1<<SPI2X);	//set whether we want 2x speed or not (1=2x speed).
	//SPDR = 0x00;		//ensure data register has nothing in it
}

uint8_t SPI_send_byte(uint8_t c )
{
	SPIC_DATA = c;					//write data to the transmission register. Writing to this initiates transmission.
	while((SPIC_STATUS & (1<<SPI_IF_bp))==0)LED_toggle();		//wait for send to complete
	return SPIC_DATA;				//return any data that was shifted into this register upon transmission.
}