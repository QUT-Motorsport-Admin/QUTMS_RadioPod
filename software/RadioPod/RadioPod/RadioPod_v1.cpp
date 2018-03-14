/*
 * RadioPod.cpp
 *
 * Created: 11/3/2015 7:02:19 PM
 *  Author: julius
 */ 

#include <avr/io.h>

void init_spi(SPI_t * spi)
{
	spi->CTRL |= (1<<SPI_ENABLE_bp)|(1<<SPI_DORD_bp)|(1<<SPI_PRESCALER0_bp)|(0<<SPI_PRESCALER1_bp)|(1<<SPI_MASTER_bp);	//enable SPI with ENABLE, MSB first, master mode, reads on leading edge, speed fosc/128 (due to 2x speed not set).
	spi->DATA = 0x00;	//ensure the data register has nothing in it.
	spi->INTCTRL |= (1<<SPI_INTLVL0_bp);
	
	//MCUCR &= ~(1<<SPIPS);		//make sure we are getting spi not on the _A spi pins.
	//SPCR = (0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR0)|(0<<SPR1);  //set interrupt, enable SPI, MSB mode, set as master, low is idle, read on leading edge, Set to speed(fosc/64) SPR0=1, SPR1=0
	//SPSR = (1<<SPI2X);	//set whether we want 2x speed or not (1=2x speed).
	//SPDR = 0x00;		//ensure data register has nothing in it
}

uint8_t spi_send_byte(SPI_t * spi, uint8_t c )
{
	spi->DATA = c;					//write data to the transmission register. Writing to this initiates transmission.
	while(!(spi->STATUS & (1<<SPI_IF_bp)));		//wait for send to complete
	return spi->DATA;				//return any data that was shifted into this register upon transmission.
}



int main(void)
{
	init_spi(MCP2515_SPI);
    while(1)
    {
        //TODO:: Please write your application code 
    }
}