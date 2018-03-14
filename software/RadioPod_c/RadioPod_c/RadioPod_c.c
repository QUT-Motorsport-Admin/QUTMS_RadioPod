/*
 * RadioPod_c.c
 *
 * Created: 11/4/2015 11:25:20 AM
 *  Author: julius
 */ 

/*#define F_CPU 32000000
#include "MCP2515.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void clock_init();
void spi_init();
uint8_t spi_send_byte(uint8_t c );

void clock_init()
{
	//Check configuration change protection section in datasheet before finishing or changing this section.
	//A new source cannot be selected if the new one is not stable.

	OSC_XOSCCTRL = (3<<6)|(11);							//set frequency range to 12-16MHz (FRQRANGE[7:6] = 3), and set the startup time to 16k(max) clock cycles(XOSCSEL[3:0]= 11)
	OSC_PLLCTRL = 2|(3<<6);								//set multiplier to 2 (PLLFAC[4:0] = 2), set PLL source to external crystal (PLLSRC[7:6] = 3)
	OSC_CTRL |= (1<<OSC_XOSCEN_bp)|(1<<OSC_PLLEN_bp);	// enable the external oscillator and the PLL.
	while(!(OSC_STATUS & OSC_XOSCRDY_bm) || !(OSC_STATUS & OSC_PLLRDY_bm))PORTK_OUT ^= (1<<PIN2_bp);	//wait for external crystal to stabilise, and PLL to be ready.

	CCP = CCP_IOREG_gc;						// disable register security for clock update
	CLK_CTRL = CLK_SCLKSEL_PLL_gc;			// switch clock source to the PLL
}
void spi_init()
{
	SPIC.CTRL = (1<<SPI_ENABLE_bp)|SPI_PRESCALER_DIV128_gc|(1<<SPI_MASTER_bp)|SPI_MODE_0_gc;	//enable SPI with ENABLE, MSB first, master mode, reads on leading edge, speed fosc/128 (due to 2x speed not set).
	SPIC.DATA = 0x00;	//ensure the data register has nothing in it.
	SPIC.INTCTRL = 0x00;
	
	//MCUCR &= ~(1<<SPIPS);		//make sure we are getting spi not on the _A spi pins.
	//SPCR = (0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR0)|(0<<SPR1);  //set interrupt, enable SPI, MSB mode, set as master, low is idle, read on leading edge, Set to speed(fosc/64) SPR0=1, SPR1=0
	//SPSR = (1<<SPI2X);	//set whether we want 2x speed or not (1=2x speed).
	//SPDR = 0x00;		//ensure data register has nothing in it
}

uint8_t spi_send_byte(uint8_t c )
{
	SPIC.DATA = c;					//write data to the transmission register. Writing to this initiates transmission.
	while(!(SPIC.STATUS & 0x80));		//wait for send to complete
	return SPIC.DATA;				//return any data that was shifted into this register upon transmission.
}

int main(void)
{
	PORTK_DIR = 7;
	PORTC_DIR = (1<<PIN4_bp)|(1<<PIN5_bp)|(1<<PIN7_bp);
	//PORTC_OUT |= 1<<PIN6_bp;
	clock_init();
	spi_init();
	//spi_send_byte(&MCP2515_SPI, 5);
	//spi_send_byte(&MCP2515_SPI, 5);
	
	//PMIC.CTRL |= PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm;	//turn on all level of interrupts
	//sei();
	//_delay_ms(200);
	MCP2515_init();
	uint8_t data = 0b10101010; 
    while(1)
    {
		//PORTK_OUT ^= (1<<PIN1_bp);
		//MCP2515_init();
		//spi_send_byte(&MCP2515_SPI, 0xA0);
		//SPIC.DATA=0x00;
		_delay_ms(5);
		//if((OSC_STATUS & (1<<OSC_XOSCRDY_bp))&&(OSC_STATUS & (1<<OSC_PLLRDY_bp)))PORTK_OUT ^= (1<<PIN2_bp);
		MCP2515_receive_status();
        //MCP2515_tx(1,1,1,1,1,1,&data);
    }
}

ISR(SPIC_INT_vect)		//receive interrupt routine for the spi
{
	MCP2515_SPI.DATA = 0x00;
	PORTK_OUT ^= (1<<PIN3_bp);
}*/

/*
 * RadioPod_c.c
 *
 * Created: 14/5/2016 21:34:20 AM
 *  Author: julius
 */ 

#define F_CPU 32000000
#include <util/delay.h>
#include <avr/io.h>

#include <avr/interrupt.h>
#include "MCP2515.h"
#include "SPI.h"
#define NULL 0
void clock_init();

void clock_init()
{
	//CCP = CCP_IOREG_gc;						// disable register security for clock update
	//CLK.CTRL = CLK_SCLKSEL_RC32M_gc;			// switch clock source to the PLL
	// Configure clock to 32MHz
	OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;  /* Enable the internal 32MHz & 32KHz oscillators */
	while(!(OSC.STATUS & OSC_RC32KRDY_bm));       /* Wait for 32Khz oscillator to stabilize */
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));       /* Wait for 32MHz oscillator to stabilize */
	DFLLRC32M.CTRL = DFLL_ENABLE_bm ;             /* Enable DFLL - defaults to calibrate against internal 32Khz clock */
	CCP = CCP_IOREG_gc;                           /* Disable register security for clock update */
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;              /* Switch to 32MHz clock */
	OSC.CTRL &= ~OSC_RC2MEN_bm;                   /* Disable 2Mhz oscillator */
}

void LED_toggle()
{
	PORTK_OUT ^= PIN3_bm|PIN2_bm|PIN1_bm;
}

int main(void)
{
	_delay_ms(10);
	clock_init();
	//PORTC_DIRCLR = (1<<PIN6_bp);
	PORTC_DIR = (1<<PIN2_bp)|(1<<PIN1_bp)|(1<<PIN0_bp)|(1<<PIN3_bp)|(1<<PIN4_bp)|(1<<PIN5_bp)|(1<<PIN7_bp);
	PORTC_OUT |= 0b00001111;
	PORTK_DIR |= PIN3_bm|PIN2_bm|PIN1_bm;	//1,2,3 are for LEDs
	
	SPI_init();
	//spi_send_byte(&MCP2515_SPI, 5);
	//spi_send_byte(&MCP2515_SPI, 5);
	
	//PMIC.CTRL |= PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm;	//turn on all level of interrupts
	//sei();
	//_delay_ms(200);
	MCP2515_init();
	MCP2515_reg_write(MCP2515_CANINTF, 0b00011100);
	//uint8_t data = 0b10101010; 
	uint8_t free_buffer = 0x00;
    while(1)
    {

		
		//MCP2515_init();
		//MCP2515_reg_read(MCP2515_CANCTRL);
		_delay_ms(100);

		//MCP2515_receive_status();

		free_buffer = MCP2515_findFreeTxBuffer();
        if(free_buffer)
		{
			//LED_toggle();
			MCP2515_TX(free_buffer,0,&free_buffer, 0x20 );
		}
		//MCP2515_tx(1,free_buffer,1,1,1,1,&data);
		
    }
}

/*ISR(SPIC_INT_vect)		//receive interrupt routine for the spi
{
	MCP2515_SPI.DATA = 0x00;
}*/