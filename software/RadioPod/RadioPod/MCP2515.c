/*
 * MCP2515.cpp
 *
 * Created: 11/8/2015 8:10:00 PM
 *  Author: julius
 */ 

#include "MCP2515.h"

void MCP2515_reg_write(uint8_t reg_address, uint8_t reg_value);


void MCP2515_reg_write(uint8_t reg_address, uint8_t reg_value)
{
	MCP2515_PORT_CS &= ~(1<<MCP2515_PIN_CS);			//unset CS so MCP2515 knows we are talking
	spi_send_byte(MCP2515_SPI, MCP2515_WRITE);
	spi_send_byte(MCP2515_SPI, reg_address);
	spi_send_byte(MCP2515_SPI, reg_value);
	MCP2515_PORT_CS |= (1<<MCP2515_PIN_CS);			//set the CS.
	//if(MCP2515_reg_read(reg_address) == reg_value)flash_LED(1,RED_LED);
}

void MCP2515_instruction(uint8_t instruction)
{
	
	MCP2515_PORT_CS &= ~(1<<MCP2515_PIN_CS);			//unset CS so MCP2515 knows we are talking
	spi_send_byte(MCP2515_SPI, instruction);
	MCP2515_PORT_CS |= (1<<MCP2515_PIN_CS);			//set the CS.
}

void MCP2515_bit_modify(uint8_t reg_address, uint8_t reg_value, uint8_t reg_mask)
{
	MCP2515_PORT_CS &= ~(1<<MCP2515_PIN_CS);			//unset CS so MCP2515 knows we are talking
	spi_send_byte(MCP2515_SPI, MCP2515_BITMODIFY);		//send instruction of bitmodify
	spi_send_byte(MCP2515_SPI, reg_address);				//send address
	spi_send_byte(MCP2515_SPI, reg_mask);				//send the mask
	spi_send_byte(MCP2515_SPI, reg_value);				//send the data
	MCP2515_PORT_CS |= (1<<MCP2515_PIN_CS);			//set the CS.
}

void init_MCP2515()
{
	
	MCP2515_PORT_RESET &= ~(1<<MCP2515_PIN_RESET);
	_delay_us(50);
	MCP2515_PORT_RESET |= (1<<MCP2515_PIN_RESET);


	//spi_send_byte(MCP2515_RESET); //instead of hard reset, perform software rest.
	MCP2515_bit_modify(MCP2515_CANCTRL,128,0xE0);		//put the device into configuration mode.

	MCP2515_reg_write(MCP2515_CNF1, 0x04);	//SJW = 0(1),BRP = 4(5)--> number in brackets is actual value, as mcp2515 adds 1.
	MCP2515_reg_write(MCP2515_CNF2, 0xCA);	//BTL = 1, SAM = 1, PHSEG1 = 001(2), PRSEG = 010 (3)
	MCP2515_reg_write(MCP2515_CNF3, 0x01);	//SOF = 0, WAKFIL = 0, PHSEG2 = 001(2).
	MCP2515_reg_write(MCP2515_CANINTE, 0b00011111);	//enable interrupt in rx0, rx1, tx0, tx1, tx2.
	MCP2515_reg_write(MCP2515_RTSCTRL, 0x01); //probably want to move this to a tx init function. eventually. if it aint broke don't fix it...
	MCP2515_init_Rx();
	MCP2515_bit_modify(MCP2515_CANCTRL, 0x00, 0xE0);		//put the device into it's functional mode currently: normal, listen is 0x60
}

void MCP2515_tx(uint8_t identifier, uint8_t transmit_buffer, uint8_t device_id, uint8_t recipient, uint8_t type_code, uint8_t dlc, uint8_t * data)
{
	MCP2515_reg_write(transmit_buffer, 0x03);			//we shall set this to the highest priority so that it sends it immediately.
	identifier = identifier & 3;
	MCP2515_reg_write(transmit_buffer+1, (identifier<<6)|(device_id>>3));	//device identifier bit and the  device ID

	MCP2515_reg_write(transmit_buffer+2, (1<<3)|(device_id<<5)|(recipient>>8));

	MCP2515_reg_write(transmit_buffer+3, recipient);			//works
	MCP2515_reg_write(transmit_buffer+4, type_code);
	MCP2515_reg_write(transmit_buffer+5, dlc);
	for (uint8_t byteCount = 0; byteCount < dlc; byteCount++)
	{
		MCP2515_reg_write(transmit_buffer+6+byteCount, *(data+byteCount));
	}

}

uint8_t MCP2515_receive_status()
{
	uint8_t mcp2515_status[2];
	MCP2515_PORT_CS &= ~(1<<MCP2515_PIN_CS);			//unset CS so MCP2515 knows we are talking
	spi_send_byte(MCP2515_SPI , 0xA0);					//send retrieve status instruction
	mcp2515_status[0] = spi_send_byte(MCP2515_SPI ,0x00);//send don't care bits while mcp2515 is retrieving data.
	mcp2515_status[1] = spi_send_byte(MCP2515_SPI ,0x00);//duplicate data is retrieved again. nothing to do with this second lot yet.
	MCP2515_PORT_CS |= (1<<MCP2515_PIN_CS);			//set the CS.
	return mcp2515_status[0];					//send it back for analysis.
}

/* README before using this!!!!!!!!
 * this function takes a pointer to an array of data to write to, it must be either 8 or 13 elements in size for safe use.
 * 13 elements in size for an address of RXBnSIDH
 * 8  elements in size for an address of RXBnD0
 * e.g-------------------------
 *
 * uint8_t data[13];
 * MCP2515_RxBufferRead(data, RXB0SIDH);
 *
 * ---------------------------> this example will fill the data[13] array with elements from RXB0SIDH-->RXB0D7
 *
 * uint8_t data[8];
 * MCP2515_RxBufferRead(data, RXB0D0);
 * ---------------------------> this example will fill the data[8] array with bytes from RXB0D0-->RXB0D7
 *
 * This function also automatically clears the interrupt flag CANINTF.RX0IF(in this case)
 */
void MCP2515_RxBufferRead(uint8_t * data, uint8_t startingAddress)
{

	//the following line combines the instruction(0b10010000), with: 0b100 for rxb0 or 0b000 for rxb1, and: 0b10 for data starting at data0, or 0b00 for SIDH
	uint8_t instruction = 0b10010000|((startingAddress > 0x70)<<2)|((startingAddress == MCP2515_RXB0D0 || startingAddress == MCP2515_RXB1D0)<<1);
	MCP2515_PORT_CS &= ~(1<<MCP2515_PIN_CS);			//lower CS.
	spi_send_byte(instruction);							//send instruction for stream of data
	//loop counts to 8 or 12 depending on whether bit 1 of instruction is set.
	for(uint8_t counter = 0; counter < (8 + 4*((instruction & 2)==0)); counter++)
	{
		*data = spi_send_byte(0x00);
		data++;
	}
	MCP2515_PORT_CS |= (1<<MCP2515_PIN_CS);				//raise CS.
}

uint8_t MCP2515_reg_read(uint8_t reg_address)
{
	uint8_t read_result;
	MCP2515_PORT_CS &= ~(1<<MCP2515_PIN_CS);			//unset CS so MCP2515 knows we are talking
	spi_send_byte(0x03);
	spi_send_byte(reg_address);
	read_result = spi_send_byte(0x00);
	MCP2515_PORT_CS |= (1<<MCP2515_PIN_CS);			//set the CS.
	return read_result;
}

uint8_t MCP2515_findFreeTxBuffer()
{
	uint8_t MCP2515_TxBuffer = 0;

	MCP2515_TxBuffer = (MCP2515_reg_read(MCP2515_CANINTF)& 0b00011100);			//get interrupt status, only the txbuffer empty ones though
	if		((MCP2515_TxBuffer & 0b00000100)==0b00000100)						//if tx0 is free,
	{
		return MCP2515_TX0;
	}
	else if	((MCP2515_TxBuffer & 0b00001000)==0b00001000)						//if tx1 is free,
	{
		return MCP2515_TX1;
	}
	else if	((MCP2515_TxBuffer & 0b00010000)==0b00010000)						//if tx2 is free,
	{
		return MCP2515_TX2;
	}
	else return 0x00;															//otherwise none are free.
}





void MCP2515_init_Rx()										//specific function for the device using the MCP2515 - t
{
	MCP2515_reg_write(MCP2515_RXM0EID0, 0b00011111);		//the mask will force RXB0 to only look at the last 5 bits of the EID. looking for specific message types.
	MCP2515_reg_write(MCP2515_RXM1EID0, 0b00011111);

	//MCP2515_reg_write(MCP2515_RXF0EID0, VOLT2_ID);			//RXB0 will only accept the second cell data packet from the CMU.
	//MCP2515_reg_write(MCP2515_RXF0SIDL, (1<<3));
	//MCP2515_reg_write(MCP2515_RXF1EID0, TEMP2_ID);			//second temp
	//MCP2515_reg_write(MCP2515_RXF1SIDL, (1<<3));

	//MCP2515_reg_write(MCP2515_RXF2EID0, AUDIT_RESPONSE);	//RXB1, programmed to accept audit response,
	//MCP2515_reg_write(MCP2515_RXF2SIDL, (1<<3));			//WIERD ONE, TAKE NOTE OF --> must set the EXIDE bit so that it filters only extended can packets.
	//MCP2515_reg_write(MCP2515_RXF3EID0, STATA_ID);			//status message A
	//MCP2515_reg_write(MCP2515_RXF3SIDL, (1<<3));
	//MCP2515_reg_write(MCP2515_RXF4EID0, TEMP1_ID);			//temp1
	//MCP2515_reg_write(MCP2515_RXF4SIDL, (1<<3));
	//MCP2515_reg_write(MCP2515_RXF5EID0, VOLT1_ID);			//RXB1 will only accept single can packets(status or response or the first of the cell data packets
	//MCP2515_reg_write(MCP2515_RXF5SIDL, (1<<3));


	MCP2515_reg_write(MCP2515_RXB0CTRL, 0b01000000);
	MCP2515_reg_write(MCP2515_RXB1CTRL, 0b01000000);
	MCP2515_reg_write(MCP2515_BFPCTRL,  0b00000101);			//bit 2: enable output of rx0buf pin, bit 0: rx0buf pin to trigger on reception of rx0b
}

uint8_t MCP2515_check_receive_status()
{
	uint8_t status;
	PORTB &= ~(1<<MCP2515_PIN_CS);			//unset CS so MCP2515 knows we are talking
	spi_send_byte(0b10110000);
	status = spi_send_byte(0x00);
	spi_send_byte(0x00);
	PORTB |= (1<<MCP2515_PIN_CS);			//set the CS
	return status;
}



