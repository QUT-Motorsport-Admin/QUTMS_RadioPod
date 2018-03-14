/*
 * IncFile1.h
 *
 * Created: 11/4/2015 3:12:01 PM
 *  Author: julius
 */ 


#ifndef INCFILE1_H_
#define INCFILE1_H_

#define VOLT1_ID 		0b00001			//**
#define VOLT2_ID 		0b00010			//**
#define TEMP1_ID 		0b00100			//**
#define TEMP2_ID 		0b00101			//**
#define STATA_ID		0b01001			//**
#define AUDIT_REQUEST 	0b10001			//**
#define AUDIT_RESPONSE 	0b10011			//**

#define CMU_COUNT 			2			//** 1 for the moment, will need to be 18 on release.
#define CMU_CELL_COUNT		8			//*** this will rarely need to change. unless the PCB changes

#define ANY_CMU 			0x00		//**
#define DEVICE_ID  			0x02		//**

#define CHASSIS_PACKET_NORMAL	0x00
#define CHASSIS_PACKET_ERROR	0x01
#define CHASSIS_PACKET_STATUS	0x02

#define CHASSIS_ERROR_NODELOST	0x01
#define CHASSIS_ERROR_NODEREG	0x02
#define CHASSIS_ERROR_CELLVOLT	0x03
#define CHASSIS_ERROR_CELLTEMP	0x04

#define CHASSIS_DATA_VOLT1_4	0x01
#define CHASSIS_DATA_VOLT5_8	0x02
#define CHASSIS_DATA_TEMP1_4	0x03
#define CHASSIS_DATA_TEMP5_8	0x04

#define CMU_STATUS				GPIOR0			//general use register
#define CMU_FAULT_ADDRESSH		GPIOR1			//holds the faulty CMU
#define CMU_FAULT_ADDRESSL		GPIOR2			//holds the seconds part of CMU address
#define CMU_NORMAL				0x00
#define CMU_NODELOST_FAULT		0x01
#define CMU_DATACORRUPT_FAULT	0x07
#define CMU_RESET_FAULT			0x02
#define CMU_VOLT_FAULT			0x03
#define CMU_VOLT_FAULT_M		0x04	//multiple faults
#define CMU_TEMP_FAULT			0x05
#define CMU_TEMP_FAULT_M		0x06	//multiple faults



#endif /* INCFILE1_H_ */