
#ifndef SPI_H
#define SPI_H
#ifndef F_CPU
#define F_CPU 32000000UL
#endif
#include <avr/io.h>
#include <util/delay.h>
uint8_t SPI_send_byte(uint8_t c);
void SPI_init();

#endif