/*
 * system.c
 *
 *  Created on: 12 Mar 2019
 *      Author: fatih
 */
#include "system.h"

uint32_t systick = 0;

uint32_t get_systick(void)
{
	return systick;
}

void delay_ms(uint32_t ms)
{
	uint32_t now = get_systick();
	while(get_systick() - now < ms);
}

void  __attribute__((optimize("O1"))) __delay_us(uint32_t micros) /* min: 3.3 usec */
{
    /* Multiply micros with multipler */
    /* Substract 10 */
	if(micros < 3) micros = 3;
	const uint32_t multiplier = 48000000 / 4000000;
    micros =  micros * multiplier - 12;
    /* 4 cycles for one loop */
    while (micros--);
}


