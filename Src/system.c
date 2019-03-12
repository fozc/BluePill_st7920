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




