/*
 * system.h
 *
 *  Created on: 12 Mar 2019
 *      Author: fatih
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "stdint.h"

void 	 delay_ms		(uint32_t ms);
uint32_t get_systick	(void);
void 	__delay_us		(uint32_t micros);


#endif /* SYSTEM_H_ */
