/*
 * ST7920.c
 *
 *  Created on: 3 Mart 2019
 *      Author: fatih
 */

#include "ST7920.h"
#include "system.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_spi.h"

/*
 *  GLCD_PIN	SPI
 *  E   ------> CLK
 *  RS  ------> CS
 *  RW  ------> Data (SID)
 */

#define ST7920_RS_PIN_PORT 		  GPIOB
#define ST7920_E_PIN_PORT  		  GPIOB
#define ST7920_RW_PIN_PORT 		  GPIOB
#define ST7920_POWER_PIN_PORT 	  GPIOB
#define ST7920_BACKLIGHT_PIN_PORT GPIOB
#define ST7920_RESET_PIN_PORT 	  GPIOB

#define ST7920_RS_PIN 		 LL_GPIO_PIN_12
#define ST7920_E_PIN  		 LL_GPIO_PIN_13
#define ST7920_RW_PIN 		 LL_GPIO_PIN_15
#define ST7920_POWER_PIN 	 LL_GPIO_PIN_10
#define ST7920_BACKLIGHT_PIN LL_GPIO_PIN_11
#define ST7920_RESET_PIN 	 LL_GPIO_PIN_14

#define ST7920_SET_RS_PIN()    LL_GPIO_SetOutputPin(ST7920_RS_PIN_PORT,   ST7920_RS_PIN)
#define ST7920_RESET_RS_PIN()  LL_GPIO_ResetOutputPin(ST7920_RS_PIN_PORT, ST7920_RS_PIN)
#define ST7920_SET_E_PIN()     LL_GPIO_SetOutputPin(ST7920_E_PIN_PORT,    ST7920_E_PIN)
#define ST7920_RESET_E_PIN()   LL_GPIO_ResetOutputPin(ST7920_E_PIN_PORT,  ST7920_E_PIN)
#define ST7920_SET_RW_PIN()    LL_GPIO_SetOutputPin(ST7920_RW_PIN_PORT,   ST7920_RW_PIN)
#define ST7920_RESET_RW_PIN()  LL_GPIO_ResetOutputPin(ST7920_RW_PIN_PORT, ST7920_RW_PIN)
#define ST7920_SET_RST_PIN()   LL_GPIO_SetOutputPin(ST7920_RESET_PIN_PORT,   ST7920_RESET_PIN)
#define ST7920_RESET_RST_PIN() LL_GPIO_ResetOutputPin(ST7920_RESET_PIN_PORT, ST7920_RESET_PIN)
#define ST7920_SET_BKL_PIN()   LL_GPIO_SetOutputPin(ST7920_BACKLIGHT_PIN_PORT,   ST7920_BACKLIGHT_PIN)
#define ST7920_RESET_BKL_PIN() LL_GPIO_ResetOutputPin(ST7920_BACKLIGHT_PIN_PORT, ST7920_BACKLIGHT_PIN)


void st7920_io_cfg(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

	LL_GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = ST7920_RS_PIN | ST7920_RESET_PIN | ST7920_RW_PIN | ST7920_E_PIN | ST7920_RS_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(ST7920_RS_PIN_PORT, &GPIO_InitStruct);

	LL_GPIO_SetOutputPin(ST7920_RS_PIN_PORT, ST7920_RS_PIN);
	LL_GPIO_SetOutputPin(ST7920_RS_PIN_PORT, ST7920_RESET_PIN);
	LL_GPIO_SetOutputPin(ST7920_RS_PIN_PORT, ST7920_RW_PIN);
	LL_GPIO_SetOutputPin(ST7920_RS_PIN_PORT, ST7920_E_PIN);
	LL_GPIO_SetOutputPin(ST7920_RS_PIN_PORT, ST7920_RS_PIN);
}

uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
	uint8_t res = 1;
	switch(msg)
	{
    case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
    	st7920_io_cfg();
      break;							// can be used to setup pins
    case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
      break;
    case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
    	__NOP();__NOP();
      break;
    case U8X8_MSG_DELAY_10MICRO:		// delay arg_int * 10 micro seconds
    	__delay_us(10);
      break;
    case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
    	delay_ms(arg_int);
      break;
	case U8X8_MSG_GPIO_SPI_CLOCK: /* Set the spi clock line level */
		if (arg_int){
			ST7920_SET_E_PIN();
		}else{
			ST7920_RESET_E_PIN();
		}
		break;
	case U8X8_MSG_GPIO_SPI_DATA: /* Set the spi data line leve */
		if (arg_int){
			ST7920_SET_RW_PIN();
		}else{
			ST7920_RESET_RW_PIN();
		}
	break;
	case U8X8_MSG_GPIO_CS1: /* Set the spi chip slect line level. */
		if (arg_int){
			ST7920_SET_RS_PIN();
		}else{
			ST7920_RESET_RS_PIN();
		}
	break;
	case U8X8_MSG_GPIO_RESET: /* Set glcd reset line level. */
		if (arg_int){
			ST7920_SET_RST_PIN();
		}else{
			ST7920_RESET_RST_PIN();
		}
	break;
	default:
		res = 0;
		return res; /* not implemented */
	}

	return res;
}
