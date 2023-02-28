/*
 * led.c
 *
 *      Author: DZ
 */

#include "led.h"

void LEDRGB_RED(uint8_t on)
{
	if(!on)
	HAL_GPIO_WritePin(GPIOB,LED2_Pin,GPIO_PIN_RESET);
	else
	HAL_GPIO_WritePin(GPIOB,LED2_Pin,GPIO_PIN_SET);
}

void LEDRGB_BLUE(uint8_t on)
{
	if(!on)
	HAL_GPIO_WritePin(GPIOB,LED1_Pin,GPIO_PIN_RESET);
	else
	HAL_GPIO_WritePin(GPIOB,LED1_Pin,GPIO_PIN_SET);
}

