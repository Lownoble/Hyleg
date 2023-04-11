/*
 * led.h
 *
 *      Author: DZ
 */

#ifndef ICODE_LED_LED_H_
#define ICODE_LED_LED_H_

#include"main.h"

#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOB


void LEDRGB_RED(uint8_t on);
void LEDRGB_BLUE(uint8_t on);

#define Cylinder_Pin GPIO_PIN_0
#define Cylinder_GPIO_Port GPIOC
void Cylinder(uint8_t on);


#endif /* ICODE_LED_LED_H_ */
