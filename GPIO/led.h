/*
 * led.h

 *
 *  Created on: Aug 19, 2018
 *      Author: bhara
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f407xx_gpio_driver.h"

#define GPIOD_PIN_12 	12
#define GPIOD_PIN_13 	13
#define GPIOD_PIN_14 	14
#define GPIOD_PIN_15 	15
#define GPIO_BUTTON_PIN 0

#define LED_GREEN 		GPIOD_PIN_12
#define LED_ORANGE 		GPIOD_PIN_13
#define LED_RED			GPIOD_PIN_14
#define LED_BLUE 		GPIOD_PIN_15


/*
 * Init function to initialize the gpio pins that handle these LEDs
 */
void led_init(void);

/* functions to turn on and turn off led */
void led_turn_on(GPIO_TypeDef * GPIOx, uint16_t pin_number);
void led_turn_off(GPIO_TypeDef * GPIOx, uint16_t pin_number);
void led_toggle(GPIO_TypeDef * GPIOx, uint16_t pin_number);

#endif /* LED_H_ */
