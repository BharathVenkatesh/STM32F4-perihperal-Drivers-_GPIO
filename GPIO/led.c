/*

 * main.c
 *
 *  Created on: Aug 8, 2018
 *      Author: bhara
 */
#include "led.h"

void led_init(void){
	gpio_pin_conf led_pin_config;
	_HAL_RCC_GPIO_D_CLK_ENABLE();

	led_pin_config.pin = LED_ORANGE;
	led_pin_config.mode = GPIO_PIN_OUTPUT_MODE;
	led_pin_config.output_type = GPIO_PIN_PUSH_PULL;
	led_pin_config.speed = GPIO_PIN_MEDUIM_SPEED;
	led_pin_config.pull = GPIO_PIN_NO_PULL_UP_PULL_DOWN;
	hal_gpio_initialization_fn(GPIOD, &led_pin_config);

	led_pin_config.pin = LED_BLUE;
	hal_gpio_initialization_fn(GPIOD, &led_pin_config);

	led_pin_config.pin = LED_GREEN;
	hal_gpio_initialization_fn(GPIOD, &led_pin_config);

	led_pin_config.pin = LED_RED;
	hal_gpio_initialization_fn(GPIOD, &led_pin_config);
}


void led_turn_on(GPIO_TypeDef * GPIOx, uint16_t pin_number){
	hal_gpio_data_writing_fn(GPIOx, pin_number, 1);
}


void led_turn_off(GPIO_TypeDef * GPIOx, uint16_t pin_number){
	hal_gpio_data_writing_fn(GPIOx, pin_number, 0);
}

void led_toggle(GPIO_TypeDef * GPIOx, uint16_t pin_number){
	if(hal_gpio_data_reading_fn(GPIOx,pin_number)){
		hal_gpio_data_writing_fn(GPIOx, pin_number, 0);
	}
	else{
		hal_gpio_data_writing_fn(GPIOx, pin_number, 1);
	}
}

void EXTI0_IRQHandler(void){
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	led_toggle(GPIOD,LED_RED);
	//for(int i=1;i<500000;i++);
	led_toggle(GPIOD,LED_ORANGE);
	
}
int main(void){

	uint32_t i;
	led_init();

	//Enabling clock for gpio port A
	_HAL_RCC_GPIO_A_CLK_ENABLE();
	
	//configuring button interrupt
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, FALLING_EDGE);
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN, EXTI0_IRQn);
	
	while(1){
		led_turn_on(GPIOD,LED_GREEN);
		led_turn_on(GPIOD,LED_BLUE);

		for(i=1;i<500000;i++);

		led_turn_off(GPIOD,LED_GREEN);
		led_turn_off(GPIOD,LED_BLUE);

		for(i=1;i<500000;i++);
	}
	return 0;
}


