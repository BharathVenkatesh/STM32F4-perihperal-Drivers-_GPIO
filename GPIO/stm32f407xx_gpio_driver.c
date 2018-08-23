/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Aug 8, 2018
 *      Author: bhara
 */
#include "stm32f407xx_gpio_driver.h"

/**
 * Implementaiton of gpio apis for initialization can be found here
 */
void hal_gpio_initialization_fn(GPIO_TypeDef * GPIOx, gpio_pin_conf * gpio_pin_config){
		hal_gpio_pin_mode_configuration(GPIOx, gpio_pin_config->pin, gpio_pin_config->mode);
		hal_gpio_speed(GPIOx, gpio_pin_config->pin, gpio_pin_config->speed);
		hal_gpio_output_type(GPIOx, gpio_pin_config->pin, gpio_pin_config->output_type);
		hal_gpio_pupdr_conf(GPIOx, gpio_pin_config->pin, gpio_pin_config->pull);
}


void hal_gpio_pin_mode_configuration(GPIO_TypeDef * GPIOx, uint32_t pin_number, uint32_t mode){
	GPIOx->MODER |= (mode<<(2 * pin_number));
}

void hal_gpio_output_type(GPIO_TypeDef *GPIOx, uint32_t pin_number, uint32_t output_type){
	GPIOx->OTYPER |= (output_type << pin_number);
}

void hal_gpio_speed(GPIO_TypeDef * GPIOx, uint32_t pin_number, uint32_t speed){
	GPIOx->OSPEEDR |= (speed<<(2 * pin_number));
}

void hal_gpio_pupdr_conf(GPIO_TypeDef * GPIOx, uint32_t pin_number, uint32_t pull){
	GPIOx->PUPDR |= (pull<<(2 * pin_number));
}

void hal_gpio_alternate_fn_conf(GPIO_TypeDef * GPIOx, uint32_t pin_number, uint32_t alternate){
	if(pin_number<=7){
		GPIOx->AFR[0] |= (alternate<<(4 * pin_number));
	}
	GPIOx->AFR[1] |= (alternate<<(4 * (pin_number%8)));
}


uint8_t hal_gpio_data_reading_fn(GPIO_TypeDef * GPIOx, uint16_t pin_number){

	uint8_t value;
	value=(GPIOx->IDR>>pin_number) & 0x00000001;
	return value;
}

void hal_gpio_data_writing_fn(GPIO_TypeDef * GPIOx, uint16_t pin_number,uint8_t value){
	if(value)
		GPIOx->BSRR = (uint32_t)(0x00000001<<pin_number);
	else
		GPIOx->BSRR = (uint32_t)(0x00010000<<pin_number);
}


void hal_gpio_configure_interrupt(uint16_t pin_number, edge_select_t edge_sel){
	if(edge_sel == RISING_EDGE){
		EXTI->RTSR |= (1<<pin_number);
	}
	else if(edge_sel == FALLING_EDGE){
		EXTI->FTSR |= (1<<pin_number);
	}
	else if(edge_sel == RISING_FALLING_EDGE){
		EXTI->RTSR |= (1<<pin_number);
		EXTI->FTSR |= (1<<pin_number);
	}
}


void hal_gpio_enable_interrupt(uint16_t pin_number, IRQn_Type irq_number){
	EXTI->IMR |=(1<<pin_number);
	NVIC_EnableIRQ(irq_number);
}


void hal_gpio_clear_interrupt(uint16_t pin_number){
	if(EXTI->PR &(1<<pin_number)){
		EXTI->PR |=(1<<pin_number);
	}
}