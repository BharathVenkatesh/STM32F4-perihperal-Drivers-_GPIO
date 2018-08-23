/*
 * stm32f407xx_gpio_driver.h

 *
 *  Created on: Aug 8, 2018
 *      Author: bharath
 */

#ifndef INCLUDE_STM32F4XX_STM32F407XX_GPIO_DRIVER_H_
#define INCLUDE_STM32F4XX_STM32F407XX_GPIO_DRIVER_H_


/*adding stm32f407xxheader file*/
#include<stm32f407xx.h>


/**********************************************
 * Gpio pin initialization macros. These are the register bit defining macros for GPIO pin
 **********************************************/


/* values for modes of the gpio pin */
#define GPIO_PIN_INPUT_MODE							((uint32_t)0x00)
#define GPIO_PIN_OUTPUT_MODE						((uint32_t)0x01)
#define GPIO_PIN_ALTERNATE_FUNTION_MODE				((uint32_t)0x02)
#define GPIO_PIN_ANALOG_MODE						((uint32_t)0x03)


/* output type deciding values*/
#define GPIO_PIN_PUSH_PULL 							((uint32_t)0x00)
#define GPIO_PIN_OPEN_DRAIN 						((uint32_t)0x01)


/*values for output speed register*/
#define GPIO_PIN_LOW_SPEED							((uint32_t)0x00)
#define GPIO_PIN_MEDUIM_SPEED						((uint32_t)0x01)
#define GPIO_PIN_HIGH_SPEED							((uint32_t)0x02)
#define GPIO_PIN_VERY_HIGH_SPEED					((uint32_t)0x03)


/*GPIO pull up/pull down register macros to decide mode  of I/O pin as pull up or pull down (GPIOx_PUPDR)*/
#define GPIO_PIN_NO_PULL_UP_PULL_DOWN				((uint32_t)0x00)
#define GPIO_PIN_PULL_UP							((uint32_t)0x01)
#define GPIO_PIN_PULL_DOWN							((uint32_t)0x02)


/*GPIO port addresses from stm43f407xx.h*/
#define GPIO_PORT_A GPIOA
#define GPIO_PORT_B GPIOB
#define GPIO_PORT_C GPIOC
#define GPIO_PORT_D GPIOD
#define GPIO_PORT_E GPIOE

/*macros to enable clocks to all GPIO ports the rcc AHB1ENR is used to enable clock
 * because ALL gpios connec through AHB 1 bus*/

#define _HAL_RCC_GPIO_A_CLK_ENABLE() 					(RCC->AHB1ENR |= (1 << 0))
#define _HAL_RCC_GPIO_B_CLK_ENABLE() 					(RCC->AHB1ENR |= (1 << 1))
#define _HAL_RCC_GPIO_C_CLK_ENABLE() 					(RCC->AHB1ENR |= (1 << 2))
#define _HAL_RCC_GPIO_D_CLK_ENABLE() 					(RCC->AHB1ENR |= (1 << 3))
#define _HAL_RCC_GPIO_E_CLK_ENABLE() 					(RCC->AHB1ENR |= (1 << 4))






/*****
 * Data structures for GPIO pin Initialization
 * these structs will be used to initialize and configure the gpio pins
 *****/

/*****
 * struct that contains all the variables related to initializing the gpio pin
 *****/

typedef struct
{

	uint32_t pin;				/* pin to be configured */

	uint32_t mode;				/* mode of gpio pin. input, output, alternate or analog */

	uint32_t pull;				/* to pull up or pull down during output mode */

	uint32_t output_type;

	uint32_t speed;

	uint32_t alternate;

}gpio_pin_conf;


/*****
 * enum that is required for edge selection for the interrupts
 *****/

typedef enum{
	RISING_EDGE,
	FALLING_EDGE,
	RISING_FALLING_EDGE
}edge_select_t;

/*****
 * functions (APIs) to the user.
 *
 *
 * Initialization function
 * Takes the address of the port x on first argument GPIO_TypeDef * GPIOx.
 * takes a pointer to gpio configuration structure gpio_pin_conf * gpio_pin_config_struct_pointer
 * it then initializes the gpio pin using this structure
 *****/
void hal_gpio_initialization_fn(GPIO_TypeDef * GPIOx, gpio_pin_conf * gpio_pin_config);


/*****
 * Pin mode setting function:  Takes the pin number and mode from gpio_pin_conf structure and sets the
 * corresponding bits in MODER register of the pin.
 *****/
void hal_gpio_pin_mode_configuration(GPIO_TypeDef * GPIOx, uint32_t pin_number, uint32_t mode);


/*****
 *function to set the output type for the pin.
 *****/
void hal_gpio_output_type(GPIO_TypeDef * GPIOx, uint32_t pin_number, uint32_t output_type);

/*****
 *function to set the speed for the pin.
 *****/
void hal_gpio_speed(GPIO_TypeDef * GPIOx, uint32_t pin_number, uint32_t speed);


/*****
 *function to configure the pin pull up or pull down.
 *****/
void hal_gpio_pupdr_conf(GPIO_TypeDef * GPIOx, uint32_t pin_number, uint32_t pull);


/*****
 *function to configure the alternate function register for the selected pin.
 *****/
void hal_gpio_alternate_fn_conf(GPIO_TypeDef * GPIOx, uint32_t pin_number, uint32_t alternate);


/*****
 *function to read data from IO pin
 *****/
uint8_t hal_gpio_data_reading_fn(GPIO_TypeDef * GPIOx, uint16_t pin_number);


/*****
 *function to write the data to IO pin
 *****/
void hal_gpio_data_writing_fn(GPIO_TypeDef * GPIOx, uint16_t pin_number,uint8_t value);

/*****
 *function to configure external interrupt controller for the gpio pins
 *****/
void hal_gpio_configure_interrupt(uint16_t pin_number, edge_select_t edge_sel);

/*****
 *function to enable  external interrupt for the given gpio pin
 *****/
void hal_gpio_enable_interrupt(uint16_t pin_number, IRQn_Type irq_number);

/*****
 *function to clear external interrupt for the given gpio pin
 *****/
void hal_gpio_clear_interrupt(uint16_t pin_number);

#endif /* INCLUDE_STM32F4XX_STM32F407XX_GPIO_DRIVER_H_ */
