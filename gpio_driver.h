

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32f407xx.h"

/***********************************************************************/
/*                                                                     */
/*                      1. Macros used for GPIO pin initialisation     */
//
/***********************************************************************/
/* GPIO Mode Selection Values*/
#define GPIO_PIN_INPUT_MODE    ((uint32_t)0x00)
#define GPIO_PIN_OUTPUT_MODE   ((uint32_t)0x01)
#define GPIO_ALT_FUN_MODE      ((uint32_t)0x02)

/*GPIO OP type selection values*/
#define GPIO_PIN_OP_TYPE_PUSHPULL  ((uint32_t)0x00)
#define GPIO_PIN_OP_TYPE_OPENDRAIN  ((uint32_t)0x01)

/*GPIO Spedd type selection values*/
#define GPIO_PIN_SPEED_LOW        ((uint32_t)0x00)
#define GPIO_PIN_SPEED_MEDIUM     ((uint32_t)0x01)
#define GPIO_PIN_SPEED_FAST       ((uint32_t)0x02)
#define GPIO_PIN_SPEED_VERY_FAST  ((uint32_t)0x03)

/*GPIO pull up/down selection*/
#define GPIO_PIN_NO_PULL_PUSH     ((uint32_t)0x00)
#define GPIO_PIN_PULL_UP          ((uint32_t)0x01)
#define GPIO_PIN_PULL_DOWN        ((uint32_t)0x02)

/* Redifining GPIO ports*/

#define GPIO_PORT_A GPIOA
#define GPIO_PORT_B GPIOB
#define GPIO_PORT_C GPIOC
#define GPIO_PORT_D GPIOD
#define GPIO_PORT_E GPIOE
#define GPIO_PORT_F GPIOF
#define GPIO_PORT_G GPIOG
#define GPIO_PORT_H GPIOH
#define GPIO_PORT_I GPIOI

/* Macros to enable Clock for diffrent GPIO ports in RCC register*/

/*Note that pointer RCC is already define in stm32f4xx.h header file
with base address and typecasted into RCC_typedef struct, so that we can
use all registers defined in RCC_Typedef structure*/

#define _HAL_RCC_GPIOA_ENABLE()   (RCC->AHB1ENR |=(1<<0))
#define _HAL_RCC_GPIOB_ENABLE()   (RCC->AHB1ENR |=(1<<1))
#define _HAL_RCC_GPIOC_ENABLE()   (RCC->AHB1ENR |=(1<<2))
#define _HAL_RCC_GPIOD_ENABLE()   (RCC->AHB1ENR |=(1<<3))
#define _HAL_RCC_GPIOE_ENABLE()   (RCC->AHB1ENR |=(1<<4))
#define _HAL_RCC_GPIOF_ENABLE()   (RCC->AHB1ENR |=(1<<5))
#define _HAL_RCC_GPIOG_ENABLE()   (RCC->AHB1ENR |=(1<<6))
#define _HAL_RCC_GPIOH_ENABLE()   (RCC->AHB1ENR |=(1<<7))

/* Macros for On board LED*/

#define GPIO_portD_12 12
#define GPIO_portD_13 13
#define GPIO_portD_14 14
#define GPIO_portD_15 15

#define LED_GREEN     GPIO_portD_12
#define LED_ORANGE    GPIO_portD_13
#define LED_RED       GPIO_portD_14
#define LED_BLUE      GPIO_portD_15

/****************************************************************/
/*                                                              */
/*              Data Structure for GPIO pin Initialisation      */
/*                                                              */
/****************************************************************/

/**
* @breif GPIO pin configuration structure
* This structure will be filled and passed to driver by the application
* to initialize the GPIO pin
*/

typedef struct
{
	uint32_t pin;
	uint32_t mode;
	uint32_t op_type;
	uint32_t pull;
	uint32_t speed;
	uint32_t alternate;
}gpio_pin_conf_t;

typedef enum
{
	INT_RISING_EDGE,
	INT_FALLING_EDGE,
	INT_RISING_FALLING_EDGE
}int_edge_sel_t;


/*************************************************************/
/*                                                           */
/*              Driver Exposed APIs                          */
/*                                                           */
/*************************************************************/

/**
* @breif  Initializes the gpio pin
* @param  *GPIOx : GPIO Port Base address
* @param  *gpio_pin_conf : Pointer to the pin conf structure sent by application
* @retval None
*/

void hal_gpio_init(GPIO_TypeDef *GPIOx,gpio_pin_conf_t *gpio_pin_conf);

/**
* @breif  Read a value from given pin number
* @param  *GPIOx : GPIO Port Base address
* @param  pin_n0 : GPIO pin number
* @retval uint8_t: Value read
*/

uint8_t hal_gpio_read_from_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no);

/**
* @breif  Write a value to given pin number
* @param  *GPIOx : GPIO Port Base address
* @param  value : value to be written
* @param  pin_n0 : GPIO pin number
* @retval None
*/

void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint8_t val);

/**
* @breif  Write a value to given pin number
* @param  *GPIOx : GPIO Port Base address
* @param  alt_fun_value: alternate function to be configured with
* @param  pin_n0 : GPIO pin number
* @retval None
*/

void hal_gpio_set_alt_fun(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint16_t alt_fun_value);

/**
* @breif  Configure the interrupt for a given pin no
* @param  edge_sel : Triggering edge selection value of type "int_edge_sel"
* @retval None
*/

void hal_gpio_configure_interrupt(uint16_t pin_no,int_edge_sel_t edge_sel);

/**
* @breif  Write a value to given pin number
* @param  irq_no: irq no to be enabled in NVIC
* @param  pin_n0 : GPIO pin number
* @retval None
*/

void hal_gpio_enable_interrupt(uint16_t pin_no,IRQn_Type irq_no);

/**
* @breif  Clears already set interrupt in the given pin
* @param  pin_n0 : GPIO pin number
* @retval None
*/
/*Not clearing out the already set interrupt will keep interrupting the process*/
void hal_gpio_clear_interrupt(uint16_t pin_no);

#endif
