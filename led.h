#ifndef __LED_H
#define __LED_H

#include "gpio_driver.h"

/* Macros for On board LED*/

#define GPIO_portD_12 12
#define GPIO_portD_13 13
#define GPIO_portD_14 14
#define GPIO_portD_15 15

#define LED_GREEN     GPIO_portD_12
#define LED_ORANGE    GPIO_portD_13
#define LED_RED       GPIO_portD_14
#define LED_BLUE      GPIO_portD_15
#define GPIO_BUTTON_PIN 0


void led_init(void);

void led_turn_on(GPIO_TypeDef *GPIOx,uint16_t pin_no);
	
void led_turn_off(GPIO_TypeDef *GPIOx,uint16_t pin_no);

void led_toggle(GPIO_TypeDef *GPIOx,uint16_t pin_no);

void delay(void);
#endif
