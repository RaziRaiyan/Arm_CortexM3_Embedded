#include "led.h"

int main(void)
{
	led_init();
	_HAL_RCC_GPIOA_ENABLE();
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN,INT_FALLING_EDGE);
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN,EXTI0_IRQn);
	while(1);
}

/**
 * @breif Initializes the LEDs
 * @param None
 * @retval None
**/

void led_init(void)
{
	gpio_pin_conf_t led_pin_conf;
	_HAL_RCC_GPIOD_ENABLE();
	
	led_pin_conf.pin=LED_ORANGE;
	led_pin_conf.mode=GPIO_PIN_OUTPUT_MODE;
	led_pin_conf.op_type=GPIO_PIN_OP_TYPE_PUSHPULL;
	led_pin_conf.speed=GPIO_PIN_SPEED_MEDIUM;
	led_pin_conf.pull=GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_init(GPIO_PORT_D,&led_pin_conf);
	
	led_pin_conf.pin=LED_BLUE;
	hal_gpio_init(GPIO_PORT_D,&led_pin_conf);
	
	led_pin_conf.pin=LED_RED;
	hal_gpio_init(GPIO_PORT_D,&led_pin_conf);
	
	led_pin_conf.pin=LED_GREEN;
	hal_gpio_init(GPIO_PORT_D,&led_pin_conf);
}

void led_turn_on(GPIO_TypeDef *GPIOx,uint16_t pin_no)
{
	hal_gpio_write_to_pin(GPIOx,pin_no,1);
}

void led_turn_off(GPIO_TypeDef *GPIOx,uint16_t pin_no)
{
	hal_gpio_write_to_pin(GPIOx,pin_no,0);
}

void led_toggle(GPIO_TypeDef *GPIOx,uint16_t pin_no)
{
	if(hal_gpio_read_from_pin(GPIO_PORT_D,LED_ORANGE))
	{
		hal_gpio_write_to_pin(GPIOx,pin_no,0);
		delay();
	}
	else
	{
		hal_gpio_write_to_pin(GPIOx,pin_no,1);
		delay();
	}
}

void delay()
{
	int i,j;
	for(i=0;i<50000;i++)
		for(j=0;j<10;j++);
}

//Interrupt service routine for EXTI0
void EXTI0_IRQHandler(void)
{
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	/*Do your task here*/
	led_toggle(GPIOD,LED_BLUE);
	led_toggle(GPIOD,LED_ORANGE);
	led_toggle(GPIOD,LED_GREEN);
	led_toggle(GPIOD,LED_RED);
}
