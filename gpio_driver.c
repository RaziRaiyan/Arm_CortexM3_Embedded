#include "gpio_driver.h"

/*******************************************************************/
/*                                                                 */
/*            Static Helper Funcions                               */
/*                                                                 */
/*******************************************************************/

/**
* @breif  Conifgues the mode of the pin: input,output,alternate,analog
* @param  *GPIOx : GPIO Port Base address
* @param  mode:mode to be configured with
* @param  pin_n0 : GPIO pin number
* @retval None
*/


static void hal_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t mode)
{
		GPIOx->MODER |=(mode<<(2*pin_no));
}

/**
* @breif  Configures the output type 
* @param  *GPIOx : GPIO Port Base address
* @param  op_type: 
* @param  pin_n0 : GPIO pin number
* @retval None
*/

static void hal_gpio_configure_pin_otype(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t op_type)
{
	GPIOx->OTYPER|=(op_type<<pin_no);
}

/**
* @breif  Configures the speed of the pin
* @param  *GPIOx : GPIO Port Base address
* @param  speed: value of the speed 
* @param  pin_n0 : GPIO pin number
* @retval None
*/

static void hal_gpio_configure_pin_speed(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t speed)
{
	GPIOx->OSPEEDR|=(speed<<(2*pin_no));
}

/**
* @breif  Configures the pullup/down of the pin
* @param  *GPIOx : GPIO Port Base address
* @param  pupd: specifies type of pull
* @param  pin_n0 : GPIO pin number
* @retval None
*/

static void hal_gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t pupd)
{
	GPIOx->PUPDR|=(pupd<<(2*pin_no));
}

/**
* @breif  Configures the alternate function of the pin
* @param  *GPIOx : GPIO Port Base address
* @param  alt_fun: type of alternate function
* @param  pin_n0 : GPIO pin number
* @retval None
*/

static void hal_gpio_set_alt_function(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint32_t alt_fun_val)
{
	if(pin_no<7)
	{
		GPIOx->AFR[0] |=(alt_fun_val<<(4*pin_no));
	}
	else
	{
		GPIOx->AFR[1] |=(alt_fun_val<<((pin_no%8)*4));
	}
	
}

/**
* @breif  Reads value from a given pin no
* @param  *GPIOx : GPIO Port Base address
* @param  pin_n0 : GPIO pin number
* @retval uint8_t: Value read
*/

uint8_t hal_gpio_read_from_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no)
{
	uint8_t value=(GPIOx->IDR>>(pin_no)&0x00000001);
	return value;
}

/**
* @breif  Writes value to a given pin no
* @param  *GPIOx : GPIO Port Base address
* @param  pin_n0 : GPIO pin number
* @param  value : value to be written
* @retval uint8_t: Value read
*/

void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint8_t val)
{
	if(val)
	{
		GPIOx->ODR|=(1<<pin_no);
	}
	else
	{
		GPIOx->ODR&=~(1<<pin_no);
	}	
}


void hal_gpio_init(GPIO_TypeDef *GPIOx, gpio_pin_conf_t *gpio_pin_conf)
{
    hal_gpio_configure_pin_mode(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->mode);
    hal_gpio_configure_pin_speed(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->speed);
    hal_gpio_configure_pin_otype(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->op_type);
    hal_gpio_configure_pin_pupd(GPIOx, gpio_pin_conf->pin, gpio_pin_conf->pull);
}

void hal_gpio_configure_interrupt(uint16_t pin_no,int_edge_sel_t edge_sel)
{
	if(edge_sel==INT_RISING_EDGE)
	{
		EXTI->RTSR|=(1<<(pin_no));
	}
	else if(edge_sel==INT_FALLING_EDGE)
	{
		EXTI->FTSR|=(1<<(pin_no));
	}
	else if(edge_sel==INT_RISING_FALLING_EDGE)
	{
		EXTI->FTSR|=(1<<(pin_no));
		EXTI->RTSR|=(1<<(pin_no));
	}
	else
	{
		//TODO
	}
}
void hal_gpio_enable_interrupt(uint16_t pin_no,IRQn_Type irq_no)
{
	EXTI->IMR|=1<<(pin_no);//Enables the Interrupt at the EXTI side
	NVIC_EnableIRQ(irq_no);//Enables the interrupt at the NVIC side
}

void hal_gpio_clear_interrupt(uint16_t pin_no)
{
	if(EXTI->PR&(1<<pin_no))
		EXTI->PR|=1<<pin_no;
}
