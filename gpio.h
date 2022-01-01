/*
 * gpio.h
 *
 *  Created on: Dec 27, 2021
 *      Author: kosa
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "hwr.h"
#include "bits.h"
#include <stdbool.h>

#pragma save
#pragma disable_warning 110
#pragma disable_warning 126
inline void gpio_setup(PORT_t * port, uint8_t pins, bool DDR, bool CR1, bool CR2)
{
	if (DDR)
		SBI(&port->DDR.byte,pins);
	else
		CBI(&port->DDR.byte,pins);
	if (CR1)
		SBI(&port->CR1.byte,pins);
	else
		CBI(&port->CR1.byte,pins);
	if (CR2)
		SBI(&port->CR2.byte,pins);
	else
		CBI(&port->CR2.byte,pins);
}

inline void gpio_set(PORT_t * port, uint8_t pins, bool val)
{
	if (val)
		SBI(&port->ODR.byte,pins);
	else
		CBI(&port->ODR.byte,pins);
}
#pragma restore

inline void gpio_toggle(PORT_t * port, uint8_t pins)
{
	port->ODR.byte^=pins;
}

inline uint8_t gpio_get(PORT_t * port, uint8_t pins)
{
	return port->IDR.byte & pins;
}

#endif /* GPIO_H_ */
