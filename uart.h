/*
 * uart.h
 *
 *  Created on: Dec 30, 2021
 *      Author: kosa
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>

void uart_start(void);

bool uart_has_packet(void);
bool uart_get(uint8_t * byte);
void uart_put(uint8_t byte);
void uart_put_fend(void);
void uart_put_raw(uint8_t byte);

#endif /* UART_H_ */
