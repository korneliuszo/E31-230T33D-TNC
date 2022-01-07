/*
 * hwr.h
 *
 *  Created on: Dec 27, 2021
 *      Author: kosa
 */

#ifndef HWR_H_
#define HWR_H_

#include "STM8_headers/include/STM8L151G6.h"
#define LED_PORT   sfr_PORTD
#define LED_PIN    PIN0

#define M0_PORT   sfr_PORTC
#define M0_PIN    PIN5

#define VCO_EN_PORT sfr_PORTB
#define VCO_EN_PIN PIN2

#define TX_EN_PORT sfr_PORTB
#define TX_EN_PIN PIN1

#define RX_EN_PORT sfr_PORTB
#define RX_EN_PIN PIN0

#define RX_LNA_EN_PORT sfr_PORTD
#define RX_LNA_EN_PIN PIN3

#define PA_V_EN_PORT sfr_PORTC
#define PA_V_EN_PIN PIN2

#define PA_BIAS_EN_PORT sfr_PORTC
#define PA_BIAS_EN_PIN PIN3

#define UART_TX_PORT sfr_PORTA
#define UART_TX_PIN PIN2

#define UART_RX_PORT sfr_PORTA
#define UART_RX_PIN PIN3

#define SPI_MOSI_PORT sfr_PORTB
#define SPI_MOSI_PIN PIN6

#define SPI_MISO_PORT sfr_PORTB
#define SPI_MISO_PIN PIN7

#define SPI_SCK_PORT sfr_PORTB
#define SPI_SCK_PIN PIN5

#define SPI_SS_PORT sfr_PORTD
#define SPI_SS_PIN PIN4

#define F_CLK 26000000UL
#define XTALDIV 2

#define DUMP_REGS
//#define DUMP_REGS_READ

#endif /* HWR_H_ */
