/*
 * uart.c
 *
 *  Created on: Dec 30, 2021
 *      Author: kosa
 */

#include "hwr.h"
#include "uart.h"
#include "fifo.h"
#include "gpio.h"

DEFINE_STATIC_FIFO(uart_rx,512)
DEFINE_STATIC_FIFO(uart_tx,512)

#define FEND 0xC0
#define FESC 0xDB
#define TFEND 0xDC
#define TFESC 0xDD

static volatile uint8_t fends;

void uart_start(void)
{

	gpio_setup(&UART_TX_PORT, UART_TX_PIN, true, true, true); // output,push-pull,fast
	gpio_setup(&UART_RX_PORT, UART_RX_PIN, false, true, false); //input, pullup, no-int

	sfr_REMAP.SYSCFG_RMPCR1.USART1TR_REMAP = 0x01;

	sfr_CLK.PCKENR1.PCKEN15 = 1;

	uint16_t  val16;

	// set UART1 behaviour
	sfr_USART1.CR1.byte = sfr_USART1_CR1_RESET_VALUE;  // enable UART2, 8 data bits, no parity control
	sfr_USART1.CR2.byte = sfr_USART1_CR2_RESET_VALUE;  // no interrupts, disable sender/receiver
	sfr_USART1.CR3.byte = sfr_USART1_CR3_RESET_VALUE;  // no LIN support, 1 stop bit, no clock output(?)

	// set baudrate (note: BRR2 must be written before BRR1!)
	val16 = (uint16_t) (((uint32_t) 16000000L)/9600);
	sfr_USART1.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
	sfr_USART1.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);

	// enable transmission
	sfr_USART1.CR2.REN  = 1;  // enable receiver
	sfr_USART1.CR2.TEN  = 1;  // enable sender

	sfr_USART1.CR2.RIEN = 1;

}

/// USART transmit ISR
ISR_HANDLER(USART1_TXE_ISR, _USART_T_TXE_VECTOR_)
{
	uint8_t   data;

	// clearing of ISR flag not required for STM8
	sfr_USART1.SR.TXE = 0;

	if(fifo_get(&uart_tx,&data))
	{
		sfr_USART1.DR.byte = data;
	}
	else
	{
		sfr_USART1.CR2.TIEN = 0;
	}

}

/// USART receive ISR
ISR_HANDLER(USART1_RXNE_ISR, _USART_R_RXNE_VECTOR_)
{
	uint8_t   data;

	// clearing of ISR flag not required for STM8
	sfr_USART1.SR.RXNE = 0;

	// read byte from UART buffer
	data = sfr_USART1.DR.byte;

	// add a new byte to the FIFO buffer
	fifo_put(&uart_rx, data);
	if(data == FEND)
		fends++;
}

bool uart_has_packet(void)
{
	bool ret;
	DISABLE_INTERRUPTS();
	ret = !!fends;
	ENABLE_INTERRUPTS();
	return ret;
}

bool uart_get(uint8_t * byte)
{
	bool ret = true;
	DISABLE_INTERRUPTS();
	*byte=0x00;
	ret &= fifo_get(&uart_rx, byte);
	if (*byte == FESC)
	{
		ret &= fifo_get(&uart_rx, byte);
		if(*byte == TFEND)
			*byte = FEND;
		else
			*byte = FESC;
	}
	else if (*byte == FEND)
	{
		fends--;
		ret = false;
	}
	ENABLE_INTERRUPTS();
	return ret;
}

void uart_put(uint8_t byte)
{
	DISABLE_INTERRUPTS();
	if(byte == FEND)
	{
		fifo_put(&uart_tx,FESC);
		fifo_put(&uart_tx,TFEND);
	}
	else if (byte == FESC)
	{
		fifo_put(&uart_tx,FESC);
		fifo_put(&uart_tx,TFESC);
	}
	else
	{
		fifo_put(&uart_tx,byte);
	}
	sfr_USART1.CR2.TIEN = 1;
	ENABLE_INTERRUPTS();
}

void uart_put_fend(void)
{
	DISABLE_INTERRUPTS();
	fifo_put(&uart_tx,FEND);
	sfr_USART1.CR2.TIEN = 1;
	ENABLE_INTERRUPTS();
}

void uart_put_raw(uint8_t byte)
{
	DISABLE_INTERRUPTS();
	while(!fifo_put(&uart_tx,byte))
	{
		ENABLE_INTERRUPTS();
		NOP();
		DISABLE_INTERRUPTS();
	}
	sfr_USART1.CR2.TIEN = 1;
	ENABLE_INTERRUPTS();
}
