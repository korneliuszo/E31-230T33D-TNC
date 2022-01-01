/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "hwr.h"
#include "gpio.h"
#include "pathsel.h"
#include "timer.h"
#include "uart.h"
#include "radio.h"

uint8_t tx_buff[256];

/////////////////
//    main routine
/////////////////
void main (void) {

	DISABLE_INTERRUPTS();

	// switch to 16MHz (default is 2MHz)
	sfr_CLK.CKDIVR.byte = 0x00;

	pathsel_init();

	gpio_setup(&LED_PORT, LED_PIN, true, true, true); // output,push-pull,fast
	gpio_set(&LED_PORT, LED_PIN, 1);

	timer_start();

	uart_start();

	ENABLE_INTERRUPTS();

	radio_start();

	Timer led_blink;
	timer_init(&led_blink,500);

	bool in_tx = false;

	// main loop
	while(1) {

		if(timer_async(&led_blink))
		{
			// toggle LED. Pass port struct as pointer
			gpio_toggle(&LED_PORT, LED_PIN);
		}
		radio_rx_data();
		if (in_tx)
		{
			//TODO: carrier detection
			uint8_t byte;
			uint16_t len=0;
			while(uart_get(&byte))
			{
				tx_buff[len++]=byte;
			}
			pathsel_tx();
			radio_tx_on();
			radio_tx_data_packet(tx_buff,len);
			radio_rx_on();
			pathsel_rx();
			in_tx = false;
		}
		else if(uart_has_packet())
		{
			uint8_t byte;
			if(uart_get(&byte))
			{
				switch(byte)
				{
				case 0x00:
					in_tx = true;
					break;
				case 0x07:
					uart_put(0x07);
					while(uart_get(&byte))
						uart_put(byte);
					uart_put_fend();
					break;
				default:
					while(uart_get(&byte)); // eat rest of packet
					break;
				}
			}
		}
	} // main loop

} // main


ISR_HANDLER(TIM4_UPD_ISR, _TIM4_UIF_VECTOR_);

/// USART transmit ISR
ISR_HANDLER(USART1_TXE_ISR, _USART_T_TXE_VECTOR_);

/// USART receive ISR
ISR_HANDLER(USART1_RXNE_ISR, _USART_R_RXNE_VECTOR_);

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
