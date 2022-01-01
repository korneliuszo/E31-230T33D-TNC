/*
 * timer.c
 *
 *  Created on: Dec 30, 2021
 *      Author: kosa
 */

#include "hwr.h"
#include "timer.h"


void timer_start(void)
{
	sfr_CLK.PCKENR1.PCKEN12 = 1;

	// stop the timer
	sfr_TIM4.CR1.CEN = 0;

	// clear counter
	sfr_TIM4.CNTR.byte = 0x00;

	// auto-reload value buffered
	sfr_TIM4.CR1.ARPE = 1;

	// clear pending events
	sfr_TIM4.EGR.byte  = 0x00;

	// set clock to 16Mhz/2^6 = 250kHz -> 4us period
	sfr_TIM4.PSCR.PSC = 6;

	// set autoreload value for 1ms (=250*4us)
	sfr_TIM4.ARR.byte  = 250;

	// enable timer 4 interrupt
	sfr_TIM4.IER.UIE = 1;

	// start the timer
	sfr_TIM4.CR1.CEN = 1;
}

static volatile uint16_t timer_count = 0;


ISR_HANDLER(TIM4_UPD_ISR, _TIM4_UIF_VECTOR_)
{
	timer_count++;
	// clear timer 4 interrupt flag
	sfr_TIM4.SR1.UIF = 0;

}

uint16_t timer_get()
{
	uint16_t ret;
	DISABLE_INTERRUPTS();
	ret = timer_count;
	ENABLE_INTERRUPTS();
	return ret;
}

bool timer_async(Timer *timer)
{
	if(!timer->duration)
		return false;
	uint16_t now = timer_get();
	if((now-timer->last_sample) > timer->duration)
	{
		timer->last_sample = now;
		return true;
	}
	return false;
}
