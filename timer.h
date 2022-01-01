/*
 * timer.h
 *
 *  Created on: Dec 30, 2021
 *      Author: kosa
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdbool.h>
#include <stdint.h>

void timer_start(void);

typedef struct  {
	uint16_t duration;
	uint16_t last_sample;
} Timer;

uint16_t timer_get();

inline void timer_init(Timer *timer, uint16_t duration)
{
	timer->duration = duration;
	timer->last_sample = timer_get();
}

bool timer_async(Timer *timer);

#endif /* TIMER_H_ */
