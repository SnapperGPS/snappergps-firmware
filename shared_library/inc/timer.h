/****************************************************************************
 * timer.h
 * Snapper GPS
 * November 2020
 *****************************************************************************/

#ifndef __TIMER_H
#define __TIMER_H

void Timer_enable(void);

void Timer_disable(void);

void Timer_delayMilliseconds(uint32_t milliseconds);

void Timer_delayMicroseconds(uint32_t microseconds);

#endif /* __TIMER_H */