//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#ifndef TIMER_H_
#define TIMER_H_

#include "cmsis_device.h"
#include <stdlib.h>

// ----------------------------------------------------------------------------

#define TIMER_FREQUENCY_HZ (1000u)

typedef uint32_t timer_ticks_t;

uint32_t uwTick;

extern volatile timer_ticks_t timer_delayCount;

//extern void
//timer_start (void);

extern void
timer_sleep (timer_ticks_t ticks);

extern void HAL_InitTick(void);

extern uint32_t HAL_GetTick(void);

// ----------------------------------------------------------------------------

#endif // TIMER_H_
