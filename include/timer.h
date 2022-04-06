#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

// Timer
struct Timer
{
    unsigned long start;
    unsigned long delay;
};

void set_timer_start(struct Timer *timer);
void set_timer_delay(struct Timer *timer, unsigned long delay);
long elapsed_time(struct Timer timer);
long remaining_time(struct Timer timer);
bool hasDelayPassed(struct Timer timer);

#endif