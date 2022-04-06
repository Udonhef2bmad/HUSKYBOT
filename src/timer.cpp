#include "timer.h"

void set_timer_start(struct Timer *timer)
{
    (*timer).start = millis();
}

void set_timer_delay(struct Timer *timer, unsigned long delay)
{
    (*timer).delay = delay;
}

long elapsed_time(struct Timer timer)
{
    return (millis() - timer.start);
}

long remaining_time(struct Timer timer)
{
    // Serial.println(F("millis() : timer.start"));
    // Serial.print(millis());
    // Serial.print(F(" : "));
    // Serial.println(timer.start);
    return (timer.delay - elapsed_time(timer));
}

bool hasDelayPassed(struct Timer timer)
{
    if (millis() - timer.start >= timer.delay)
    {
        return true;
    }
    return false;
}

