#ifndef HW_PWM_H
#define HW_PWM_H

#include <Arduino.h>

class HW_pwm
{
public:
    void set_dutycycle(double normalized_dutycycle);
    void set_frequency(double frequency);

    HW_pwm(
        char pin,
        uint8_t channel,
        uint8_t resolution,
        double frequency);

private:
    char pin_;           // output pwm signal pin
    uint8_t channel_;    // pwm channel: 0-15 different channels
    uint8_t resolution_; // pwm resolution: 1-16 bits
    uint32_t max_dutycyle_;
    double frequency_; // pwm frequency:

    void configure();
    void set_dutycycle(uint32_t dutycycle);
};

#endif