#include "HW_pwm.h"

HW_pwm::HW_pwm(
        char pin,
        uint8_t channel = 1,
        uint8_t resolution = 16,
        double frequency = 40) : pin_(pin),
                                channel_(channel),
                                resolution_(resolution),
                                frequency_(frequency)
{
    configure();
    set_dutycycle((uint32_t) 0); //init with 0 dutycycle
}

// set dutycyle using a normalized double [0:1]
void HW_pwm::set_dutycycle(double normalized_dutycycle)
{
    set_dutycycle((uint32_t)(normalized_dutycycle * max_dutycyle_));
}

void HW_pwm::configure()
{
    pinMode(pin_, OUTPUT);
    ledcSetup(channel_, frequency_, resolution_);
    ledcAttachPin(pin_, channel_);
    max_dutycyle_ = pow(2, resolution_);
}

// set dutycycle directly
void HW_pwm::set_dutycycle(uint32_t dutycycle)
{
    ledcWrite(channel_, dutycycle);
}