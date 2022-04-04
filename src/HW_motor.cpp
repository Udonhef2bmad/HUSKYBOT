#include "HW_motor.h"

HW_motor::HW_motor(
    char pin_dir_pos,
    char pin_dir_neg,
    HW_pwm hw_pwm) : pin_dir_pos_(pin_dir_pos),
                     pin_dir_neg_(pin_dir_neg),
                     hw_pwm_(hw_pwm)
{
    // initialise pins
    configure();
}

void HW_motor::configure()
{
    pinMode(pin_dir_pos_, OUTPUT);
    pinMode(pin_dir_neg_, OUTPUT);
}

// update motor output using a normalized double [-1:1]
void HW_motor::update(double normalized_input)
{
    bool dir = (normalized_input < 0);
    digitalWrite(pin_dir_pos_, dir);
    digitalWrite(pin_dir_neg_, !dir);

    hw_pwm_.set_dutycycle(abs(normalized_input));
}