#ifndef HW_MOTOR_H
#define HW_MOTOR_H

#include <Arduino.h>
#include "HW_pwm.h"

class HW_motor
{
public:
    HW_motor(
        char pin_dir_pos,
        char pin_dir_neg,
        HW_pwm hw_pwm);

    void configure();
    void update(double normalized_input);

private:
    char pin_dir_pos_; // positive direction pin
    char pin_dir_neg_; // negative direction pin
    HW_pwm hw_pwm_;    // pwm signal pin & config
};

#endif