#ifndef HW_MOTOR_H
#define HW_MOTOR_H

#include <Arduino.h>
#include "HW_pwm.h"

class Dir_config
{
    public:
    Dir_config(
        double min_value,
        double max_value,
        double coefficient,
        double threshold
    );

    double min_value_; //minimal value to send to output 
    double max_value_; //maximal value to send to output
    double coefficient_; //coefficient applied to output
    double threshold_;  //value beyond which the motor starts turning 
};

class HW_motor
{
public:
    HW_motor(
        char pin_dir_pos,
        char pin_dir_neg,
        HW_pwm hw_pwm,
        Dir_config config_pos,
        Dir_config config_neg
        );

    void configure();
    void update(double normalized_input);

private:
    char pin_dir_pos_; // positive direction pin
    char pin_dir_neg_; // negative direction pin

    HW_pwm hw_pwm_;    // pwm signal pin & config
    Dir_config config_pos_; //positive direction configuration
    Dir_config config_neg_; //negative direction configuration
};

#endif