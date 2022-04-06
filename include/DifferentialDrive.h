#ifndef DIFFERENTIALDRIVE_H
#define DIFFERENTIALDRIVE_H

#include <Arduino.h>
#include "HW_motor.h"


double range_cut(double input, double min_range, double max_range);

class DifferentialDrive
{
public:
    DifferentialDrive(
        HW_motor left_motor,
        HW_motor right_motor);

    void control(double linear, double angular);
    void test_motor_threshold(double step, long ms_delay);
    void test_motor_drift(double speed, long ms_delay);
    
private:
    HW_motor left_motor_;
    HW_motor right_motor_;
};


#endif