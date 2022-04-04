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

private:
    HW_motor left_motor_;
    HW_motor right_motor_;
};


#endif