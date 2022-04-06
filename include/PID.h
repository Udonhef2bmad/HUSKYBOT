#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID
{
public:
    PID(
        double p_gain,
        double i_gain,
        double d_gain);
    void set_setPoint(double setpoint);
    double getOutput(double input);
    void reset();

private:
    double p_gain_;
    double i_gain_;
    double d_gain_;

    unsigned long currentTime_, previousTime_;
    double elapsedTime_;
    double lastError_;
    double input_, output_, setpoint_;
    double error_, i_error_, d_error_;

};

#endif