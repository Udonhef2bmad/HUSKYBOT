#include "PID.h"

// PID constants

PID::PID(
    double p_gain = 2,
    double i_gain = 5,
    double d_gain = 1) : p_gain_(p_gain),
                         i_gain_(i_gain),
                         d_gain_(d_gain)
{
    setpoint_ = 0;
    i_error_ = 0;
    previousTime_ = 0;
}

// PID implementation found on https://www.teachmemicro.com/arduino-pid-control-tutorial/
/*double PID::getOutput(double input)
{

    currentTime_ = millis();                               // get current time
    elapsedTime_ = (double)(currentTime_ - previousTime_); // compute time elapsed from previous computation

    error_ = setpoint_ - input;                      // determine error
    i_error_ += error_ * elapsedTime_;               // compute integral
    d_error_ = (error_ - lastError_) / elapsedTime_; // compute derivative

    double out = p_gain_ * error_ + i_gain_ * i_error_ + d_gain_ * d_error_; // PID output

    lastError_ = error_;          // remember current error
    previousTime_ = currentTime_; // remember current time

    return out; // have function return the PID output
}*/

double PID::getOutput(double input)
{
     currentTime_ = millis();                               // get current time
    elapsedTime_ = (double)(currentTime_ - previousTime_); // compute time elapsed from previous computation

    error_ = setpoint_ - input;                      // determine error
    i_error_ += error_ * elapsedTime_;               // compute integral
    d_error_ = (error_ - lastError_) / elapsedTime_; // compute derivative

    if(error_ * lastError_ < 0)//check if input changed sign
    {
        i_error_ = 0;
    }

    double out = p_gain_ * error_ + i_gain_ * i_error_ + d_gain_ * d_error_; // PID output

    lastError_ = error_;          // remember current error
    previousTime_ = currentTime_; // remember current time

    return out; // have function return the PID output
}

//reset pid before usage
void PID::reset()
{
    i_error_ = 0;
    previousTime_ = millis()-1;
}


void PID::set_setPoint(double setpoint)
{
    setpoint_ = setpoint;
}