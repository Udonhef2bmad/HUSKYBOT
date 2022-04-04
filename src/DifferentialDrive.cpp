#include "DifferentialDrive.h"

// cut input to a certain range
double range_cut(double input, double min_range, double max_range)
{
    return max(min(input, max_range), min_range);
}


DifferentialDrive::DifferentialDrive(
    HW_motor left_motor,
    HW_motor right_motor) : left_motor_(left_motor),
                         right_motor_(right_motor)
{
}

// update differential drive using a pair of normalized double [-1:1]
void DifferentialDrive::control(double linear, double angular)
{
    //Serial.println("linear : angular");
    //Serial.print(linear);
    //Serial.print(" : ");
    //Serial.println(angular);

    double left_output, right_output;

    // prioritize angular velocity : cut off excess linear velocity
    double left_excess, right_excess;

    left_excess = max(0.0, -1 + (linear + angular));
    right_excess = -min(0.0, 1 + (linear + angular));

    left_output = linear + angular;
    right_output = linear - angular;

    right_output = right_output - left_excess + right_excess;
    
    // original
    // left_output = linear + angular;
    // right_output = linear - angular;

    // cut off excess
    left_output = range_cut(left_output, -1.0, 1.0);
    right_output = range_cut(right_output, -1.0, 1.0);
    

    left_motor_.update(left_output);   // update left motor
    right_motor_.update(right_output); // update right motor

    //Serial.println("left_output : right_output");
    //Serial.print(left_output);
    //Serial.print(" : ");
    //Serial.println(right_output);
}