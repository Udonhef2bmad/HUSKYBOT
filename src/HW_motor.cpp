#include "HW_motor.h"

Dir_config::Dir_config(
    double min_value = 0,
    double max_value = 1,
    double coefficient = 1,
    double threshold = 0) : min_value_(min_value),
                            max_value_(max_value),
                            coefficient_(coefficient),
                            threshold_(threshold)
{
}

HW_motor::HW_motor(
    char pin_dir_pos,
    char pin_dir_neg,
    HW_pwm hw_pwm,
    Dir_config config_pos = Dir_config(),
    Dir_config config_neg = Dir_config()) : pin_dir_pos_(pin_dir_pos),
                             pin_dir_neg_(pin_dir_neg),
                             hw_pwm_(hw_pwm),
                             config_pos_(config_pos),
                             config_neg_(config_neg)
{
    // initialise pins
    configure();
}

// maps a value from a given range to another - Credit: https://stackoverflow.com/questions/5731863
double d_map(double val, double input_min, double input_max, double output_min, double output_max)
{
    double slope = (output_max - output_min) / (input_max - input_min); // slope isn't affected by val and can be stored to avoid redundant calculations, see map_get_slope and map_slope
    return output_min + slope * (val - input_min);
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

    Dir_config *config;
    if(dir)
    {
        config = &config_pos_;
    }
    else
    {
        config = &config_neg_;
    }

    // constrain to [0;1]
    normalized_input = abs(normalized_input);

    if (normalized_input <= (*config).threshold_)
    {
        hw_pwm_.set_dutycycle(0.0);
        return;
    }

    // apply min and max value filter
    // normalized_input = d_map(normalized_input, 0, 1, min_value_, max_value_);

    // solution 1 : cut off (linear, loss of precision)
    // normalized_input = min(min_value_ + (normalized_input) * coefficient_, max_value_);

    // solution 2 : map (not linear, no loss of precision)
    normalized_input = d_map((normalized_input)*(*config).coefficient_, 0, (*config).coefficient_, (*config).min_value_, (*config).max_value_);

    hw_pwm_.set_dutycycle(normalized_input);

    Serial.print(F("  "));
    Serial.print(normalized_input);
    Serial.print(F("  "));
}