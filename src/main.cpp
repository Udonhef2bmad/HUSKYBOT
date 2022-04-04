#include <Arduino.h>

#include "HW_pwm.h"
#include "HW_motor.h"
#include "DifferentialDrive.h"

#include "husklens_utils.h"

// Misc

int16_t xCenter = 160;

HUSKYLENS huskylens;
HUSKYLENSResult result;

// Differential drive initialisations
HW_pwm pwm1 = HW_pwm(25, 1, 16, 50);
HW_motor motor1 = HW_motor(32, 33, pwm1);

HW_pwm pwm2 = HW_pwm(14, 2, 16, 50);
HW_motor motor2 = HW_motor(26, 27, pwm2);

DifferentialDrive dfDrive = DifferentialDrive(motor1, motor2);

// Timer
struct Timer
{
    unsigned long start;
    unsigned long delay;
};

void set_timer_start(struct Timer *timer)
{
    (*timer).start = millis();
}

void set_timer_delay(struct Timer *timer, unsigned long delay)
{
    (*timer).delay = delay;
}

long elapsed_time(struct Timer timer)
{
    return (millis() - timer.start);
}

long remaining_time(struct Timer timer)
{
    // Serial.println(F("millis() : timer.start"));
    // Serial.print(millis());
    // Serial.print(F(" : "));
    // Serial.println(timer.start);
    return (timer.delay - elapsed_time(timer));
}

bool hasDelayPassed(struct Timer timer)
{
    if (millis() - timer.start >= timer.delay)
    {
        return true;
    }
    return false;
}

// States
enum State
{
    RACE_INIT,
    RED_LIGHT,
    WP_SEARCH,
    WP_MOVE,
    WP_REACHED,
    FINISH,
};
enum State state;

// maps a value from a given range to another - Credit: https://stackoverflow.com/questions/5731863
double d_map(double input_min, double input_max, double output_min, double output_max, double val)
{
    double slope = (output_max - output_min) / (input_max - input_min); // slope isn't affected by val and can be stored to avoid redundant calculations, see map_get_slope and map_slope
    return output_min + slope * (val - input_min);
}

//
int16_t target_height = 100;
void advance(HUSKYLENSResult result)
{
    bool debug = 0;

    double coef_angular = 2; // perfect
    double coef_linear = 5;  // tweakable

    double angular = coef_angular * (result.xCenter - xCenter);
    double linear = coef_linear * (target_height - result.height);

    if (debug)
    {
        Serial.println(F("linear1 : angular1"));
        Serial.print(linear);
        Serial.print(F(" : "));
        Serial.println(angular);
    }

    double angular_range = 255;
    double linear_range = 255;

    angular = range_cut(angular, -angular_range, angular_range);
    linear = range_cut(linear, -linear_range, linear_range);

    if (debug)
    {
        Serial.println(F("linear2 : angular2"));
        Serial.print(linear);
        Serial.print(F(" : "));
        Serial.println(angular);
    }

    // map angular and linear to normalized [-1:1] values
    angular = angular / angular_range;
    linear = linear / linear_range;

    if (debug)
    {
        Serial.println(F("linear3 : angular3"));
        Serial.print(linear);
        Serial.print(F(" : "));
        Serial.println(angular);
    }

    dfDrive.control(linear, angular);
}

int target_tag;
void race_init() // swap algorithm mode
{
    Serial.println(F("RACE_INIT"));

    huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING); // look for green light
    dfDrive.control(0, 0);                                 // stop
    Serial.println(F("Transitionning to RED_LIGHT"));
    state = RED_LIGHT;
}

int green_light_id = 1;
bool scan_dir;
void red_light() // light is red; wait for it to be green
{
    // delay(3000);
    result = getResult(&huskylens);
    if (!isResultValid(result))
        return;

    if (green_light_id == getResultID(result))
    {
        target_tag = 1;                                      // set target tag to 1
        scan_dir = 1;                                        // default turning direction right
        huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION); // look for tag
        Serial.println(F("Transitionning to WP_SEARCH"));
        Serial.print(F("Target : "));
        Serial.println(target_tag);
        state = WP_SEARCH; // go and search first tag
    }
}

struct Timer lost_timeout;
void wp_search() // turn around and search for a target waypoint
{
    double scanspeed = 0;
    long inactive_time = 1000;
    long active_time = 150;

    if (millis() % (inactive_time + active_time) > inactive_time)
    {
        scanspeed = 1;
    }

    if (scan_dir)
    {
        dfDrive.control(0, -scanspeed);
    }
    else
    {
        dfDrive.control(0, scanspeed);
    }

    result = getResult(&huskylens);
    if (!isResultValid(result))
        return;

    if (getResultID(result) == target_tag)
    {
        scan_dir = get_side(result); // remember last side the target was on

        set_timer_start(&lost_timeout);      // set timer for timeout
        set_timer_delay(&lost_timeout, 100); // timeout waits 100ms before image is considered lost

        dfDrive.control(0, 0);

        Serial.println(F("Transitionning to WP_MOVE"));
        Serial.print(F("Target : "));
        Serial.println(target_tag);
        state = WP_MOVE;
    }
}

void wp_move() // move towards the waypoint until reached or lost
{
    result = getResult(&huskylens);
    if (!isResultValid(result))
    {
        if (hasDelayPassed(lost_timeout)) // abort move if timeout is reached
        {
            dfDrive.control(0, 0);
            Serial.println(F("Timeout - lost target"));
            Serial.println(F("Transitionning to WP_SEARCH"));
            state = WP_SEARCH;
        }
        return;
    }

    if (getResultID(result) == target_tag)
    {
        scan_dir = get_side(result); // remember last side the target was on

        set_timer_start(&lost_timeout); // reset timeout

        advance(result); // move toward target

        if (is_in_range(result, target_height, 5))
        {
            dfDrive.control(0, 0);
            Serial.println(F("Transitionning to WP_REACHED"));
            state = WP_REACHED;
        }
    }
}

void wp_reached() // reached wapypoint;  select new waypoint or finish race
{
    Serial.print(F("Reached Target ["));
    Serial.print(target_tag);
    Serial.println(F("]"));

    int last_tag = 5; // tag that indicates race end
    if (target_tag < last_tag)
    {
        target_tag++; // increment tag id
        Serial.print(F("New target ["));
        Serial.print(target_tag);
        Serial.println(F("]"));
        Serial.println(F("Transitionning to WP_SEARCH"));
        state = WP_SEARCH; // search new waypoint
    }
    else
    {
        Serial.println(F("Last Target Reached"));
        Serial.println(F("Transitionning to FINISH"));
        state = FINISH; // finish race
    }
}

void finish() // celebrate and look for another red light
{
    Serial.println(F("Wooh we made it!"));
    /*rick_setup(); // run once
    while (1)
    {
        rick_loop();
    }*/
    state = RACE_INIT;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println(F("Serial OK"));

    Wire.begin();
    Serial.println(F("Wire OK"));

    while (!huskylens.begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS "
                         "(General Settings>>Protocol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
    Serial.println(F("huskylens OK"));

    state = RACE_INIT;
}

void loop()
{
    switch (state)
    {
    case RACE_INIT:
        race_init();
        break;

    case RED_LIGHT:
        red_light();
        break;

    case WP_SEARCH: // search next waypoint
        wp_search();
        break;

    case WP_MOVE: // attempt to reach waypoint
        wp_move();
        break;

    case WP_REACHED: // waypoint reached : go to next waypoint or finish race
        wp_reached();
        break;

    case FINISH: // race finished : do something flashy
        finish();
        break;
    }
}