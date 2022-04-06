#include <Arduino.h>

#include "HW_pwm.h"
#include "HW_motor.h"
#include "DifferentialDrive.h"

#include "timer.h"
#include "PID.h"

#include "husklens_utils.h"

/// tweakable values

long lost_timeout_delay = 200; // timeout delay after target is lost (ms)

int16_t target_height = 130;          // target height; determines how close to the object the robot has to stop
int16_t target_height_margin = 50; // allows for a certain range of inaccuracy when approaching the target

// reaching
//PID angular_PID = PID(.5, 0, 0);
//PID linear_PID = PID(1.2, 0, 0);
 
//PID angular_PID = PID(.05, 0, 0);
//PID linear_PID = PID(.8, 0, 0);

PID angular_PID = PID(.04, 0, 0);
PID linear_PID = PID(1, 0, 0);

//PID angular_PID = PID(.02, 0, 2);//save1
//PID linear_PID = PID(1, 0, 0);

double angular_range = 255; //
double linear_range = 255;  //

// scanning
long inactive_time = 1000; // time for camera stabilisation
// long active_time = 140;    // time to turn : 5V
long active_time = 150; // time to turn : 12V

//double active_scan_speed = 0;
double active_scan_speed = .3; //angular speed during active scan turn

int last_tag = 5; // tag that indicates race end

// motor config

double left_motor_coef = 1;  // left motor coefficient : 12V
double right_motor_coef = 1; // right motor coefficient : 12V

// test config
// Dir_config(min, max, coef, threshold);
/*Dir_config config_pos_left =  Dir_config(0, .5, left_motor_coef, 0);
Dir_config config_neg_left =  Dir_config(0, .5, left_motor_coef, 0);
Dir_config config_pos_right = Dir_config(0, .5, right_motor_coef, 0);
Dir_config config_neg_right = Dir_config(0, .5, right_motor_coef, 0);//*/

// 5V config
// Dir_config(min, max, coef, threshold);
Dir_config config_pos_left =  Dir_config(0, 1, left_motor_coef, 0);
Dir_config config_neg_left =  Dir_config(0, 1, left_motor_coef, 0);
Dir_config config_pos_right = Dir_config(0, 1, right_motor_coef, 0);
Dir_config config_neg_right = Dir_config(0, 1, right_motor_coef, 0);

// 12V config
// Dir_config(min, max, coef, threshold);
 //Dir_config config_pos_left = Dir_config(0, 1.0, left_motor_coef, 0);
 //Dir_config config_neg_left = Dir_config(0, 1.0, left_motor_coef, 0);
 //Dir_config config_pos_right = Dir_config(0.20, 1.0, right_motor_coef, 0);
 //Dir_config config_neg_right = Dir_config(0.20, 1.0, right_motor_coef, 0);

/// Misc

// buzzes when checkpoint is reached
HW_pwm waypoint_buzzer = HW_pwm(2, 3, 16, 50); // apparently tone doesn't exist on the esp, so we are using a pwm pin.
int16_t xCenter = 160;

HUSKYLENS huskylens;
HUSKYLENSResult result;

// Differential drive initialisations
HW_pwm pwm1 = HW_pwm(25, 1, 8, 40);
HW_motor motor1 = HW_motor(32, 33, pwm1, config_pos_left, config_neg_left); // left motor

HW_pwm pwm2 = HW_pwm(14, 4, 8, 40);//pwm channel 2 seems damaged?
HW_motor motor2 = HW_motor(26, 27, pwm2, config_pos_right, config_neg_right); // right motor

DifferentialDrive dfDrive = DifferentialDrive(motor1, motor2);

/// States
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

// returns +1 if positive of -1 if negative
double getSign(double value)
{
    if (value > 0)
    {
        return 1;
    }
    return -1;
}

int16_t get_angular_error(HUSKYLENSResult result)
{
    return (result.xCenter - xCenter);
}

int16_t get_linear_error(HUSKYLENSResult result)
{
    return (target_height - result.height);
}
void update_move(int16_t angular_error, int16_t linear_error)
{
    bool debug = 1;

    if (debug)
    {
        Serial.println(F("\n\nError"));
        Serial.print(linear_error);
        Serial.print(F(" : "));
        Serial.println(angular_error);
    }

    double linear = linear_PID.getOutput(-linear_error);
    double angular = angular_PID.getOutput(-angular_error);

    // double linear = 0;
    // double angular = .005 * pow(angular_error,2) * getSign(angular_error);

    if (debug)
    {
        Serial.println(F("PID"));
        Serial.print(linear);
        Serial.print(F(" : "));
        Serial.println(angular);
    }

    angular = range_cut(angular, -angular_range, angular_range);
    linear = range_cut(linear, -linear_range, linear_range);

    if (debug)
    {
        Serial.println(F("Cut"));
        Serial.print(linear);
        Serial.print(F(" : "));
        Serial.println(angular);
    }

    // map angular and linear to normalized [-1:1] values
    angular = angular / angular_range;
    linear = linear / linear_range;

    if (debug)
    {
        Serial.println(F("Map"));
        Serial.print(linear);
        Serial.print(F(" : "));
        Serial.println(angular);
    }

    dfDrive.control(linear, angular);
    delay(100);
}

int target_tag;
void race_init() // swap algorithm mode
{
    huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING); // look for green light
    dfDrive.control(0, 0);                               // stop
    Serial.println(F("Transitionning to RED_LIGHT"));
    state = RED_LIGHT;
}

int green_light_id = 1;
bool scan_dir;
void red_light() // light is red; wait for it to be green
{
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

        // test motors
        //dfDrive.test_motor_threshold(.01, 500); // calibrate min threshold
        //dfDrive.test_motor_drift(.3, 500);
    }
}

struct Timer lost_timeout;
int16_t angular_error;
int16_t linear_error;
void wp_search() // turn around and search for a target waypoint
{
    double scan_speed = 0;

    if (millis() % (inactive_time + active_time) > inactive_time)
    {
        scan_speed = active_scan_speed;
    }

    if (scan_dir)
    {
        dfDrive.control(0, scan_speed);
    }
    else
    {
        dfDrive.control(0, -scan_speed);
    }

    result = getResult(&huskylens);
    if (!isResultValid(result))
        return;

    if (getResultID(result) == target_tag)
    {
        scan_dir = get_side(result); // remember last side the target was on

        set_timer_start(&lost_timeout);                     // set timer for timeout
        set_timer_delay(&lost_timeout, lost_timeout_delay); // timeout waits 100ms before image is considered lost

        dfDrive.control(0, 0);

        // reset errors
        angular_PID.reset();
        linear_PID.reset();
        angular_error = 0;
        linear_error = 0;

        Serial.println(F("Transitionning to WP_MOVE"));
        Serial.print(F("Target : "));
        Serial.println(target_tag);
        state = WP_MOVE;
    }
}

void wp_move() // move towards the waypoint until reached or lost
{
    result = getResult(&huskylens);
    if (isResultValid(result) && getResultID(result) == target_tag)
    {
        scan_dir = get_side(result); // remember last side the target was on

        set_timer_start(&lost_timeout); // reset timeout

        // advance(result); // move toward target
        linear_error = get_linear_error(result);
        angular_error = get_angular_error(result);

        if (is_close(result, target_height, target_height_margin))
        {
            dfDrive.control(0, 0);
            Serial.println(F("Transitionning to WP_REACHED"));
            state = WP_REACHED;
            return;
        }
    }
    else
    {
        angular_PID.reset();
        linear_PID.reset();
        if (hasDelayPassed(lost_timeout)) // abort move if timeout is reached
        {
            Serial.println(F("Timeout - lost target"));
            Serial.println(F("Transitionning to WP_SEARCH"));
            state = WP_SEARCH;
        }
    }
    // update
    update_move(angular_error, linear_error);
}

void wp_reached() // reached wapypoint;  select new waypoint or finish race
{
    Serial.print(F("Reached Target ["));
    Serial.print(target_tag);
    Serial.println(F("]"));


    if (target_tag < last_tag)
    {
        waypoint_buzzer.set_frequency(900);
        waypoint_buzzer.set_dutycycle(.5);
        delay(250);
        waypoint_buzzer.set_frequency(1000);
        waypoint_buzzer.set_dutycycle(.5);
        delay(400);
        waypoint_buzzer.set_dutycycle(0.0);
        
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
    dfDrive.control(0, 1);
    waypoint_buzzer.set_frequency(1000);
    waypoint_buzzer.set_dutycycle(.5);
    delay(200);
    waypoint_buzzer.set_frequency(0);
    waypoint_buzzer.set_dutycycle(.5);
    delay(100);
    waypoint_buzzer.set_frequency(1000);
    waypoint_buzzer.set_dutycycle(.5);
    delay(200);
    waypoint_buzzer.set_frequency(0);
    waypoint_buzzer.set_dutycycle(.5);
    delay(100);
    waypoint_buzzer.set_frequency(1000);
    waypoint_buzzer.set_dutycycle(.5);
    delay(200);
    waypoint_buzzer.set_frequency(0);
    waypoint_buzzer.set_dutycycle(.5);
    dfDrive.control(0, -1);
    delay(100);
    waypoint_buzzer.set_frequency(900);
    waypoint_buzzer.set_dutycycle(.5);
    delay(600);
    waypoint_buzzer.set_frequency(0);
    waypoint_buzzer.set_dutycycle(.5);
    delay(100);
    waypoint_buzzer.set_frequency(1000);
    waypoint_buzzer.set_dutycycle(.5);
    delay(1200);
    waypoint_buzzer.set_dutycycle(0.0);
    dfDrive.control(0, 0);

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

    // init buzzer pin
    waypoint_buzzer.set_frequency(900);
    waypoint_buzzer.set_dutycycle(.5);
    delay(250);
    waypoint_buzzer.set_frequency(1000);
    waypoint_buzzer.set_dutycycle(.5);
    delay(400);
    waypoint_buzzer.set_dutycycle(0.0);

    Serial.println(F("Transitionning to RACE_INIT"));
    state = RACE_INIT;
}

void loop()
{
    // delay(100);
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