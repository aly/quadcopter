#include "Motor.h"

#include <Arduino.h>
#include <Servo.h>

// --- Motor values
static const int MAX_SIGNAL = 1000;
static const int MIN_SIGNAL = 750;
static const int INIT_SIGNAL = 700;
static const float MOTOR_ACCEL = 10.0f; // accel change per second
#define MOTOR_PIN_1 9 // Motor 1
#define MOTOR_PIN_2 10 // Motor 2
#define MOTOR_PIN_3 11 // Motor 3
#define MOTOR_PIN_4 12 // Motor 4
#define NUM_MOTORS 2

// Store actual motor speeds
unsigned int motor_speed_actual[NUM_MOTORS];
unsigned int motor_speed_set[NUM_MOTORS];
// Servos that are written to
Servo motor_device[NUM_MOTORS];

// prototypes
int write_to_motor( bool force = false );

#define MIN(a,b) a < b ? a : b
#define MAX(a,b) a > b ? a : b
String ftos( float f )
{
    int integer = (int)f;
    String out = (String)integer;
    out += ".";
    int dp = (f * 100) - (integer*100);
    out += (String)dp;
    out += "f";
    return out;
}

void MotorSetup( State &s ) {
    Serial.println("MotorSetup");
    // zero all motor speeds
    memset(motor_speed_actual, 0, NUM_MOTORS * sizeof(int));
    memset(motor_speed_set, 0, NUM_MOTORS * sizeof(int));

    // Initialise the motor speeds
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_speed_actual[i] = INIT_SIGNAL;
        motor_speed_set[i] = INIT_SIGNAL;
    }

    motor_device[0].attach(MOTOR_PIN_1);
    motor_device[1].attach(MOTOR_PIN_2);

    // Send min output (used to initialise ESC)
    /*Serial.println("Sending minimum output");
    for(int i = 0; i < NUM_MOTORS; i++) {
        //motor[i].writeMicroseconds(motor_speed[i]);
    }*/

    // set state to minumum value motor speed for motor[0]
    s.desired_motor_zero_value = INIT_SIGNAL;

    // Send the min signal to the motor
    write_to_motor(true);
}

void MotorUpdate( float frame_delta,  State &s ) {
    //Serial.println("MotorUpdate");

    //
    // fix the desired value if it's out of bounds
    for ( int motor_id = 0; motor_id < NUM_MOTORS; ++motor_id )
    {
        if ( s.desired_motor_zero_value != motor_speed_set[motor_id] )
        {
            int uncapped_desired_pwm = s.desired_motor_zero_value;
            s.desired_motor_zero_value = MIN(s.desired_motor_zero_value,MAX_SIGNAL);
            s.desired_motor_zero_value = MAX(s.desired_motor_zero_value,MIN_SIGNAL);
            if ( uncapped_desired_pwm != s.desired_motor_zero_value )
            {
                Serial.println("Limiting speed: Requested " + (String)uncapped_desired_pwm + " Setting " + s.desired_motor_zero_value);
            }
            motor_speed_set[motor_id] = s.desired_motor_zero_value;
        }
    }

    // todo: loop all motors
    bool speed_updated = false;
    for ( int motor_id = 0; motor_id < NUM_MOTORS; ++motor_id )
    {
        if ( motor_speed_set[motor_id] != motor_speed_actual[motor_id] )
        {
            speed_updated = true;
            Serial.println("Motor speed needs adjusting.");
            // todo: Set actual using frame_delta and motor_speed_set

            Serial.println("actual was " + (String)motor_speed_actual[motor_id] );
            Serial.println("set is " + (String)motor_speed_set[motor_id] );
            float diff = (int)motor_speed_set[motor_id] - (int)motor_speed_actual[motor_id];
            float abs_diff = abs(diff);
            float max_change = frame_delta * MOTOR_ACCEL;
            float abs_diff_max = MIN(abs_diff, max_change);
            Serial.println("frame_delta: " + ftos(frame_delta) + "s" );
            Serial.println("diff " + ftos(diff) + ", abs_diff: " + ftos(abs_diff) + ", max_change: " + ftos(max_change) + "; abs_diff_max : " + ftos(abs_diff_max) );

            if ( diff > 0.0f )
            {
                Serial.println("PLUSTIME ");
                motor_speed_actual[motor_id] += abs_diff_max;
            }
            else
            {
                Serial.println("minUSTIME" );
                motor_speed_actual[motor_id] -= abs_diff_max;  
            }
            Serial.println("actual is now " + (String)motor_speed_actual[motor_id] );
        }
    }
    if ( speed_updated ) write_to_motor();
}


int write_to_motor( bool force ) {

    for ( int i = 0; i < NUM_MOTORS; i++ )
    {
        int new_value = (int)motor_speed_actual[i];
        new_value = MIN(new_value, MAX_SIGNAL);
        new_value = MAX(new_value, INIT_SIGNAL);

        motor_device[i].writeMicroseconds(new_value);
        Serial.println("MOTOR["+(String)i+"] ACTUAL IS NOW " + (String)motor_speed_actual[i] );
    }

    return 0;
}