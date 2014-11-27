#include "Motor.h"

#include <Arduino.h>
#include <Servo.h>

// --- Motor values
static const int MAX_SIGNAL = 1000;
static const int MIN_SIGNAL = 750;
static const int INIT_SIGNAL = 700;
static const float MOTOR_ACCEL = 80.0f; // accel change per second
#define MOTOR_PIN_1 9 // Motor 1
#define MOTOR_PIN_2 10 // Motor 2
#define MOTOR_PIN_3 11 // Motor 3
#define MOTOR_PIN_4 12 // Motor 4
#define NUM_MOTORS 2

// Store actual motor speeds
unsigned long motor_speed_actual[NUM_MOTORS];
unsigned long motor_speed_set[NUM_MOTORS];
// Servos that are written to
Servo motor_device[NUM_MOTORS];

//int motors_running = 0;
//int motor_0_values[] = { 700, 740, 780, 750, 800 };
//int motor_1_values[] = { 700, 740, 780, 800, 750 };

int write_to_motor( bool force = false );

void MotorSetup( State &s ) {
    // zero all motor speeds
    memset(motor_speed_actual, 0, NUM_MOTORS * sizeof(int));
    memset(motor_speed_set, 0, NUM_MOTORS * sizeof(int));

    // Initialise the motor speeds
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_speed_actual[i] = 0.0f;
        motor_speed_set[i] = INIT_SIGNAL;
    }

    motor_device[0].attach(MOTOR_PIN_1);
    motor_device[1].attach(MOTOR_PIN_2);

    // Send min output (used to initialise ESC)
    /*Serial.println("Sending minimum output");
    for(int i = 0; i < NUM_MOTORS; i++) {
        //motor[i].writeMicroseconds(motor_speed[i]);
    }*/

    // Send the min signal to the motor
    write_to_motor(true);
}

#define MIN(a,b) a < b ? a : b
#define MAX(a,b) a > b ? a : b

void MotorUpdate( float frame_delta,  State &s ) {
    //Serial.println("MotorUpdate");

    int motor_id = 0;
    // fix the desired value if it's out of bounds
    if ( s.desired_motor_value != motor_speed_set[motor_id] )
    {
        int uncapped_desired_pwm = s.desired_motor_value;
        s.desired_motor_value = MIN(s.desired_motor_value,MAX_SIGNAL);
        s.desired_motor_value = MAX(s.desired_motor_value,MIN_SIGNAL);
        if ( uncapped_desired_pwm != s.desired_motor_value )
        {
            Serial.println("Limiting speed: Requested " + (String)uncapped_desired_pwm + " Setting " + s.desired_motor_value);
        }
        motor_speed_set[motor_id] = s.desired_motor_value;
    }

    // todo: loop all motors
    if ( motor_speed_set[motor_id] != (int)motor_speed_actual[motor_id] )
    {
        Serial.println("Motor speed needs adjusting.");
        // todo: Set actual using frame_delta and motor_speed_set

        Serial.println("actual was " + (String) (int)(motor_speed_actual[motor_id]*100));
        Serial.println("set is " + (String) (int)(motor_speed_set[motor_id]*100));
        double diff = motor_speed_actual[motor_id] - motor_speed_set[motor_id];
        double abs_diff = abs(diff);
        double max_change = frame_delta * MOTOR_ACCEL;
        abs_diff = MIN(abs_diff, max_change);
        Serial.println("frame_delta: " + (int)(frame_delta*100));
        Serial.println("diff " + (String)(int)(diff*100) + ", abs_diff: " + (String)(int)(abs_diff*100) + ", max_change: " + (String)(int)(max_change*100));

        if ( diff > 0 ) motor_speed_actual[motor_id] -= abs_diff;
        else motor_speed_actual[motor_id] += abs_diff;
        Serial.println("actual is now " + (String) (int)(motor_speed_actual[motor_id]*100));

        write_to_motor();
    }
}


int write_to_motor( bool force ) {

    for ( int i = 0; i < NUM_MOTORS; i++ )
    {
        if ( motor_speed_set[i] == (int)motor_speed_actual[i] ) continue;

        {
            char buf[128];
            sprintf( buf, "Writing to motor %d: %d", i, (int)(motor_speed_actual[i]*100));
            Serial.println(buf);
        }

        //motor_device[i].writeMicroseconds((int)motor_speed_actual[i]);
        Serial.println("MOTOR["+(String)i+"] SET " + (String)(int)(motor_speed_actual[i]*100));
        motor_speed_actual[i] = motor_speed_set[i];
    }

    return 0;
}