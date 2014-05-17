#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#define MOTOR_PIN_1 9 // Motor 1
#define MOTOR_PIN_2 10 // Motor 2
#define MOTOR_PIN_3 11 // Motor 3
#define MOTOR_PIN_4 12 // Motor 4

// Store motor speeds
int motor_speed[4];

// ESC wiring, white to pin 9, black to ground

Servo motor[4];

void setup() {
    Serial.begin(9600);
    Serial.println("Program begin...");

    // Initialise the motor speeds
    for (int i = 0; i < 4; i++) {
        motor_speed[i] = MIN_SIGNAL;
    }

    motor[0].attach(MOTOR_PIN_1);
    motor[1].attach(MOTOR_PIN_2);
    motor[2].attach(MOTOR_PIN_3);
    motor[3].attach(MOTOR_PIN_4);
    
    // Send min output (used to initialise ESC)
    Serial.println("Sending minimum output");
    for(int i = 0; i < 4; i++) {
        motor[i].writeMicroseconds(MIN_SIGNAL);
    }
}

void loop() {  
    // Wait for input
    while (Serial.available() > 0) {
        int speed = Serial.parseInt();
        Serial.println("Now spinning at " + (String)speed);
        // Set motor to specified speed
        for (int i = 0; i < 4; i++) {
            int pwm = update_motor(i, speed);
            motor[i].writeMicroseconds(pwm);
        }
    }
}


int update_motor(int motor, int speed) {
    int pwm;

    // Safety code reduce danger when testing.
    if (speed > 10) {
        Serial.println("Safety error: speed increment too large!!");
        return motor_speed[motor];
    }

    if (motor_speed[motor] + speed > MAX_SIGNAL) {
        pwm = MAX_SIGNAL;
    } else if (motor_speed[motor] + speed < MIN_SIGNAL) {
        pwm = MIN_SIGNAL;
    } else {
        pwm = motor_speed[motor] + speed;
    }

    return pwm;
}

