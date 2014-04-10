#include <Servo.h>

#define MAX_SIGNAL 2000
#define SPIN_SIGNAL 1200
#define MIN_SIGNAL 700
#define MOTOR_PIN_1 9 // Motor 1
#define MOTOR_PIN_2 10 // Motor 2
#define MOTOR_PIN_3 11 // Motor 3
#define MOTOR_PIN_4 12 // Motor 4

// ESC wiring, white to pin 9, black to ground

Servo motor_1;
Servo motor_2;
Servo motor_3;
Servo motor_4;

void setup() {
    Serial.begin(9600);
    Serial.println("Program begin...");
    
    motor_1.attach(MOTOR_PIN_1);
    motor_2.attach(MOTOR_PIN_2);
    motor_3.attach(MOTOR_PIN_3);
    motor_4.attach(MOTOR_PIN_4);
    
    // Send min output (used to initialise ESC)
    Serial.println("Sending minimum output");
    motor_1.writeMicroseconds(MIN_SIGNAL);
    motor_2.writeMicroseconds(MIN_SIGNAL);
    motor_3.writeMicroseconds(MIN_SIGNAL);
    motor_4.writeMicroseconds(MIN_SIGNAL);
}

void loop() {  
    // Wait for input
    while (Serial.available() > 0) {
        int speed = Serial.parseInt();
        Serial.println("Now spinning at " + (String)speed);
        // Set motor to specified speed
        motor_1.writeMicroseconds(speed);
        motor_2.writeMicroseconds(speed);
        motor_3.writeMicroseconds(speed);
        motor_4.writeMicroseconds(speed);
    }
}
