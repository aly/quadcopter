#include <Servo.h>

#define MAX_SIGNAL 2000
#define SPIN_SIGNAL 1200
#define MIN_SIGNAL 700
#define MOTOR_PIN 9

// ESC wiring, white to pin 9, black to ground

Servo motor;

void setup() {
    Serial.begin(9600);
    Serial.println("Program begin...");
    
    motor.attach(MOTOR_PIN);
    
    // Send min output (used to initialise ESC)
    Serial.println("Sending minimum output");
    motor.writeMicroseconds(MIN_SIGNAL);
}

void loop() {  
    // Wait for input
    while (Serial.available() > 0) {
        int speed = Serial.parseInt();
        Serial.println("Now spinning at " + (String)speed);
        // Set motor to specified speed
        motor.writeMicroseconds(speed);
    }
}
