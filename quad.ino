#include <Servo.h>

#define MAX_SIGNAL 2000
#define SPIN_SIGNAL 1200
#define MIN_SIGNAL 700
#define MOTOR_PIN 8

// ESC wiring, white to pin 9, black to ground

Servo motor;

void setup() {
    Serial.begin(9600);
    Serial.println("Program begin...");
    
    motor.attach(MOTOR_PIN);
    
    // Send min output
    Serial.println("Sending minimum output");
    motor.writeMicroseconds(MIN_SIGNAL);

    // Wait for input
    while (!Serial.available());
    Serial.read();

    Serial.println("Now spinning ??");
    motor.writeMicroseconds(SPIN_SIGNAL);
    
    // Wait for input
    while (!Serial.available());
    Serial.read();

    Serial.println("Now spinning faster??");
    motor.writeMicroseconds(MAX_SIGNAL);
}

void loop() {  
    // Wait for input
    /*while (!Serial.available());
    char* speed = Serial.read();

    Serial.println("Now spinning at " + speed);
    motor.writeMicroseconds(atoi(speed));*/
}
