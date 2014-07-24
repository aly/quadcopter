//Arduino 1.0+ only
#include <Servo.h>
#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int cur[3];
int prev[3];

// PID controller values
double error = 0.0;
double integral = 0.0;
double derivitive = 0.0;

// Motor values
static const int MAX_SIGNAL = 2000;
static const int MIN_SIGNAL = 700;
#define MOTOR_PIN_1 9 // Motor 1
#define MOTOR_PIN_2 10 // Motor 2
#define MOTOR_PIN_3 11 // Motor 3
#define MOTOR_PIN_4 12 // Motor 4

// Store motor speeds
int motor_speed[4];
Servo motor[4];

void setup(){

    Wire.begin();
    Serial.begin(9600);

    Serial.println("starting up L3G4200D");
    setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec

    setup_motor();

    delay(1500); //wait for the sensor to be ready
}

void setup_motor() {
    // Initialise the motor speeds
    for (int i = 0; i < 2; i++) {
        motor_speed[i] = MIN_SIGNAL;
    }

    motor[0].attach(MOTOR_PIN_1);
    motor[1].attach(MOTOR_PIN_2);
}

int motors_running = 0;
int motor_0_values[] = { 700, 740, 780, 750, 800 };
int motor_1_values[] = { 700, 740, 780, 800, 750 };

void loop() {
    if (motors_running <= 5) {
        // Wait for input
        Serial.println("waiting for input");
        while (Serial.available() <= 0) { }
        while (Serial.available() > 0 ) { Serial.read(); }
        Serial.println("input accepted");

        update_motor(0, motor_0_values[motors_running]);
        update_motor(1, motor_1_values[motors_running]);
        motors_running++;
        return;
    }

    getGyroValues(cur);  // This will update x, y, and z with new values

    char buf[128];
    sprintf(buf, "Y: cur(%d) prev(%d), diff(%d)\n", cur[1], prev[1], prev[1]-cur[1]);
    Serial.print(buf);

    static double target = 0.0;
    error = target - cur[1];
    integral += cur[1] - target;

    //(controller output) = (error)*100/(proportional band)
    static double kp = 1.0;
    double pval = (error * 100) / kp;

    //CONTROLLER OUTPUT = (1/INTEGRAL) (Integral of) e(t) d(t)

    // CONTROLLER OUTPUT = DERIVATIVE * dm / dt

    //Serial.print(" Y cur:");
    //Serial.print(prev[1] - cur[1]);

    memcpy(prev, cur, 3 * sizeof(int));

    delay(200); //Just here to slow down the serial to make it more readable
}

void getGyroValues(int* v){

    byte xMSB = readRegister(L3G4200D_Address, 0x29);
    byte xLSB = readRegister(L3G4200D_Address, 0x28);
    v[0] = ((xMSB << 8) | xLSB);

    byte yMSB = readRegister(L3G4200D_Address, 0x2B);
    byte yLSB = readRegister(L3G4200D_Address, 0x2A);
    v[1] = ((yMSB << 8) | yLSB);

    byte zMSB = readRegister(L3G4200D_Address, 0x2D);
    byte zLSB = readRegister(L3G4200D_Address, 0x2C);
    v[2] = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
    //From  Jim Lindblom of Sparkfun's code

    // Enable x, y, z and turn off power down:
    writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

    // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
    writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

    // Configure CTRL_REG3 to generate data ready interrupt on INT2
    // No interrupts used on INT1, if you'd like to configure INT1
    // or INT2 otherwise, consult the datasheet:
    writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

    // CTRL_REG4 controls the full-scale range, among other things:

    if(scale == 250){
        writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
    }else if(scale == 500){
        writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
    }else{
        writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
    }

    // CTRL_REG5 controls high-pass filtering of outputs, use it
    // if you'd like:
    writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){
    int v;

    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}

int update_motor(int m, int desired_pwm) {
    char buf[128];
    static const int MAX_ACCEL = 80;

    sprintf( buf, "Updating motor %d to requested %d", m, desired_pwm);
    Serial.println(buf);

    // Safety code reduce danger when testing
    int difference = desired_pwm - motor_speed[m];
    if (difference > MAX_ACCEL) {
        motor_speed[m] += MAX_ACCEL;
        sprintf(buf,"Limiting accel. Setting %d instead of requested %d", motor_speed[m], desired_pwm);
        Serial.println(buf);
    } else {
        motor_speed[m] = desired_pwm;
    }

    sprintf(buf, "New motor speed: %d", motor_speed[m]);
    Serial.println(buf);

    if ( motor_speed[m] > MAX_SIGNAL ) motor_speed[m] = MAX_SIGNAL;
    if ( motor_speed[m] < MIN_SIGNAL ) motor_speed[m] = MIN_SIGNAL;

    motor[m].writeMicroseconds(motor_speed[m]);
    return motor_speed[m];
}

