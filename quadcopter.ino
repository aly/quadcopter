//Arduino 1.0+ only
#include <Servo.h>
#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D gyro

int cur[3];
int prev[3];

// ms = 1/Hz * 1000
float framerate = 30.0f; // 30Hz
float update_time = (1/framerate) * 1000;
unsigned long ms_per_frame = (int)(update_time + 0.5f);
unsigned long last_update = 0;

// PID controller values
double error = 0.0;
double integral = 0.0;
double derivitive = 0.0;

// Motor values
// ESC wiring, white to pin 9, black to ground
static const int MAX_SIGNAL = 1000;
static const int MIN_SIGNAL = 750;
static const int INIT_SIGNAL = 700;
static const int MOTOR_ACCEL = 80; // accel change per second
#define MOTOR_PIN_1 9 // Motor 1
#define MOTOR_PIN_2 10 // Motor 2
#define MOTOR_PIN_3 11 // Motor 3
#define MOTOR_PIN_4 12 // Motor 4

// Store motor speeds
double motor_speed_actual[4];
int motor_speed_set[4];
Servo motor[4];

void setup(){

    Wire.begin();
    Serial.begin(9600);

    Serial.println("Starting up L3G4200D");
    setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec

    setup_motor();

    delay(1500); //wait for the sensor to be ready
}

void setup_motor() {
    // zero all motor speeds
    memset(motor_speed_actual, 0, 4 * sizeof(int));
    memset(motor_speed_set, 0, 4 * sizeof(int));

    // Initialise the motor speeds
    for (int i = 0; i < 2; i++) {
        motor_speed_actual[i] = INIT_SIGNAL;
        motor_speed_set[i] = INIT_SIGNAL;
    }

    motor[0].attach(MOTOR_PIN_1);
    motor[1].attach(MOTOR_PIN_2);

    last_update = millis();
}

int motors_running = 0;
int motor_0_values[] = { 700, 740, 780, 750, 800 };
int motor_1_values[] = { 700, 740, 780, 800, 750 };

void loop() {
    unsigned long frame_start = millis();
    unsigned long deltal = frame_start - last_update;
    float delta = (float)deltal;
    double deltad = 56.0;

    char buf[128];
    sprintf(buf, "delta: %d / %d / %d", (int)(delta * 100), (int)(deltad*100), deltal);
    Serial.println(buf);

    // limit framerate
    //if ( delta < ms_per_frame ) return;

    if (motors_running <= 5) {
        // Wait for input
        Serial.println("waiting for input");
        while (Serial.available() <= 0) { }
        while (Serial.available() > 0 ) { Serial.read(); }
        Serial.println("input accepted");

        set_motor_speed(0, motor_0_values[motors_running]);
        set_motor_speed(1, motor_1_values[motors_running]);
        motors_running++;

        update_motor(delta);
        last_update = frame_start;
        return;
    }
    update_motor(delta);

    getGyroValues(cur);  // Results are written to cur

    {
        char buf[128];
        sprintf(buf, "Y: cur(%d) prev(%d), diff(%d)\n", cur[1], prev[1], prev[1]-cur[1]);
        Serial.print(buf);
    }

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

    last_update = frame_start;
}

void getGyroValues(int* v) {
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

int set_motor_speed( int m, int desired_pwm ) {
    // set speed directly into the 'set' motor array, the update method does the actual motor setting
    
    // Safety code reduce danger when testing
    //int difference = desired_pwm - motor_speed_set[m];
    if (desired_pwm >= MAX_SIGNAL) {
        //motor_speed_set[m] = MAX_SPEED;
        char buf[128];
        sprintf(buf,"Limiting motor[%d] speed. Requested %d. Current %d", m,  desired_pwm, motor_speed_set[m]);
        Serial.println(buf);
    } else if (desired_pwm < INIT_SIGNAL) {
        motor_speed_set[m] = 0;
        char buf[128];
        sprintf(buf,"Limiting motor[%d] speed. Requested %d. Set to %d", m,  desired_pwm, motor_speed_set[m]);
        Serial.println(buf);
    } else {
        char buf[128];
        sprintf(buf,"Set motor[%d] to %d", m, desired_pwm);
        Serial.println(buf);
        motor_speed_set[m] = desired_pwm;
    }

    // return currently set speed
    return motor_speed_set[m];
}

int update_motor( float delta ) {

    {
        char buf[128];
        sprintf( buf, "Motor update: delta '%d'", delta);
        Serial.println(buf);
    }

    /*{
        char buf[128];
        sprintf(buf, "New motor speed: %d", motor_speed[m]);
        Serial.println(buf);
    }*/
    
    // for now: ignore rounding errors due to fast framerate. We seem to get about 200ms/frame
    float max_diff = delta * 1.0f; //((double)delta / 1000.0) * (double)MOTOR_ACCEL; 
    float max_diff2 = delta * 1.0f; 

#define MIN(a,b) a < b ? a : b
#define MAX(a,b) a > b ? a : b

    for ( int i = 0; i < 2; i++ ) {
        if ( motor_speed_set[i] == motor_speed_actual[i] ) continue;
        
        double difference = motor_speed_set[i] - motor_speed_actual[i];
        
    {
        char buf[128];
        sprintf( buf, "max diff: '%f','%f', actualdiff: '%f', set: '%d', actual: '%f'", max_diff, max_diff2, difference, motor_speed_set[i], motor_speed_actual[i]);
        Serial.println(buf);
    }
        if ( difference > 0 )
        {
            motor_speed_actual[i] = MIN( (motor_speed_actual[i] + max_diff), MAX_SIGNAL );
        }
        else if ( difference < 0 )
        {
            motor_speed_actual[i] = MAX( (motor_speed_actual[i] - max_diff), MIN_SIGNAL );
        }
        // update actual motor with the new value
        {
            char buf[64];
            sprintf(buf, "Hardware motor[%d] set to: %d", i, motor_speed_actual[i]);
            Serial.println(buf);
        }
        motor[i].writeMicroseconds((int)motor_speed_actual[i]);
    }

    return 0; 
}

