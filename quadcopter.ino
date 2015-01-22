#include "State.h"

#include "Motor.h"
#include "SerialInput.h"

#include <Servo.h> // because retarded. This is also included in Motor.cpp, but both are required.
#include <Wire.h>

// ms = 1/Hz * 1000
float framerate = 30.0f;//0.5f; // value in Hz
float update_time = (1.0f/framerate) * 1000.0f;
unsigned long ms_per_frame = (int)(update_time + 0.5f);
unsigned long last_update = 0;

State copter_state;

void setup()
{
  //copter_state.setState(State.STATE_SETUP);

  Serial.begin(9600);
  Wire.begin();

  Serial.println("Starting up GyroPid controller");

  SerialInputSetup(copter_state);

  MotorSetup(copter_state);

  //GyroSetup();
  //GyroCalibrate();

  delay(1500); //wait for the sensor to be ready
  last_update = millis();
}

void loop()
{
  unsigned long frame_start = millis();
  // limit framerate
  unsigned long frame_delta = frame_start - last_update;
  float f_delta = (float)frame_delta / 1000.0f;
  if ( frame_delta < ms_per_frame ) return;

  SerialInputUpdate(frame_delta, copter_state);

  MotorUpdate(f_delta, copter_state);

  /*getGyroValues(cur);  // This will update x, y, and z with new values

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
  */

  // store time from start of frame, so we keep our update rate constant, even if this frame took longer
  last_update = frame_start;
}
