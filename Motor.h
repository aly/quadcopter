#ifndef MOTOR_H
#define MOTOR_H
#include "State.h"

void MotorSetup( State &s );

void MotorUpdate( float frame_delta, State &s );


#endif // MOTOR_H