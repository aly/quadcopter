#ifndef SERIAL_INPUT_H
#define SERIAL_INPUT_H

#include "State.h"

void SerialInputSetup( State &s );

void SerialInputUpdate( unsigned long frame_delta, State &s );

#endif