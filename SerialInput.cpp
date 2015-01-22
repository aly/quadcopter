
#include "SerialInput.h"
#include <Arduino.h>

String input = "";

void SerialInputSetup( State &s )
{
	Serial.println("SerialInputSetup");
}

void SerialInputUpdate( unsigned long frame_delta, State &s )
{
	//Serial.println("SerialInputUpdate");

	if ( Serial.available() > 0 )
	{
		int val = Serial.parseInt();
		/*while ( int inByte = Serial.read() )
		{
			input += inByte;
		}*/
		Serial.println("Recieved value: " + (String)val);
		s.desired_motor_value[0] = val;
		s.desired_motor_value[1] = val;
    	//Serial.write(val);
    	//sSerial.print("\n");
	}
}