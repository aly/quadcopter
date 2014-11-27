#ifndef STATE_H
#define STATE_H

class State
{
public:
	State() : desired_motor_value(0) {}
	~State() {}

	int desired_motor_value;
};

#endif