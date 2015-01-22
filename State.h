#ifndef STATE_H
#define STATE_H

class State
{
public:
	State() : desired_motor_zero_value(0) {}
	~State() {}

	// StateMachine
	//setState( COPTER_STATE s ) { state_machine = s; }
	//getState( )
public:
	int desired_motor_zero_value;

	// State machine for copter
	enum COPTER_STATE {
		STATE_NONE = 0,
		STATE_SETUP,
		STATE_CALIBRATE,
		STATE_HOVER
	};

protected:
	COPTER_STATE state_machine;
};

#endif