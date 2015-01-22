#ifndef STATE_H
#define STATE_H

class State
{
public:
	State() {
		desired_motor_value[0] = 0;
		desired_motor_value[1] = 0;
	}
	~State() {}

	// StateMachine
	//setState( COPTER_STATE s ) { state_machine = s; }
	//getState( )
public:
	int desired_motor_value[2];

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