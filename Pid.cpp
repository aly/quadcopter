
class Pid {
public:
	Pid():
		pid_error(0.0),
		pid_integral(0.0),
		pid_derivitive(0.0)
	{}

	//Update();

public:
	// PID controller values
	double pid_error;
	double pid_integral;
	double pid_derivitive;
};
