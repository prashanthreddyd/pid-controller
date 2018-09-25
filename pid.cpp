#include <PID.h>

PID::PID():_dt(0.01), _imax(1000), _imin(-1000), _max(1024), _min(1), _Kp(0), _Kd(0), _Ki(0), _pre_error(0), _integral(0){
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

void PID::init(double Kp_new, double Ki_new, double Kd_new, double imax_new, double imin_new, double max_new, double min_new){
	_Kp = Kp_new;
	_Ki = Ki_new;
	_Kd = Kd_new;
	_imax = imax_new;
	_imin = imin_new;
	_max = max_new;
	_min = min_new;
	_pre_error = 0;
	_integral = 0;
}

void PID::update(double Kp_new, double Ki_new, double Kd_new, double max_new, double min_new){
	_Kp = Kp_new;
	_Ki = Ki_new;
	_Kd = Kd_new;
	_max = max_new;
	_min = min_new;
	_pre_error = 0;
	_integral = 0;
}

double PID::calculate(double sp, double pv) {
	double error = 0;
	double Pout=0;
	double Iout = 0;
	double derivative=0;
	double Dout = 0;
	double output = 0;

	// Calculate error
	error = sp - pv;

	// Proportional term
	Pout = _Kp * error;

	// Integral term
	_integral += error * _dt;
	// Restrict Integral term to max/min
	if( _integral > _imax )
		_integral = _imax;
	else if( _integral < _imin )
		_integral = _imin;

	Iout = _Ki * _integral;

	// Derivative term
	derivative = (error - _pre_error) / _dt;
	Dout = _Kd * derivative;

	// Calculate total output
	output = Pout + Iout + Dout;

	// Restrict output to max/min
	if( output > _max )
		output = _max;
	else if( output < _min )
		output = _min;

	// Save error to previous error
	_pre_error = error;
  
  return output;
}
