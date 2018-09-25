#ifndef PID_H_
#define PID_H_

class PID {
public:
	static PID& getInstance()
	{
		static PID _instance;
		return _instance;
	}
	PID();
	virtual ~PID();
	void init(double Kp_new, double Ki_new, double Kd_new, double imax_new, double imin_new, double max_new, double min_new);
	void update(double Kp_new, double Ki_new, double Kd_new, double max_new, double min_new);
	double calculate(double sp, double pv);
  
private:
	double _dt;
	double _imax;
	double _imin;
	double _max;
	double _min;
	double _Kp;
	double _Kd;
	double _Ki;
	double _pre_error;
	double _integral;
};
#endif /* PID_H_ */
