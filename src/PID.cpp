#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double setKp, double setKi, double setKd) {
	Kp = setKp;
	Ki = setKi;
	Kd = setKd;

	i_error = 0.0;
	p_error = 0.0;

	total_error = 0.0;
 	cycle_n = 0;
}

void PID::UpdateError(double cte) {
	d_error = (cte-p_error);
	p_error = cte;
	i_error += cte;

	total_error += (cte*cte);
	cycle_n++;

}

double PID::TotalError() {
	return total_error;
}

