#include "pid.hpp"

template <typename U>
double PIDController<U>::calculate(U set_point, U current_pos) {
	error = current_pos - set_point;
	error_change = error - error_prev;
	error_total += error;
	error_prev = error;

	return (error * kP + error_total * kI + error_change * kD);
}

template <typename U>
void PIDController<U>::set_gains(double kP, double kI, double kD) {
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;
}

template <typename U>
double PIDController<U>::get_error() { return error; }

template <typename U>
void PIDController<U>::reset() {
	error_change = 0;
	error_prev = 0;
	error_total = 0;
}
