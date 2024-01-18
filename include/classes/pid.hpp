#pragma once
#include "main.h"

template <typename U>
class PIDController {
	static_assert(units::traits::is_unit_t<U>::value, "Template parameter \"U\" must be a unit");

  private:
	double kP, kI, kD;
	double time_change;
	double prev_time = -1;
	U error, error_prev, error_total, error_change, derivative;

  public:
	PIDController(double kP, double kI, double kD) : kP(kP), kI(kI), kD(kD) {}

	/**
	 * Run PID calculation
	 */
	inline double calculate(U set_point, U current_pos) {
		error = set_point - current_pos;
		error_change = error - error_prev;
		error_total += error;

		double time = pros::millis() / 1000.0;

		if (prev_time == -1) prev_time = time;

		time_change = time - prev_time;
		derivative = error_change / time_change;

		// Reset integral on pass through 0
		if ((error_prev < U(0) && error > U(0)) || (error_prev > U(0) && error < U(0)))
			error_total = U(0);

		// Clamp integral to + or - 64
		if (error_total > U(64)) error_total = U(64);
		if (error_total < U(-64)) error_total = U(-64);

		error_prev = error;
		prev_time = time;

		return (error * kP + error_total * kI + derivative * kD).template to<double>();
	}

	/**
	 * Check if at or very close to desired point
	 */
	inline bool settled() {
		return units::math::abs(error_change) < U(0.09) && units::math::abs(error) < U(0.1) * kP;
	}

	/**
	 * Configure constants
	 */
	inline void set_gains(double kP, double kI, double kD) {
		this->kP = kP;
		this->kI = kI;
		this->kD = kD;
	}

	/**
	 * Get the error value
	 */
	inline U get_error() { return error; }

	/**
	 * Reset state
	 */
	inline void reset() {
		error_change = U(0);
		error_prev = U(0);
		error_total = U(0);
	}
};