#pragma once
#include "main.h"

/**
 * @brief PID Controller class
 *
 * @tparam U Unit
 */
template <typename U>
class PIDController {
	static_assert(units::traits::is_unit_t<U>::value, "Template parameter \"U\" must be a unit");

  private:
	double kP, kI, kD, settle_accuracy;
	double time_change;
	double prev_time = -1;
	int settle_start, settle_time;
	U error, error_prev, error_total, error_change, derivative;

  public:
	PIDController(double kP, double kI, double kD, int settle_time, double settle_accuracy)
	    : kP(kP), kI(kI), kD(kD), settle_time(settle_time), settle_accuracy(settle_accuracy) {}

	/**
	 * @brief Run PID calculation
	 *
	 * @param set_point Desired point
	 * @param current_pos Current position
	 * @return Motor output value
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
		error_total = clamp(error_total, 64);

		error_prev = error;
		prev_time = time;

		return (error * kP + error_total * kI + derivative * kD).template to<double>();
	}

	/**
	 * @brief Check if at or very close to desired point
	 * @return Settled state
	 */
	inline bool settled() {
		if (abs(error_prev) > settle_accuracy && abs(error) < settle_accuracy)
			settle_start = pros::millis();
		if (settle_start - pros::millis() > settle_time) return true;
		return false;
	}

	/**
	 * @brief Configure constants
	 *
	 * @param kP P constant
	 * @param kI I constant
	 * @param kD D constant
	 */
	inline void set_gains(double kP, double kI, double kD) {
		this->kP = kP;
		this->kI = kI;
		this->kD = kD;
	}

	/**
	 * @brief Get the current error
	 * @return Error
	 */
	inline U get_error() { return error; }

	/**
	 * @brief Reset PID state
	 */
	inline void reset() {
		error_change = U(0);
		error_prev = U(0);
		error_total = U(0);
	}
};