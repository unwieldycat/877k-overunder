#pragma once
#include "devices.hpp"
#include "main.h"
#include "units.h"

template <typename U>
class PIDController {
	static_assert(units::traits::is_unit_t<U>::value, "Template parameter \"U\" must be a unit");

  private:
	units::dimensionless::scalar_t kP, kI, kD;
	U error, error_prev, error_total, error_change;

  public:
	PIDController(double kP, double kI, double kD) : kP(kP), kI(kI), kD(kD) {}

	/**
	 * Run PID calculation
	 */
	inline double calculate(U set_point, U current_pos) {
		error = set_point - current_pos;
		error_change = error - error_prev;
		error_total += error;
		error_prev = error;

		return (error * kP + error_total * kI + error_change * kD).template to<double>();
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
	inline double get_error() { return error.template to<double>(); }

	/**
	 * Reset state
	 */
	inline void reset() {
		error_change = 0;
		error_prev = 0;
		error_total = 0;
	}
};