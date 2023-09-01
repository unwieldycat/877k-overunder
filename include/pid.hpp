#pragma once
#include "devices.hpp"
#include "main.h"

template <typename U> 
class PIDController {
	static_assert(units::traits::is_unit_t<U>::value, "Template parameter \"U\" must be a unit");

  private:
	double kP, kI, kD;
	U error, error_prev, error_total, error_change;

  public:
	PIDController(double kP, double kI, double kD) : kP(kP), kI(kI), kD(kD) {}

	/**
	 * Run PID calculation
	 */
	double calculate(U target, U current_pos);

	/**
	 * Configure constants
	 */
	void set_gains(double kP, double kI, double kD);

	/**
	 * Get the error value
	 */
	double get_error();

	/**
	 * Reset state
	 */
	void reset();
};