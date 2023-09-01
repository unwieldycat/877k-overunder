#pragma once
#include "devices.hpp"
#include "main.h"

class PIDController {
  private:
	double kP, kI, kD;
	double error, error_prev, error_total, error_change;

  public:
	PIDController(double kP, double kI, double kD) : kP(kP), kI(kI), kD(kD) {}

	/**
	 * Run PID calculation
	 */
	double calculate(double target, double current_pos);

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