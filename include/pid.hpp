#pragma once
#include "devices.hpp"
#include "main.h"

class PIDController {
  private:
	float kP, kI, kD;
	float error, error_prev, error_total, error_change;

  public:
	PIDController(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}

	/**
	 * Run PID calculation
	 */
	float calculate(float target, float current_pos);

	/**
	 * Configure constants
	 */
	void set_gains(float kP, float kI, float kD);

	/**
	 * Get the error value
	 */
	float get_error();

	/**
	 * Reset state
	 */
	void reset();
};