#pragma once
#include "devices.hpp"
#include "main.h"

class PIDController {
  private:
	float kP;
	float kI;
	float kD;

	float error_prev;
	float error_total;
	float error_change;

  public:
	PIDController(float kP, float kI, float kD) {}

	/**
	 * Run PID calculation
	 */
	float calculate(float set_point, float current_pos);

	/**
	 * Checks if the exit condition has been met
	 */
	bool condition_met(float range);

	/**
	 * Reset state
	 */
	void reset();
};