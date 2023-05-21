#pragma once
#include "devices.hpp"
#include "main.h"

class PIDController {
  private:
	float kP, kI, kD;
	float error_prev, error_total, error_change;
	float set_point, range;

  public:
	PIDController(float kP, float kI, float kD) {}

	/**
	 * Run PID calculation
	 */
	float calculate(float current_pos);

	/**
	 * Set exit condition
	 */
	void set_goal(float set_point, float range);

	/**
	 * Checks if the exit condition has been met
	 */
	bool goal_met();

	/**
	 * Reset state
	 */
	void reset();
};