#pragma once
#include "main.h"

class SlewController {
  private:
	int volt_max;
	int volt_prev;

  public:
	/**
	 * @brief Create a new Slew Controller
	 *
	 * @param max Maximum change in motor voltage
	 */
	SlewController(int max);

	/**
	 * @brief Process value
	 *
	 * @param voltage Desired voltage for motor
	 * @return Processed value
	 */
	int calculate(int voltage);
};