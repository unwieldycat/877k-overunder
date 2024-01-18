#include "slew.hpp"
#include "units.h"

SlewController::SlewController(int max) : volt_max(max), volt_prev(0) {}

int SlewController::calculate(int voltage) {
	if (voltage > 127) voltage = 127;
	if (voltage < -127) voltage = -127;
	if (voltage > volt_prev + volt_max) voltage = volt_max;
	volt_prev = voltage;
	return voltage;
}