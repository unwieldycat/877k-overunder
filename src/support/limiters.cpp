#include "main.h"

int slew(int new_volts, int current_volts, int max) {
	if (new_volts > 127) new_volts = 127;
	if (new_volts < -127) new_volts = -127;
	if (max > new_volts - current_volts) new_volts = max;
	return new_volts;
}

int clamp(int value, int max) { return clamp(value, max, -max); }

int clamp(int value, int max, int min) {
	if (value > max) value = max;
	if (value < min) value = min;
	return value;
}