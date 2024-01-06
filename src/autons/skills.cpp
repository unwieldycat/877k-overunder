#include "chassis/pursuit.hpp"
#include "main.h"

const int launches = 50;

void skills() {
	// Set odom to starting point
	odom::calibrate(2_ft, 2_ft, 225_deg);

	punch_motors.move(104);

	// chassis::pursuit("/usd/paths/skills0.csv", true);
	// chassis::pursuit("/usd/paths/skills1.csv");
}
