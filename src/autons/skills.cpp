#include "chassis/pursuit.hpp"
#include "main.h"

const int launches = 50;

void skills() {
	// Set odom to starting point
	odom::calibrate(2_ft, 2_ft, 225_deg);

	while (true) {
		puncher::prime();
		pros::delay(1000);
		puncher::release();
	}

	chassis::pursuit("/usd/paths/skills0.csv", true);
	chassis::pursuit("/usd/paths/skills1.csv");
}
