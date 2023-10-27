#include "autons.hpp"
#include "chassis/auton.hpp"

void park_right() {
	chassis::drive(2_ft);
	chassis::turn_rel(-90_deg);
	chassis::drive(2_ft);
	chassis::turn_abs(90_deg);
}

void park_left() {
	chassis::drive(2_ft);
	chassis::turn_rel(90_deg);
	chassis::drive(2_ft);
	chassis::turn_abs(-90_deg);
}