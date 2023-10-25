#include "autons.hpp"
#include "chassis.hpp"

void park_out(bool left) {
	chassis::turn_rel(-45_deg);
	chassis::drive(5.65_ft);
	chassis::turn_abs(90_deg);
}

void park_in(bool left) {
	chassis::drive(2_ft);
	chassis::turn_rel(-90_deg);
	chassis::drive(2_ft);
	chassis::turn_abs(90_deg);
}