#include "chassis/auton.hpp"
#include "devices.hpp"
#include "main.h"
#include <iostream>

const int launch_time = 45000;

void skills() {
	odom::calibrate(0_ft, 0_ft, 180_deg);

	// align to launch
	chassis::drive(23_in);
	chassis::turn_rel(-95_deg);
	chassis::drive(20_in);
	left_wing.extend();

	// LAUNCH!

	punch_motors.move(104);

	int start_time = pros::millis();
	while (start_time - pros::millis() < launch_time)
		pros::delay(1000);

	punch_motors.brake();

	// Cross
	pros::delay(500);
	left_wing.retract();
	chassis::drive(-16_in);
	chassis::turn_rel(-145_deg, chassis::TurnSide::Right);
	chassis::drive(15_in);
	chassis::turn_rel(-25_deg, chassis::TurnSide::Right);
	chassis::drive(5.4_ft, 275_deg);

	// Push into goal
	chassis::turn_rel(-37_deg, chassis::TurnSide::Right);
	chassis::drive(18_in);
	chassis::turn_rel(-40_deg);
	chassis::drive(127, 1.5_s);
	chassis::drive(-8_in);
	chassis::turn_rel(-80_deg);
	chassis::drive(40_in);
	chassis::turn_rel(90_deg);
	left_wing.extend();
	right_wing.extend();
	chassis::drive(2_ft);
	chassis::turn_rel(45_deg, chassis::TurnSide::Left);
	chassis::drive(127, 4_s);
	chassis::drive(-127, 1.5_s);
	chassis::turn_rel(-90_deg);
	chassis::drive(1.5_ft);
	chassis::turn_rel(90_deg);
	chassis::drive(127, 2_s);

	// chassis::pursuit("/usd/paths/skills0.csv", true);
	// chassis::pursuit("/usd/paths/skills1.csv");
}
