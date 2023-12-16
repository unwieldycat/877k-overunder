#include "main.h"
#include "autons.hpp"
#include "chassis/auton.hpp"
#include "chassis/user.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "subsystems/catapult.hpp"
#include "subsystems/misc.hpp"
#include <sys/select.h>

rd::Selector selector;
rd::Image logo("Logo", "/usd/logo.bin");

// ========================== Initialize Functions ========================== //

bool initialized = false;
void initialize() {
	if (initialized) return;
	initialized = true;

	odom::initialize();
	cata_rot.reset_position();

	// Tasks
	pros::Task odom_task(odom::track_position, "Odometry");
}

void competition_initialize() {
	initialize();

	selector.add_autons(
	    {{"Field Triball Push", push_field},
	     {"Preload Triball Push", push_preload},
	     {"Skills", skills}}
	);

	selector.focus();
}

// ============================ Match Functions ============================ //

void autonomous() {
	initialize();

	logo.focus();
	odom_pistons.extend();
	selector.do_auton();
}

void opcontrol() {
	initialize();

	pros::Task drive_task(chassis::user_drive, "User Chassis Control");
	pros::Task cata_task(cata::user, "Catapult Control");
	pros::Task misc_task(user, "Misc User Control");
}

void disabled() {}
