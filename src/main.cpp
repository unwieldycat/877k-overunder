#include "main.h"
#include "autons.hpp"
#include "chassis/auton.hpp"
#include "chassis/user.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "subsystems/catapult.hpp"
#include "subsystems/misc.hpp"

rd::SelectorView selector;
rd::ImageView logo("Logo", "/usd/logo.bin");

// ========================== Initialize Functions ========================== //

void initialize() {
	rd::initialize();
	odom::initialize();
	strip_left.set_all(0xff6600);
	strip_right.set_all(0xff6600);
	cata_rot.reset_position();

	// Configure GUI
	rd::register_views({&selector, &logo});

	selector.add_autons(
	    {{"Park (left)", park_left},
	     {"Park (right)", park_right},
	     {"Field Triball Push", push_field},
	     {"Preload Triball Push", push_preload},
	     {"Skills", skills}}
	);

	// Tasks
	pros::Task odom_task(odom::track_position, "Odometry");
}

void competition_initialize() { rd::set_view(&selector); }

// ============================ Match Functions ============================ //

void autonomous() {
	rd::set_view(&logo);

	odom_pistons.extend();
	pros::delay(1000);
	odom_pistons.retract();

	selector.do_auton();
}

void opcontrol() {
	rd::set_view(&logo);
	pros::Task drive_task(chassis::user_drive, "User Chassis Control");
	pros::Task cata_task(cata::user, "Catapult Control");
	pros::Task misc_task(user, "Misc User Control");
}

void disabled() {}
