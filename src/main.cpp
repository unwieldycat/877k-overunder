#include "main.h"
#include "autons.hpp"
#include "chassis/auton.hpp"
#include "chassis/user.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "subsystems/catapult.hpp"
#include "subsystems/misc.hpp"
#include <sys/select.h>

rd::Selector selector({
    {"Field Triball Push", push_field},
    {"Preload Triball Push", push_preload},
    {"Skills", skills},
});

rd::Image logo("Logo", "/usd/logo.bin");

// ========================== Initialize Functions ========================== //

void initialize() {
	odom::initialize();
	cata_rot.reset_position();

	pros::Task odom_task(odom::track_position, "Odometry");
}

void competition_initialize() { selector.focus(); }

// ============================ Match Functions ============================ //

void autonomous() {
	logo.focus();
	odom_pistons.extend();
	selector.run_auton();
}

void opcontrol() {
	pros::Task drive_task(chassis::user, "User Chassis Control");
	pros::Task cata_task(cata::user, "Catapult Control");
	pros::Task misc_task(user, "Misc User Control");
}

void disabled() {}
