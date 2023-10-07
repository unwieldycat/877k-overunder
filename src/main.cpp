#include "main.h"
#include "chassis.hpp"
#include "devices.hpp"
#include "input.hpp"
#include "odom.hpp"
#include "pursuit.hpp"

// ========================== Initialize Functions ========================== //

void initialize() {
	odom::initialize();

	input::set_buttons({
	    {pros::E_CONTROLLER_DIGITAL_X, wrap_func(roller_piston.toggle)},
	    {pros::E_CONTROLLER_DIGITAL_R1, wrap_func(right_wing.toggle)},
	    {pros::E_CONTROLLER_DIGITAL_L1, wrap_func(left_wing.toggle)},
	    {pros::E_CONTROLLER_DIGITAL_UP, wrap_func(transmission.retract)},
	    {pros::E_CONTROLLER_DIGITAL_DOWN, wrap_func(transmission.extend)},
	});

	pros::Task odom_task(odom::track_position, "Odometry");
}

void competition_initialize() {}

// ============================ Match Functions ============================ //

void autonomous() {}

void opcontrol() {
	pros::Task drive_task(input::driver, "User Chassis Control");
	pros::Task buttons_task(input::watcher, "Button watcher");
}

void disabled() {}
