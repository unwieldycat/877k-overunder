#include "main.h"
#include "autons.hpp"
#include "devices.hpp"
#include "input.hpp"
#include "macros.hpp"
#include "odom.hpp"

rd::SelectorView selector;
rd::ImageView logo("Logo", "/usd/logo.bin");

// ========================== Initialize Functions ========================== //

void initialize() {
	odom::initialize();

	input::set_buttons({
	    {pros::E_CONTROLLER_DIGITAL_X, wrap_func(roller_piston.toggle)},
	    {pros::E_CONTROLLER_DIGITAL_R1, wrap_func(right_wing.toggle)},
	    {pros::E_CONTROLLER_DIGITAL_L1, wrap_func(left_wing.toggle)},
	    {pros::E_CONTROLLER_DIGITAL_UP, wrap_func(transmission.retract)},
	    {pros::E_CONTROLLER_DIGITAL_DOWN, wrap_func(transmission.extend)},
	    {pros::E_CONTROLLER_DIGITAL_UP, macros::reverse},
	    {pros::E_CONTROLLER_DIGITAL_A, macros::catapult},
	});

	rd::initialize();
	rd::register_views({&selector, &logo});

	selector.add_autons(
	    {{"Park (left)", park_left},
	     {"Park (right)", park_right},
	     {"Field Triball Push", push_field},
	     {"Preload Triball Push", push_preload},
	     {"Skills", skills}}
	);

	pros::Task odom_task(odom::track_position, "Odometry");
}

void competition_initialize() { rd::set_view(&selector); }

// ============================ Match Functions ============================ //

void autonomous() { rd::set_view(&logo); }

void opcontrol() {
	rd::set_view(&logo);
	pros::Task drive_task(input::driver, "User Chassis Control");
	pros::Task buttons_task(input::watcher, "Button watcher");
}

void disabled() {}
