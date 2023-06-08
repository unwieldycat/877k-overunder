#include "main.h"
#include "chassis.hpp"
#include "odom.hpp"

// ========================== Initialize Functions ========================== //

void initialize() {
	odom::initialize();
	pros::Task odom_task(odom::track_position, "Odometry");
}

void competition_initialize() {}

// ============================ Match Functions ============================ //

void autonomous() {}

void opcontrol() { pros::Task user_task(chassis::user_control, "User Chassis Control"); }

void disabled() {}
