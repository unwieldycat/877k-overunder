#include "main.h"

rd::Selector selector({
    {"AWP", awp},
    {"Push Right", push_right},
    {"Push Left", push_left},
    {"Skills", skills},
});

rd::Image logo("Logo", "/usd/logo.bin");

// ========================== Initialize Functions ========================== //

void initialize() {
	odom::initialize();
	puncher_rot.reset_position();
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
	pros::Task cata_task(puncher::user, "Puncher Control");
	pros::Task misc_task(user, "Misc User Control");
	pros::Task piston_task(booster::automatic, "Auto boosters");
}

void disabled() {}
