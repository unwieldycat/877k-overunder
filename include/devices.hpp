#pragma once
#include "main.h"
#include "pros/adi.hpp"

// ========================== Device Declarations ========================== //

extern pros::Controller controller;

extern pros::MotorGroup drive_left;
extern pros::MotorGroup drive_right;
extern pros::MotorGroup catapult;

extern pros::Rotation odom_left;
extern pros::Rotation odom_rear;
extern pros::Imu imu;

extern pros::adi::Pneumatics roller_piston;
extern pros::adi::Pneumatics left_wing;
extern pros::adi::Pneumatics right_wing;
extern pros::adi::Pneumatics transmission;

extern pros::adi::Led strip_left;
extern pros::adi::Led strip_right;