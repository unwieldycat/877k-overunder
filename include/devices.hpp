#pragma once
#include "main.h"

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