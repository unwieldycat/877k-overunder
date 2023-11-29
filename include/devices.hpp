#pragma once
#include "main.h"

// ========================== Device Declarations ========================== //

extern pros::Controller controller;

extern pros::MotorGroup drive_left;
extern pros::MotorGroup drive_right;
extern pros::Motor catapult;

extern pros::Rotation odom_left;
extern pros::Rotation odom_rear;
extern pros::Imu imu;

extern pros::Rotation cata_rot;
extern pros::Optical cata_optical;

extern pros::adi::Pneumatics roller_piston;
extern pros::adi::Pneumatics left_wing;
extern pros::adi::Pneumatics right_wing;
extern pros::adi::Pneumatics transmission;
extern pros::adi::Pneumatics odom_pistons;

extern pros::adi::Led strip_left;
extern pros::adi::Led strip_right;