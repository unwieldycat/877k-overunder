#pragma once
#include "main.h"

// ========================== Device Declarations ========================== //

extern pros::Controller controller;

extern pros::MotorGroup drive_left;
extern pros::MotorGroup drive_right;
extern pros::MotorGroup punch_motors;

extern pros::Rotation odom_left;
extern pros::Rotation odom_rear;
extern pros::Imu imu;

extern pros::Rotation puncher_rot;

extern pros::adi::Pneumatics left_lift;
extern pros::adi::Pneumatics right_lift;
extern pros::adi::Pneumatics roller_piston;
extern pros::adi::Pneumatics left_wing;
extern pros::adi::Pneumatics right_wing;
extern pros::adi::Pneumatics transmission;
extern pros::adi::Pneumatics odom_pistons;

extern pros::adi::Led strip_left;
extern pros::adi::Led strip_right;