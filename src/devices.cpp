#include "devices.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"

// =========================== Device Definitions =========================== //

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
pros::MotorGroup drive_left({19, -20});
pros::MotorGroup drive_right({-2, 3});
pros::MotorGroup catapult({6, -7});

// Odometry sensors
pros::Rotation odom_left(14);
pros::Rotation odom_rear(12);
pros::Imu imu(13);

// Pneumatics
pros::adi::Pneumatics roller_piston('A', true);
pros::adi::Pneumatics left_wing('B', false);
pros::adi::Pneumatics right_wing('C', false);
pros::adi::Pneumatics transmission('D', false);