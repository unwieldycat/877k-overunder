#include "devices.hpp"

// =========================== Device Definitions =========================== //

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
pros::MotorGroup drive_left({1, 2});
pros::MotorGroup drive_right({3, 4});

// Odometry sensors
pros::Rotation odom_left(11);
pros::Rotation odom_rear(12);
pros::Imu imu(13);
