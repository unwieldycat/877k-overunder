#include "main.h"

// =========================== Device Definitions =========================== //

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
pros::MotorGroup drive_left({6, -7, 8});
pros::MotorGroup drive_right({-16, 17, -18});
pros::MotorGroup punch_motors({-4, 5});

// Odometry sensors
pros::Rotation odom_left(9);
pros::Rotation odom_rear(10);
pros::Imu imu(20);

// Puncher
pros::Rotation puncher_rot(3);

// Pneumatics
pros::adi::Pneumatics roller_piston('A', true);
pros::adi::Pneumatics left_wing('B', false);
pros::adi::Pneumatics right_wing('C', false);
pros::adi::Pneumatics transmission('D', true);
pros::adi::Pneumatics odom_pistons('E', false);
pros::adi::Pneumatics left_lift('F', false);
pros::adi::Pneumatics right_lift('G', false);
