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

// Catapult
pros::adi::DigitalIn cata_switch('H');
pros::Optical cata_optical(15);

// Pneumatics
pros::adi::Pneumatics roller_piston('A', true);
pros::adi::Pneumatics left_wing('B', false);
pros::adi::Pneumatics right_wing('C', false);
pros::adi::Pneumatics transmission('D', false);
pros::adi::Pneumatics odom_pistons('E', false);

// Addressable LEDs
// (Requires ADI expander on port 10)
pros::adi::Led strip_left({10, 'A'}, 24);
pros::adi::Led strip_right({10, 'B'}, 24);
