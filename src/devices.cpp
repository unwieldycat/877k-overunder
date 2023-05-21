#include "devices.hpp"

// =========================== Device Definitions =========================== //

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup drive_left({1, 2});
pros::MotorGroup drive_right({3, 4});
