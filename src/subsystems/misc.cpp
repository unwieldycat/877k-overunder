#include "devices.hpp"

void user() {
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) roller_piston.toggle();
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) right_wing.toggle();
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) left_wing.toggle();
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) transmission.retract();
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) transmission.extend();
	pros::delay(20);
}