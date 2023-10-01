#include "chassis.hpp"
#include "devices.hpp"

// ========================= User Control Functions ========================= //

void tank_drive(input::analog_inputs_t inputs) {
	drive_left.move(inputs.left.y);
	drive_right.move(inputs.right.y);
}

void arcade_drive(input::analog_inputs_t inputs) {
	drive_left.move(inputs.left.y + inputs.right.x);
	drive_right.move(inputs.left.y - inputs.right.x);
}

void curvature_drive(input::analog_inputs_t inputs) {
	double power = inputs.left.y / 127.0;
	double curvature = inputs.right.x / 127.0;

	double left = power + power * curvature;
	double right = power - power * curvature;
	double max = std::max(std::fabs(left), std::fabs(right));

	if (max > 1.0) {
		left /= max;
		right /= max;
	}

	drive_left.move(left * 127.0);
	drive_right.move(right * 127.0);
}
