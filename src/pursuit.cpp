#include "pursuit.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pros/motors.h"
#include "units.h"
using namespace units::math;

std::vector<std::pair<foot_t, foot_t>> points = {std::make_pair(0_ft, 0_ft)};

void pursuit::add_point(foot_t x_ft, foot_t y_ft) {
	if (points[points.size() - 1].first != x_ft || points[points.size() - 1].second != y_ft) {
		points.push_back({x_ft, y_ft});
	}
}

void pursuit::pursuit(
    foot_t lookahead_Distance, int voltage_constant, foot_t lowest_x = 0_ft, foot_t lowest_y = 0_ft,
    foot_t highest_x = 0_ft, foot_t highest_y = 0_ft
) {
	int current_point = 1;
	units::dimensionless::scalar_t slope_par, slope_perp;
	foot_t closest_point, next_objective_x, next_objective_y, const_par, const_perp;
	degree_t heading_objective;
	bool restricted = false;

	drive_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// Checking if a restriction zone is set
	if (highest_x - lowest_x != 0_ft && highest_y - lowest_y != 0_ft) {
		restricted = true;
	}

	// Loops until all points have been passed NOTE: remove type conversion after odom has units
	while (current_point < points.size()) {
		degree_t current_heading = (degree_t)imu.get_heading();
		foot_t current_posX = (foot_t)odom::get_x(), current_posY = (foot_t)odom::get_y();

		// BOOKMARK: Begin finding next objective
		//  Finds closest point when X coordinates of previous and current points are different
		if (points[current_point - 1].first != points[current_point].first) {
			slope_par = (points[current_point].second - points[current_point - 1].second) /
			            (points[current_point].first - points[current_point - 1].first);
			const_par = (points[current_point].second - slope_par * points[current_point].first);
			closest_point = current_posX;

			int sign =
			    (points[current_point].first - points[current_point - 1].first) > 0_ft ? 1 : -1;
			next_objective_x =
			    (-(2 * (slope_par * (const_par - current_posY) - current_posX)) +
			     sign * sqrt(
			                pow<2>(2 * (slope_par * (const_par - current_posY) - current_posX)) -
			                4 * (pow<2>(slope_par) + 1) *
			                    (pow<2>(current_posX) + pow<2>(const_par - current_posY) -
			                     pow<2>(lookahead_Distance))
			            )) /
			    (2 * (pow<2>(slope_par) + 1));

			next_objective_y = slope_par * next_objective_x + const_par;

		} else {
			// X coordinates of previous and current point are the same
			int sign =
			    (points[current_point].second - points[current_point - 1].second) > 0_ft ? 1 : -1;
			next_objective_x = points[current_point].first;
			next_objective_y =
			    sign * sqrt(pow<2>(lookahead_Distance) - pow<2>(next_objective_x - current_posX)) +
			    current_posY;
		}

		// BOOKMARK: Begin distance calculations for closest point
		//  Finds distance between goal and robot when Y coordinates of previous and current
		//  points are different
		if (fabs(points[current_point].second - points[current_point - 1].second) > 0.1_ft) {
			slope_perp = -(points[current_point].first - points[current_point - 1].first) /
			             (points[current_point].second - points[current_point - 1].second);
			const_perp = (current_posY - slope_perp * current_posX);
			closest_point = (const_perp - const_par) / (slope_par - slope_perp);

			// Robot is more than the lookahead distance away from the path
			if (sqrt(
			        pow<2>(closest_point - current_posX) +
			        pow<2>(slope_par * closest_point + const_par - current_posY)
			    ) > lookahead_Distance) {
				// NOTE: add action to bring robot back to path
				drive_left.brake();
				drive_right.brake();
				std::cout << "Robot too far from path!" << std::endl;
				break;
			}
		}

		// BOOKMARK: Check if goal is in a restricted zone
		if (restricted && ((lowest_x < next_objective_x && next_objective_x < highest_x) ||
		                   (lowest_y < next_objective_y && next_objective_y < highest_y))) {
			// NOTE: change coordinates to corner of the goal
			drive_left.brake();
			drive_right.brake();
			controller.set_text(1, 1, "Can't go there! Rerouted-");
			points.insert(points.begin(), current_point, {1_ft, 2_ft});
			continue;
		}

		// BOOKMARK: Goal increment
		if (fabs(next_objective_x - points[current_point].first) < 0.1_ft) {
			current_point++;
		}

		// BOOKMARK: Heading calculations
		//  Find math heading objective
		heading_objective =
		    atan2(next_objective_x - current_posY, next_objective_y - current_posX) * 180 / M_PI;
		if (heading_objective < 0_deg) {
			heading_objective += 360_deg;
		}

		/* convert to inertial heading
		if (0 <= heading_objective && heading_objective <= 90) {
		    heading_objective = 90 - heading_objective;
		} else if (90 < heading_objective && heading_objective <= 180) {
		    heading_objective += 90;
		} else if (180 < heading_objective && heading_objective <= 270) {
		    heading_objective = 270 - heading_objective + 180;
		} else if (270 < heading_objective && heading_objective < 360) {
		    heading_objective = 360 - heading_objective + 180;
		}

		// failsafe
		if (heading_objective >= 360) {
		    heading_objective -= 360;
		} else if (heading_objective < 0) {
		    heading_objective += 360;
		}
		*/

		degree_t heading_error = heading_objective - current_heading;
		if (heading_error > 180_deg)
			heading_error -= 360_deg;
		else if (heading_error < -180_deg)
			heading_error += 360_deg;

		if (fabs(heading_error) > 360_deg) heading_error = heading_objective;

		/*
		// heading failsafe
		if (heading_objective > 180 || heading_objective < -180) {
		    heading_objective += heading_objective > 0 ? -360 : 360;
		}

		double heading_error = heading_objective - current_heading;
		if (fabs(heading_error) > 180) {
		    heading_error += heading_error > 0 ? -360 : 360;
		}
		*/

		// BOOKMARK: Movement
		drive_left.move(
		    voltage_constant * (180 - fabs(heading_error.to<double>())) / 360 +
		    (127 - voltage_constant) * (heading_error.to<double>()) / 360
		);
		drive_right.move(
		    voltage_constant * (180 - fabs(heading_error.to<double>())) / 360 -
		    (127 - voltage_constant) * (heading_error.to<double>()) / 360
		);
		// delay to prevent brain crashing
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
	points.clear();
}
