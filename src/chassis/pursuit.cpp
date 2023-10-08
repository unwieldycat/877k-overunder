#include "pursuit.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pros/motors.h"
#include "units.h"
#include <ostream>
#include <utility>
using namespace units::math;

std::vector<std::pair<int, degree_t>> specify_angles;
std::vector<std::pair<foot_t, foot_t>> points = {std::make_pair(0_ft, 0_ft)};

void chassis::pursuit::add_point(
    foot_t x_ft, foot_t y_ft, bool need_angle, degree_t specify_angle
) {
	if (points[points.size() - 1].first != x_ft || points[points.size() - 1].second != y_ft) {
		points.push_back({x_ft, y_ft});
	}
	if (need_angle) {
		specify_angles.push_back({points.size() - 1, specify_angle});
	}
}

void chassis::pursuit::pursuit(
    foot_t lookahead_Distance, int voltage_constant, foot_t lowest_x, foot_t lowest_y,
    foot_t highest_x, foot_t highest_y
) {
	int current_point = 1;
	int current_angle_tracker = -1;
	if (specify_angles.size() > 1) current_angle_tracker++;
	units::dimensionless::scalar_t slope_par, slope_perp;
	foot_t closest_point, next_objective_x, next_objective_y, const_par, const_perp;
	degree_t heading_objective;
	bool restricted = false;
	double left_speed, right_speed;

	drive_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// Checking if a restriction zone is set
	if (highest_x - lowest_x != 0_ft && highest_y - lowest_y != 0_ft) {
		restricted = true;
	}

	// Checking if there are any points that need a specific angle
	// Needs calculations for second point and inf slope
	if (specify_angles.size() == 0) {
		for (int i = 0; i < specify_angles.size(); i++) {
			if (specify_angles[i].first > 1 &&
			    (points[specify_angles[i].first - 1].second -
			     points[specify_angles[i].first - 2].second) != 0_ft) {
				double prev_slope = (points[specify_angles[i].first - 1].second -
				                     points[specify_angles[i].first - 2].second) /
				                    (points[specify_angles[i].first - 1].first -
				                     points[specify_angles[i].first - 2].first),
				       new_slope = sin((radian_t)specify_angles[i].second) /
				                   cos((radian_t)specify_angles[i].second);
				foot_t const_prev = prev_slope * points[specify_angles[i].first - 2].first -
				                    points[specify_angles[i].first - 2].second,
				       new_const = new_slope * points[specify_angles[i].first].first -
				                   points[specify_angles[i].first].second;
				foot_t intersect_x = (new_const - const_prev) / (prev_slope - new_slope),
				       intersect_y = new_slope * intersect_x;
				points.insert(
				    points.begin(), specify_angles[i].first + i, {intersect_x, intersect_y}
				);
				specify_angles[i].first += i;
			} else if (specify_angles[i].first == 1 ||
						points[specify_angles[i].first - 1].second - 
						points[specify_angles[i].first - 2].second == 0_ft) {
				double new_slope = sin((radian_t)specify_angles[i].second) /
				                   cos((radian_t)specify_angles[i].second);
				foot_t new_const = new_slope * points[specify_angles[i].first].first -
				                   points[specify_angles[i].first].second,
				       intersect_x = points[specify_angles[i].first - 1].first,
				       intersect_y = new_slope * intersect_x + new_const;
				points.insert(
				    points.begin(), specify_angles[i].first + i, {intersect_x, intersect_y}
				);
				specify_angles[i].first += i;
			}
		}
	}

	// Loops until all points have been passed
	while (current_point < points.size()) {
		degree_t current_heading = (degree_t)imu.get_heading(), heading_error;
		foot_t current_posX = odom::get_x(), current_posY = odom::get_y();
		// BOOKMARK: Make robot actually follow specified angles
		while (current_angle_tracker > -1 &&
		       specify_angles[current_angle_tracker].first == current_point - 1 &&
		       fabs(current_heading - specify_angles[current_angle_tracker].second) > 1_deg) {
			heading_objective = specify_angles[current_angle_tracker].second;
			heading_error = heading_objective - current_heading;

			if (heading_error < -180_deg)
				heading_error += 360_deg;
			else if (heading_error > 180_deg)
				heading_error -= 360_deg;

			drive_left.move(voltage_constant * heading_error.to<double>() / 90);
			drive_right.move(-voltage_constant * heading_error.to<double>() / 90);
		}

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
		} else {
			// if both points have the same y coordinates, the x coordinates of the robot cannot be
			// more than lookahead distance to the found point
			if (fabs(points[current_point].first - current_posX) > lookahead_Distance) {
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
			std::cout << "Can't go there! Rerouted-" << std::endl;

			if (lowest_x < points[current_point].first && points[current_point].first < highest_x &&
			    lowest_y < points[current_point].second &&
			    points[current_point].second < highest_y) {
				std::cout << "invalid point" << std::endl;
				current_point++;
				continue;
			}

			if (points[current_point].first > points[current_point - 1].first) {
				points.insert(points.begin(), current_point, {lowest_x - 5_in, highest_y + 5_in});
			} else if (points[current_point].first < points[current_point - 1].first) {
				points.insert(points.begin(), current_point, {highest_x + 5_in, highest_y + 5_in});
			}

			continue;
		}

		// BOOKMARK: Goal increment
		if (fabs(next_objective_x - points[current_point].first) < 0.1_ft &&
		    fabs(next_objective_y - points[current_point].second) < 0.1_ft) {
			current_point++;
		}

		// BOOKMARK: Heading calculations
		//  Find math heading objective
		heading_objective = atan2(next_objective_y - current_posY, next_objective_x - current_posX);
		if (heading_objective < 0_deg) {
			heading_objective += 360_deg;
		}

		heading_error = heading_objective - current_heading;

		if (heading_error < -180_deg)
			heading_error += 360_deg;
		else if (heading_error > 180_deg)
			heading_error -= 360_deg;

		// BOOKMARK: Movement
		if (fabs(heading_error) < 2_deg) {
			left_speed = 127;
			right_speed = 127;

		} else if (fabs(heading_error) < 15_deg) {
			left_speed = voltage_constant * (180 - fabs(heading_error.to<double>())) / 180 +
			             (127 - voltage_constant) * (heading_error.to<double>()) / 180;
			right_speed = voltage_constant * (180 - fabs(heading_error.to<double>())) / 180 -
			              (127 - voltage_constant) * (heading_error.to<double>()) / 180;

		} else if (fabs(heading_error) < 45_deg) {
			left_speed = voltage_constant * (180 - fabs(heading_error.to<double>())) / 360 +
			             (127 - voltage_constant) * (heading_error.to<double>()) / 90;
			right_speed = voltage_constant * (180 - fabs(heading_error.to<double>())) / 360 -
			              (127 - voltage_constant) * (heading_error.to<double>()) / 90;

		} else {
			left_speed = voltage_constant * heading_error.to<double>() / 90;
			right_speed = -voltage_constant * heading_error.to<double>() / 90;
		}

		drive_left.move(left_speed);
		drive_right.move(right_speed);

		// delay to prevent brain crashing
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
	points.clear();
}