#include "pursuit.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pros/motors.h"
#include "units.h"
#include <ostream>
#include <utility>
using namespace units::math;

std::vector<chassis::pursuit::Point> chassis::pursuit::points = {Point(0_ft, 0_ft, 1)};

void chassis::pursuit::add_point(
    foot_t x_ft, foot_t y_ft, bool need_angle, degree_t specify_angle
) {
	if (points[points.size() - 1].xCoord != x_ft || points[points.size() - 1].yCoord != y_ft) {
		points.push_back(Point(x_ft, y_ft, need_angle, specify_angle, points.size() + 1));
		if (need_angle) Point::with_angle++;
	}
}

void chassis::pursuit::pursuit(
    foot_t lookahead_Distance, int voltage_constant, foot_t lowest_x, foot_t lowest_y,
    foot_t highest_x, foot_t highest_y
) {
	int current_point = 1;
	int current_angle_tracker = -1;
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
	for (int i = 0; i < points.size(); i++) {
		if (points[i].specify_angle) {
			if (points[i].position > 1 && points[i - 1].yCoord - points[i - 2].yCoord != 0_ft &&
			    points[i - 1].xCoord - points[i - 2].xCoord != 0_ft) {
				// BOOKMARK: Begin writing hereeeee
				double prev_slope =
				           chassis::pursuit::Point::calc_par_slope(points[i - 1], points[i - 2]),
				       new_slope = sin((radian_t)points[i].angle) / cos((radian_t)points[i].angle);
				foot_t const_prev = chassis::pursuit::Point::calc_const(points[i - 2], prev_slope),
				       new_const = chassis::pursuit::Point::calc_const(points[i], new_slope);
				foot_t intersect_x = (new_const - const_prev) / (prev_slope - new_slope),
				       intersect_y = new_slope * intersect_x;
				points.insert(points.begin(), i, chassis::pursuit::Point(intersect_x, intersect_y));
				for (int t = i; t < points.size(); t++) {
					points[t].push();
				}
			} else if (points[i].position == 1 || points[i - 1].xCoord - points[i - 2].xCoord == 0_ft) {
				double new_slope = sin((radian_t)points[i].angle) / cos((radian_t)points[i].angle);
				foot_t new_const = chassis::pursuit::Point::calc_const(points[i], new_slope),
				       intersect_x = points[i - 1].xCoord,
				       intersect_y = new_slope * intersect_x + new_const;
				points.insert(points.begin(), i, chassis::pursuit::Point(intersect_x, intersect_y));
				for (int t = i; t < points.size(); t++) {
					points[t].push();
				}
			}
		}
	}

	// Loops until all points have been passed
	while (current_point < points.size()) {
		degree_t current_heading = (degree_t)imu.get_heading(), heading_error;
		foot_t current_posX = odom::get_x(), current_posY = odom::get_y();
		// BOOKMARK: Make robot actually follow specified angles
		while (points[current_point].specify_angle &&
		       fabs(current_heading - points[current_point].angle) > 3_deg) {
			heading_objective = points[current_point].angle;
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
		if (points[current_point - 1].xCoord != points[current_point].xCoord) {
			slope_par = chassis::pursuit::Point::calc_par_slope(
			    points[current_point - 1], points[current_point]
			);
			const_par = chassis::pursuit::Point::calc_const(points[current_point], slope_par);
			closest_point = current_posX;

			int sign =
			    (points[current_point].xCoord - points[current_point - 1].xCoord) > 0_ft ? 1 : -1;
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
			    (points[current_point].yCoord - points[current_point - 1].yCoord) > 0_ft ? 1 : -1;
			next_objective_x = points[current_point].xCoord;
			next_objective_y =
			    sign * sqrt(pow<2>(lookahead_Distance) - pow<2>(next_objective_x - current_posX)) +
			    current_posY;
		}

		// BOOKMARK: Begin distance calculations for closest point
		//  Finds distance between goal and robot when Y coordinates of previous and current
		//  points are different
		if (fabs(points[current_point].yCoord - points[current_point - 1].yCoord) > 0.1_ft) {
			slope_perp = chassis::pursuit::Point::calc_perp_slope(
			    points[current_point - 1], points[current_point]
			);
			const_perp =
			    chassis::pursuit::Point::calc_const(Point(current_posX, current_posY), slope_perp);
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
			if (fabs(points[current_point].xCoord - current_posX) > lookahead_Distance) {
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

			if (lowest_x < points[current_point].xCoord &&
			    points[current_point].xCoord < highest_x &&
			    lowest_y < points[current_point].yCoord &&
			    points[current_point].yCoord < highest_y) {
				std::cout << "invalid point" << std::endl;
				current_point++;
				continue;
			}

			if (points[current_point].xCoord > points[current_point - 1].xCoord) {
				points.insert(
				    points.begin(), current_point,
				    Point(lowest_x - 5_in, highest_y + 5_in, current_point)
				);
			} else if (points[current_point].xCoord < points[current_point - 1].xCoord) {
				points.insert(
				    points.begin(), current_point,
				    Point(highest_x + 5_in, highest_y + 5_in, current_point)
				);
			}

			continue;
		}

		// BOOKMARK: Goal increment
		if (fabs(next_objective_x - points[current_point].xCoord) < 0.1_ft &&
		    fabs(next_objective_y - points[current_point].yCoord) < 0.1_ft) {
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

static units::dimensionless::scalar_t
calc_par_slope(chassis::pursuit::Point a, chassis::pursuit::Point b) {
	return (b.yCoord - a.yCoord) / (b.xCoord - a.xCoord);
}
static units::dimensionless::scalar_t
calc_per_slope(chassis::pursuit::Point a, chassis::pursuit::Point b) {
	return -(b.xCoord - a.xCoord) / (b.yCoord - a.yCoord);
}

static foot_t calc_const(chassis::pursuit::Point a, units::dimensionless::scalar_t slope) {
	return a.yCoord - slope * a.xCoord;
}