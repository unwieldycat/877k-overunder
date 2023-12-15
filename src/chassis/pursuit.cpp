#include "pursuit.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pros/motors.h"
#include "units.h"
#include <ostream>
#include <utility>
using namespace units::math;

std::vector<Point> chassis::pursuit::points = {};

void chassis::pursuit::add_point(
    foot_t x_ft, foot_t y_ft, units::dimensionless::scalar_t curvature
) {
	if (points[points.size() - 1].x != x_ft || points[points.size() - 1].y != y_ft) {
		points.push_back(Point(x_ft, y_ft, curvature));
	}
}

// TODO: wing toggle function
void chassis::pursuit::wing(char side) {}

void chassis::pursuit::pursuit() {
	int current_point = 1;
	units::dimensionless::scalar_t slope_par, slope_perp;
	foot_t closest_point, next_objective_x, next_objective_y, const_par, const_perp;
	degree_t heading_objective;
	double left_speed, right_speed;

	drive_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// Loops until all points have been passed
	while (current_point < points.size()) {
		degree_t current_heading = (degree_t)imu.get_heading(), heading_error;
		foot_t current_posX = odom::get_x(), current_posY = odom::get_y();
		auto lookahead_distance =
		    (1_ft / (points[current_point - 1].curvature) > 6_ft
		         ? 6_ft
		         : 1_ft / (points[current_point - 1].curvature));

		// BOOKMARK: Begin finding next objective
		//  Finds closest point when X coordinates of previous and current points are different
		if (points[current_point - 1].x != points[current_point].x) {
			slope_par = Point::calc_par_slope(points[current_point - 1], points[current_point]);
			const_par = Point::calc_const(points[current_point], slope_par);
			closest_point = current_posX;

			int sign = (points[current_point].x - points[current_point - 1].x) > 0_ft ? 1 : -1;
			next_objective_x =
			    (-(2 * (slope_par * (const_par - current_posY) - current_posX)) +
			     sign * sqrt(
			                pow<2>(2 * (slope_par * (const_par - current_posY) - current_posX)) -
			                4 * (pow<2>(slope_par) + 1) *
			                    (pow<2>(current_posX) + pow<2>(const_par - current_posY) -
			                     pow<2>(lookahead_distance))
			            )) /
			    (2 * (pow<2>(slope_par) + 1));

			next_objective_y = slope_par * next_objective_x + const_par;

		} else {
			// X coordinates of previous and current point are the same
			int sign = (points[current_point].y - points[current_point - 1].y) > 0_ft ? 1 : -1;
			next_objective_x = points[current_point].x;
			next_objective_y =
			    sign * sqrt(pow<2>(lookahead_distance) - pow<2>(next_objective_x - current_posX)) +
			    current_posY;
		}

		// BOOKMARK: Begin distance calculations for closest point
		//  Finds distance between goal and robot when Y coordinates of previous and current
		//  points are different
		if (fabs(points[current_point].y - points[current_point - 1].y) > 0.1_ft) {
			slope_perp = Point::calc_per_slope(points[current_point - 1], points[current_point]);
			const_perp = Point::calc_const(Point(current_posX, current_posY), slope_perp);
			closest_point = (const_perp - const_par) / (slope_par - slope_perp);

			// Robot is more than the lookahead distance away from the path
			if (sqrt(
			        pow<2>(closest_point - current_posX) +
			        pow<2>(slope_par * closest_point + const_par - current_posY)
			    ) > lookahead_distance) {
				// NOTE: add action to bring robot back to path
				drive_left.brake();
				drive_right.brake();
				std::cout << "Robot too far from path!" << std::endl;
				break;
			}
		} else {
			// if both points have the same y coordinates, the x coordinates of the robot cannot be
			// more than lookahead distance to the found point
			if (fabs(points[current_point].x - current_posX) > lookahead_distance) {
				drive_left.brake();
				drive_right.brake();
				std::cout << "Robot too far from path!" << std::endl;
				break;
			}
		}

		// BOOKMARK: Goal increment
		if (fabs(next_objective_x - points[current_point].x) < 0.1_ft &&
		    fabs(next_objective_y - points[current_point].y) < 0.1_ft) {
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
		left_speed =
		    127 * (1 - points[current_point - 1].curvature * 3) + 127 * heading_error / 360_deg;
		right_speed =
		    127 * (1 - points[current_point - 1].curvature * 3) - 127 * heading_error / 360_deg;

		drive_left.move(left_speed);
		drive_right.move(right_speed);

		// delay to prevent brain crashing
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
	points.clear();
	points.push_back(Point(0_ft, 0_ft));
}