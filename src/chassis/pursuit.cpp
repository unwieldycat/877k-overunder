#include "devices.hpp"
#include "main.h"
#include "odom.hpp"
#include "units.h"
#include <cmath>
#include <fstream>

using namespace units::math;

std::vector<Point> points = {};
std::stringstream path_logs;

// ============================ Helper Functions ============================ //

// File format: x_coord, y_coord, kappa, left_wing, right_wing
// Warning: CSV keys are ignored, columns must be in the correct order
void parse_file(std::string file_path) {
	if (!pros::usd::is_installed()) return;

	std::ifstream file(file_path);
	if (!file) return;

	std::string line;
	std::getline(file, line); // Skip first line
	while (std::getline(file, line)) {
		// Remove spaces
		std::string::iterator new_end = std::remove(line.begin(), line.end(), ' ');
		line.erase(new_end, line.end());

		// Add comma to end of line if not present already
		if (!line.ends_with(',')) line.append(",");

		// Substring values by comma
		std::vector<std::string> values;
		int idx;

		while ((idx = line.find(',')) != std::string::npos) {
			values.push_back(line.substr(0, idx));
			line.erase(0, idx + 1);
		}

		// Parse values
		foot_t x = foot_t(std::stof(values.at(0)));
		foot_t y = foot_t(std::stof(values.at(1)));
		units::dimensionless::scalar_t curve = std::stof(values.at(2));
		bool left_wing = values.at(3) == "1";
		bool right_wing = values.at(4) == "1";

		// Create point
		points.push_back({x, y, curve, left_wing, right_wing});
	}

	file.close();
}

void record_error(Point goal, Point robot, degree_t heading) {
	path_logs << "Robot left path at ( " << robot.x << ", " << robot.y << ") trying to reach ( "
	          << goal.x << ", " << goal.y << ") with heading " << heading << std::endl;
}

void write_logs() {
	if (pros::usd::is_installed()) {
		std::ofstream file("/usd/paths/path_deviations.txt");
		file << path_logs.rdbuf();
		file.close();
	}

	path_logs.str("");
	path_logs.clear();
}

// ============================ Public functions ============================ //

void chassis::add_point(
    foot_t x_ft, foot_t y_ft, units::dimensionless::scalar_t curvature, bool left_wing,
    bool right_wing
) {
	if (points[points.size() - 1].x != x_ft || points[points.size() - 1].y != y_ft) {
		points.push_back(Point(x_ft, y_ft, curvature, left_wing, right_wing));
	}
}

void chassis::pursuit(std::string file_path, bool backwards) {
	parse_file(file_path);
	if (points.empty()) return;

	// Calculation assisting variables:
	units::dimensionless::scalar_t slope_par, slope_perp;
	foot_t const_par, const_perp;
	double slope_parD, const_parD, lookaheadD;
	double a, b, c, x;
	auto lookahead_distance =
	    (1_ft / (points[0].curvature) < 0.8_ft ? 1_ft / (points[0].curvature) : 0.8_ft);

	// Location tracking variables:
	foot_t closest_point, next_objective_x, next_objective_y;
	foot_t prev_posX, prev_posY, x_change;
	foot_t current_posX = odom::get_x(), current_posY = odom::get_y();
	double current_posXD, current_posYD;
	degree_t heading_objective, current_heading = (degree_t)imu.get_heading();
	degree_t heading_error, prev_errorh;

	// Counting variables:
	int current_point = 1, same_obj = 0;

	// Movement variables:
	double kk = 1, kh = 1, kf = 1, left_speed, right_speed;

	// BOOKMARK: Initialize
	drive_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	left_wing.set_value(points[0].left_wing);
	left_wing.set_value(points[0].right_wing);

	// Loops until all points have been passed
	while (current_point < points.size()) {
		// BOOKMARK: Update variables
		current_posX = odom::get_x();
		current_posY = odom::get_y();
		current_heading = (degree_t)imu.get_heading();

		// BOOKMARK: Begin finding next objective
		//  Finds closest point when X coordinates of previous and current points are different
		if (points[current_point - 1].x != points[current_point].x) {
			slope_par = Point::calc_par_slope(points[current_point - 1], points[current_point]);
			const_par = Point::calc_const(points[current_point], slope_par);
			closest_point = current_posX;

			int sign = (points[current_point].x - points[current_point - 1].x) > 0_ft ? 1 : -1;

			a = pow(slope_parD, 2) + 1;
			b = 2.0 * (slope_parD * (const_parD - current_posYD) - current_posXD);
			c = pow(current_posXD, 2) + pow(const_parD - current_posYD, 2) - pow(lookaheadD, 2);

			x = ((-b + sign * sqrt(pow(b, 2) - 4 * a * c)) / (2 * a));
			next_objective_x =
			    (pow(b, 2) - 4 * a * c) > 0 ? (foot_t)(x) : next_objective_x + x_change;

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
			    ) > 1.5 * lookahead_distance) {
				// NOTE: add action to bring robot back to path
				drive_left.brake();
				drive_right.brake();
				record_error(
				    Point(points[current_point].x, points[current_point].y),
				    Point(current_posX, current_posY), (degree_t)(imu.get_heading())
				);

				std::cout << "Exiting pursuit: robot is too far from path\n";
				break;
			}
		} else {
			// if both points have the same y coordinates, the x coordinates of the robot cannot be
			// more than lookahead distance to the found point
			if (fabs(points[current_point].x - current_posX) > 1.5 * lookahead_distance) {
				drive_left.brake();
				drive_right.brake();
				record_error(
				    Point(points[current_point].x, points[current_point].y),
				    Point(current_posX, current_posY), (degree_t)(imu.get_heading())
				);
				std::cout << "Exiting pursuit: robot is too far from path\n";
				break;
			}
		}

		// BOOKMARK: If the robot is stuck, it will try to find a point that is in a different
		// direction
		if (abs(current_posX - prev_posX) < 0.001_ft && abs(current_posY - prev_posY) < 0.001_ft) {
			// same_obj++;
		}
		if (same_obj >= 3) {
			if (left_wing.is_extended() || right_wing.is_extended()) {
				left_wing.retract();
				right_wing.retract();
				continue;
			}
			while (fabs(
			           atan2(
			               points[current_point].y - current_posY,
			               points[current_point].x - current_posX
			           ) -
			           atan2(
			               points[current_point + 1].y - current_posY,
			               points[current_point + 1].x - current_posX
			           )
			       ) < 20_deg) {
				current_point++;
				if (current_point > points.size() - 1) {
					drive_left.brake();
					drive_right.brake();
					break;
				}
			}

			same_obj = 0;
		}

		// BOOKMARK: Goal increment
		if (fabs(next_objective_x - points[current_point].x) < 0.05_ft &&
		    fabs(next_objective_y - points[current_point].y) < 0.05_ft) {
			current_point++;
			left_wing.set_value(points[current_point].left_wing);
			left_wing.set_value(points[current_point].right_wing);
			lookahead_distance =
			    (1_ft / (points[0].curvature) < 0.8_ft ? 1_ft / (points[0].curvature) : 0.8_ft);
		}

		// BOOKMARK: Heading calculations
		//  Find heading objective
		heading_objective = atan2(next_objective_y - current_posY, next_objective_x - current_posX);
		if (backwards) heading_objective += 180_deg;
		while (heading_objective < 0_deg) {
			heading_objective += 360_deg;
		}

		heading_error = heading_objective - current_heading;

		while (heading_error < -180_deg)
			heading_error += 360_deg;
		while (heading_error > 180_deg)
			heading_error -= 360_deg;

		// BOOKMARK: Movement
		int sign = backwards ? -1 : 1;

		// FIXME: curvature doesn't have direction
		double drive = (kf - (heading_error / 180_deg)) * 127;
		double turn =
		    127 * (heading_error / 180_deg) * kh + 127 * points[current_point - 1].curvature * kk;
		left_speed = sign * (drive + sign * turn);
		right_speed = sign * (drive - sign * turn);

		drive_left.move(left_speed);
		drive_right.move(right_speed);

		// BOOKMARK: Make adjustments to movement constants as needed
		if (fabs(left_speed) > 127 || fabs(right_speed) > 127) {
			kf *= 0.9;
			kh *= 0.9;
			kk *= 0.9;
		}
		if (fabs(heading_error) > fabs(prev_errorh)) {
			// FIXME: Constants change if error is increasing
		}

		// BOOKMARK: Change variables
		prev_errorh = heading_error;
		x_change = current_posX - prev_posX;
		prev_posX = current_posX;
		prev_posY = current_posY;

		// delay to prevent brain crashing
		pros::delay(200);
	}

	drive_left.brake();
	drive_right.brake();
	points.clear();
	write_logs();
}
