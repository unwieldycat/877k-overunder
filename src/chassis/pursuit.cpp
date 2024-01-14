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
	Point robot(odom::get_x(), odom::get_y(), (degree_t)(imu.get_heading()));
	Point robot_prev(odom::get_x(), odom::get_y(), (degree_t)(imu.get_heading()));
	Point next_obj(0_ft, 0_ft, 0_deg);
	foot_t closest_point;
	foot_t x_change;
	double current_posXD, current_posYD;
	degree_t heading_error, prev_errorh;

	// Counting variables:
	int pursuing = 1, same_obj = 0;

	// Movement variables:
	double kk = 1, kh = 1, kf = 1, left_speed, right_speed;

	// Initialize
	drive_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	left_wing.set_value(points[0].left_wing);
	left_wing.set_value(points[0].right_wing);

	// Loops until all points have been passed
	while (pursuing < points.size()) {
		// Update variables
		robot.update(odom::get_x(), odom::get_y(), (degree_t)(imu.get_heading()));

		// BOOKMARK: Begin finding next objective
		//  Finds closest point when X coordinates of previous and current points are different
		int sign = (points[pursuing].x - points[pursuing - 1].x) > 0_ft ? 1 : -1;
		if (points[pursuing - 1].x != points[pursuing].x) {
			slope_par = Point::calc_par_slope(points[pursuing - 1], points[pursuing]);
			const_par = Point::calc_const(points[pursuing], slope_par);
			closest_point = robot.x;

			a = pow(slope_parD, 2) + 1;
			b = 2.0 * (slope_parD * (const_parD - current_posYD) - current_posXD);
			c = pow(current_posXD, 2) + pow(const_parD - current_posYD, 2) - pow(lookaheadD, 2);

			x = ((-b + sign * sqrt(pow(b, 2) - 4 * a * c)) / (2 * a));
			next_obj.x = (pow(b, 2) - 4 * a * c) > 0 ? (foot_t)(x) : next_obj.x + x_change;

			next_obj.y = slope_par * next_obj.x + const_par;

		} else {
			// X coordinates of previous and current point are the same
			next_obj.x = points[pursuing].x;
			next_obj.y =
			    sign * sqrt(pow<2>(lookahead_distance) - pow<2>(next_obj.x - robot.x)) + robot.y;
		}

		// BOOKMARK: Check if the point found is too far from the robot
		if (fabs(points[pursuing].y - points[pursuing - 1].y) > 0.1_ft) {
			slope_perp = Point::calc_per_slope(points[pursuing - 1], points[pursuing]);
			const_perp = Point::calc_const(Point(robot.x, robot.y), slope_perp);
			closest_point = (const_perp - const_par) / (slope_par - slope_perp);

			// Robot is more than the lookahead distance away from the path
			if (sqrt(
			        pow<2>(closest_point - robot.x) +
			        pow<2>(slope_par * closest_point + const_par - robot.y)
			    ) > 1.5 * lookahead_distance) {
				// NOTE: add action to bring robot back to path
				drive_left.brake();
				drive_right.brake();
				record_error(
				    Point(points[pursuing].x, points[pursuing].y), Point(robot.x, robot.y),
				    (degree_t)(imu.get_heading())
				);

				std::cout << "Exiting pursuit: robot is too far from path\n";
				break;
			}
		}

		// BOOKMARK: If the robot is stuck, it will try to find a point that is in a different
		// direction
		if (abs(robot.x - robot_prev.x) < 0.001_ft && abs(robot.y - robot_prev.y) < 0.001_ft) {
			same_obj++;
		}
		if (same_obj >= 3) {
			if (left_wing.is_extended() || right_wing.is_extended()) {
				left_wing.retract();
				right_wing.retract();
				continue;
			}
			// TODO: Do something when robot is stuck

			same_obj = 0;
		}

		// BOOKMARK: Goal increment
		if (fabs(next_obj.x - points[pursuing].x) < 0.05_ft &&
		    fabs(next_obj.y - points[pursuing].y) < 0.05_ft) {
			pursuing++;
			left_wing.set_value(points[pursuing].left_wing);
			left_wing.set_value(points[pursuing].right_wing);
			lookahead_distance =
			    (1_ft / (points[0].curvature) < 0.8_ft ? 1_ft / (points[0].curvature) : 0.8_ft);
		}

		// BOOKMARK: Heading calculations
		//  Find heading objective
		next_obj.heading = atan2(next_obj.y - robot.y, next_obj.x - robot.x);
		if (backwards) next_obj.heading += 180_deg;
		while (next_obj.heading < 0_deg) {
			next_obj.heading += 360_deg;
		}

		heading_error = next_obj.heading - robot.heading;

		while (heading_error < -180_deg)
			heading_error += 360_deg;
		while (heading_error > 180_deg)
			heading_error -= 360_deg;

		// BOOKMARK: Movement
		int dir = backwards ? -1 : 1;

		// FIXME: curvature doesn't have direction
		double drive = (kf - (heading_error / 180_deg)) * 127;
		double turn =
		    127 * (heading_error / 180_deg) * kh + 127 * points[pursuing - 1].curvature * kk;
		left_speed = dir * (drive + dir * turn);
		right_speed = dir * (drive - dir * turn);

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
		x_change = robot.x - robot_prev.x;
		robot_prev.x = robot.x;
		robot_prev.y = robot.y;

		// delay to prevent brain crashing
		pros::delay(200);
	}

	drive_left.brake();
	drive_right.brake();
	points.clear();
	write_logs();
}
