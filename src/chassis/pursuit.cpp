#include "devices.hpp"
#include "main.h"
#include <fstream>

using namespace units::math;

std::vector<Point> points = {};
std::stringstream path_logs;
const double turn_const = 1.5;

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

	int current_point = 1, same_obj = 0;
	units::dimensionless::scalar_t slope_par, slope_perp;
	foot_t closest_point, next_objective_x, next_objective_y, const_par, const_perp;
	foot_t prev_objective_x = points[0].x, prev_objective_y = points[0].y;
	degree_t heading_objective;
	double left_speed, right_speed;

	drive_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	left_wing.set_value(points[0].left_wing);
	left_wing.set_value(points[0].right_wing);

	// Loops until all points have been passed
	while (current_point < points.size()) {
		degree_t current_heading = (degree_t)imu.get_heading(), heading_error;
		foot_t current_posX = odom::get_x(), current_posY = odom::get_y();
		// FIXME: 6 feet lookahead distance??
		auto lookahead_distance =
		    (1_ft / (points[current_point - 1].curvature) < 6_in
		         ? 1_ft / (points[current_point - 1].curvature)
		         : 0.05_ft);

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
				record_error(
				    Point(points[current_point].x, points[current_point].y),
				    Point(current_posX, current_posY), (degree_t)(imu.get_heading())
				);
				break;
			}
		} else {
			// if both points have the same y coordinates, the x coordinates of the robot cannot be
			// more than lookahead distance to the found point
			if (fabs(points[current_point].x - current_posX) > lookahead_distance) {
				drive_left.brake();
				drive_right.brake();
				record_error(
				    Point(points[current_point].x, points[current_point].y),
				    Point(current_posX, current_posY), (degree_t)(imu.get_heading())
				);
				break;
			}
		}

		// BOOKMARK: If the robot is stuck, it will try to find a point that is in a different
		// direction
		if (abs(next_objective_x - prev_objective_x) < 0.05_ft &&
		    abs(next_objective_y - prev_objective_y) < 0.05_ft) {
			same_obj++;
		}
		if (same_obj >= 3) {
			if (left_wing.is_extended() || right_wing.is_extended()) {
				left_wing.retract();
				right_wing.retract();
				continue;
			}
			while (abs(atan2(prev_objective_y - current_posY, prev_objective_x - current_posX) -
			           atan2(next_objective_y - current_posY, next_objective_x - current_posX)) <
			       20_deg) {
				current_point++;
			}
			if (current_point >= points.size()) {
				drive_left.brake();
				drive_right.brake();
				break;
			}
			same_obj = 0;
		}

		// BOOKMARK: Goal increment
		if (fabs(next_objective_x - points[current_point].x) < 0.1_ft &&
		    fabs(next_objective_y - points[current_point].y) < 0.1_ft) {
			current_point++;
			left_wing.set_value(points[current_point].left_wing);
			left_wing.set_value(points[current_point].right_wing);
			std::cout << "next!" << std::endl;
		}

		// BOOKMARK: Heading calculations
		//  Find math heading objective
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
		double max_speed = (1 - points[current_point - 1].curvature) < 0.0
		                       ? 0.05
		                       : (1 - points[current_point - 1].curvature).to<double>();
		double drive = 127 * max_speed * (360_deg - heading_error) / 360_deg;
		double turn = sign * 127 * heading_error / 180_deg * turn_const;
		left_speed = sign * (drive + turn);
		right_speed = sign * (drive - turn);

		drive_left.move(left_speed);
		drive_right.move(right_speed);

		prev_objective_x = next_objective_x;
		prev_objective_y = next_objective_y;

		// delay to prevent brain crashing
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
	points.clear();
	write_logs();
}
