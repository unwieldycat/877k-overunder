#include "pure_pursuit.hpp"

// Vectors to store all the coordinates for the robot to make a path for
std::vector<double> points_x;
std::vector<double> points_y;
// Robot coordinates and heading from odom, temporary placeholder variables
double robot_x, robot_y, heading;

/* Append an x and y coordinate to the current list
 * Coordinates must be in feet!
 * The positive direction is to the right(x) or forward(y) from the driver box
 */
void add_point(double x_ft, double y_ft) {
	if (x_ft != points_x[points_x.size() - 1] || y_ft != points_y[points_y.size() - 1]) {
		points_x.push_back(x_ft);
		points_y.push_back(y_ft);
	}
}

/*
 * The lookahead distance represents the distance that the robot can drive to in the amount of time
 * of the delay between each while loop (currently 50ms).
 * The voltage constant determines the voltage to the motors if it is driving on a straight path.
 * The current objective increments each time the point from the list of coordinates is reached to
 * find the next path the robot should be taking.
 * The heading objective is what direction the robot should be facing to return to the path.
 */
void pursuit(int lookahead_Distance, int voltage_constant) {
	int current_point = 1;
	double next_objective_x;
	double next_objective_y;
	double heading_objective;
	while (current_point < points_x.size()) {
		if (points_x[current_point - 1] != points_x[current_point]) {
			double slope_par = (points_y[current_point] - points_y[current_point - 1]) /
			                   (points_x[current_point] - points_x[current_point - 1]);
			double const_par = (points_y[current_point] - slope_par * points_x[current_point]);
			double slope_perp = -(points_x[current_point] - points_x[current_point - 1]) /
			                    (points_y[current_point] - points_y[current_point - 1]);
			double const_perp = (robot_y - slope_perp * robot_x);
			double closest_point = (const_perp - const_par) / (slope_par - slope_perp);
			if (sqrt(pow(closest_point, 2) + pow((slope_par * closest_point + const_par), 2)) >
			    lookahead_Distance) {
				// No points
				controller.set_text(1, 1, "Robot too far from path!");
				break;
			}
			int sign = (points_x[current_point] - points_x[current_point - 1]) > 0 ? 1 : -1;
			next_objective_x =
			    (-(2 * (slope_par * (const_par - robot_y) - robot_x)) +
			     sign * sqrt(
			                pow((2 * (slope_par * (const_par - robot_y) - robot_x)), 2) -
			                4 * (pow(slope_par, 2) + 1) *
			                    (pow(robot_x, 2) + pow(const_par - robot_y, 2) -
			                     pow(lookahead_Distance, 2))
			            )) /
			    (2 * (pow(slope_par, 2) + 1));
			next_objective_y = slope_par * next_objective_x + const_par;
			if (fabs(next_objective_x - points_x[current_point]) < 0.1) {
				current_point++;
			}
		} else {
			int sign = (points_y[current_point] - points_y[current_point - 1]) > 0 ? 1 : -1;
			next_objective_x = points_x[current_point];
			next_objective_y =
			    sign * sqrt(pow(lookahead_Distance, 2) - pow(next_objective_x - robot_x, 2)) +
			    robot_y;
			if (fabs(next_objective_y - points_y[current_point]) < 0.1) {
				current_point++;
			}
		}
		// The desired heading is converted to a compass heading
		heading_objective = 90 - atan2(next_objective_x - robot_y, next_objective_y - robot_x);
		// heading failsafe
		if (heading_objective > 180 || heading_objective < -180) {
			heading_objective += heading_objective > 0 ? -360 : 360;
		}
		double heading_error = heading_objective - heading;
		if (fabs(heading_error) > 180) {
			heading_error += heading_error > 0 ? -360 : 360;
		}
		// FIXME: declare motors and change variables to motor voltages
		double left_motor = voltage_constant * (180 - fabs(heading_error)) / 180 +
		                    (127 - voltage_constant) * (heading_error) / 180;
		double right_motor = voltage_constant * (180 - fabs(heading_error)) / 180 -
		                     (127 - voltage_constant) * (heading_error) / 180;
		// delay to prevent brain crashing
		pros::delay(50);
	}
}