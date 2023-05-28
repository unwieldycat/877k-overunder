#include "pursuit.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pros/motors.h"

// Vectors to store all the coordinates for the robot to make a path for
std::vector<double> points_x;
std::vector<double> points_y;

/* Append an x and y coordinate to the current list
 * Coordinates must be in feet!
 * The positive direction is to the right(x) or forward(y) from the driver box
 */
void pursuit::add_point(double x_ft, double y_ft) {
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
 * Lowest x and y, and highest x and y are coordinates to define a restricted zone that the robot
 * cannot go in. It is a failsafe to prevent the function from ever finding or going toward an
 * disqualifying area such as a goal or the opponent's side.
 */
void pursuit::pursuit(
    int lookahead_Distance, int voltage_constant, int lowest_x = 0, int lowest_y = 0,
    int highest_x = 0, int highest_y = 0
) {
	int current_point = 1;
	double next_objective_x;
	double next_objective_y;
	double heading_objective;
	bool restricted;
	drive_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	if (highest_x - lowest_x != 0 && highest_y - lowest_y != 0) {
		restricted = true;
	}
	while (current_point < points_x.size()) {
		// X coordinates of previous and current points are different
		if (points_x[current_point - 1] != points_x[current_point]) {
			double slope_par = (points_y[current_point] - points_y[current_point - 1]) /
			                   (points_x[current_point] - points_x[current_point - 1]);
			double const_par = (points_y[current_point] - slope_par * points_x[current_point]);
			double closest_point = odom::get_x();
			// Y coordinates of previous and current points are different
			if (fabs(points_y[current_point] - points_y[current_point - 1]) > 0.1) {
				double slope_perp = -(points_x[current_point] - points_x[current_point - 1]) /
				                    (points_y[current_point] - points_y[current_point - 1]);
				double const_perp = (odom::get_y() - slope_perp * odom::get_x());
				closest_point = (const_perp - const_par) / (slope_par - slope_perp);
			}
			// Robot is more than the lookahead distance away from the path
			if (sqrt(
			        pow(closest_point - odom::get_x(), 2) +
			        pow((slope_par * closest_point + const_par - odom::get_y()), 2)
			    ) > lookahead_Distance) {
				// TODO: add action to bring robot back to path
				drive_left.brake();
				drive_right.brake();
				controller.set_text(1, 1, "Robot too far from path!");
				break;
			}
			int sign = (points_x[current_point] - points_x[current_point - 1]) > 0 ? 1 : -1;
			next_objective_x =
			    (-(2 * (slope_par * (const_par - odom::get_y()) - odom::get_x())) +
			     sign *
			         sqrt(
			             pow((2 * (slope_par * (const_par - odom::get_y()) - odom::get_x())), 2) -
			             4 * (pow(slope_par, 2) + 1) *
			                 (pow(odom::get_x(), 2) + pow(const_par - odom::get_y(), 2) -
			                  pow(lookahead_Distance, 2))
			         )) /
			    (2 * (pow(slope_par, 2) + 1));
			next_objective_y = slope_par * next_objective_x + const_par;
			// Robot is at goal point
			if (fabs(next_objective_x - points_x[current_point]) < 0.1) {
				current_point++;
			}
		} else { // X coordinates of previous and current point are the same
			int sign = (points_y[current_point] - points_y[current_point - 1]) > 0 ? 1 : -1;
			next_objective_x = points_x[current_point];
			next_objective_y =
			    sign * sqrt(pow(lookahead_Distance, 2) - pow(next_objective_x - odom::get_x(), 2)) +
			    odom::get_y();
			if (fabs(next_objective_y - points_y[current_point]) < 0.1) {
				current_point++;
			}
		}
		// Check if goal is in a restricted zone
		if (restricted &&
		    (lowest_x < next_objective_x < highest_x || lowest_y < next_objective_y < highest_y)) {
			// TODO: change coordinates to corner of the goal
			drive_left.brake();
			drive_right.brake();
			controller.set_text(1, 1, "Can't go there! Rerouted-");
			points_x.insert(points_x.begin(), current_point, 1);
			points_y.insert(points_y.begin(), current_point, 2);
			continue;
		}
		// The desired heading is converted to a compass heading
		heading_objective =
		    90 - (atan2(next_objective_x - odom::get_y(), next_objective_y - odom::get_x()) * 180 /
		          M_PI);
		// heading failsafe
		if (heading_objective > 180 || heading_objective < -180) {
			heading_objective += heading_objective > 0 ? -360 : 360;
		}
		double heading_error = heading_objective - imu.get_heading();
		if (fabs(heading_error) > 180) {
			heading_error += heading_error > 0 ? -360 : 360;
		}
		drive_left.move(
		    voltage_constant * (180 - fabs(heading_error)) / 180 +
		    (127 - voltage_constant) * (heading_error) / 180
		);
		drive_right.move(
		    voltage_constant * (180 - fabs(heading_error)) / 180 -
		    (127 - voltage_constant) * (heading_error) / 180
		);
		// delay to prevent brain crashing
		pros::delay(50);
	}
	drive_left.brake();
	drive_right.brake();
	points_x.clear();
	points_y.clear();
}