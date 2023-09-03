#include "pursuit.hpp"
#include "devices.hpp"
#include "odom.hpp"
#include "pros/motors.h"

// Vectors to store all the coordinates for the robot to make a path for
std::vector<double> points_x = {0};
std::vector<double> points_y = {0};

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
 * of the delay between each while loop (currently 300ms).
 * The voltage constant determines the voltage to the motors if it is driving on a straight path.
 * The current objective increments each time the point from the list of coordinates is reached to
 * find the next path the robot should be taking.
 * The heading objective is what direction the robot should be facing to return to the path.
 * Lowest x and y, and highest x and y are coordinates to define a restricted zone that the robot
 * cannot go in. It is a failsafe to prevent the function from ever finding or going toward an
 * disqualifying area such as a goal or the opponent's side.
 */
void pursuit::pursuit(
    double lookahead_Distance, int voltage_constant, int lowest_x = 0, int lowest_y = 0,
    int highest_x = 0, int highest_y = 0
) {
	int current_point = 1;
	double slope_par, const_par, closest_point, slope_perp, const_perp;
	double next_objective_x, next_objective_y, heading_objective;
	bool restricted = false;

	drive_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	drive_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// Checking if a restriction zone is set
	if (highest_x - lowest_x != 0 && highest_y - lowest_y != 0) {
		restricted = true;
	}

	// Loops until all points have been passed
	while (current_point < points_x.size()) {
		double current_heading = imu.get_heading(), current_posX = odom::get_x(),
		       current_posY = odom::get_y();

		// BOOKMARK: Begin finding next objective
		//  Finds closest point when X coordinates of previous and current points are different
		if (points_x[current_point - 1] != points_x[current_point]) {
			slope_par = (points_y[current_point] - points_y[current_point - 1]) /
			            (points_x[current_point] - points_x[current_point - 1]);
			const_par = (points_y[current_point] - slope_par * points_x[current_point]);
			closest_point = current_posX;

			int sign = (points_x[current_point] - points_x[current_point - 1]) > 0 ? 1 : -1;
			next_objective_x =
			    (-(2 * (slope_par * (const_par - current_posY) - current_posX)) +
			     sign * sqrt(
			                pow((2 * (slope_par * (const_par - current_posY) - current_posX)), 2) -
			                4 * (pow(slope_par, 2) + 1) *
			                    (pow(current_posX, 2) + pow(const_par - current_posY, 2) -
			                     pow(lookahead_Distance, 2))
			            )) /
			    (2 * (pow(slope_par, 2) + 1));

			next_objective_y = slope_par * next_objective_x + const_par;

		} else {
			// X coordinates of previous and current point are the same
			int sign = (points_y[current_point] - points_y[current_point - 1]) > 0 ? 1 : -1;
			next_objective_x = points_x[current_point];
			next_objective_y =
			    sign * sqrt(pow(lookahead_Distance, 2) - pow(next_objective_x - current_posX, 2)) +
			    current_posY;
		}

		// BOOKMARK: Begin distance calculations for closest point
		//  Finds distance between goal and robot when Y coordinates of previous and current
		//  points are different
		if (fabs(points_y[current_point] - points_y[current_point - 1]) > 0.1) {
			slope_perp = -(points_x[current_point] - points_x[current_point - 1]) /
			             (points_y[current_point] - points_y[current_point - 1]);
			const_perp = (current_posY - slope_perp * current_posX);
			closest_point = (const_perp - const_par) / (slope_par - slope_perp);

			// Robot is more than the lookahead distance away from the path
			if (sqrt(
			        pow(closest_point - current_posX, 2) +
			        pow((slope_par * closest_point + const_par - current_posY), 2)
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
			points_x.insert(points_x.begin(), current_point, 1);
			points_y.insert(points_y.begin(), current_point, 2);
			continue;
		}

		// BOOKMARK: Goal increment
		if (fabs(next_objective_x - points_x[current_point]) < 0.1) {
			current_point++;
		}

		// BOOKMARK: Heading calculations
		//  Find math heading objective
		heading_objective =
		    atan2(next_objective_x - current_posY, next_objective_y - current_posX) * 180 / M_PI;
		if (heading_objective < 0) {
			heading_objective += 360;
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

		double heading_error = heading_objective - current_heading;
		if (heading_error > 180)
			heading_error -= 360;
		else if (heading_error < -180)
			heading_error += 360;

		if (fabs(heading_error) > 360) heading_error = heading_objective;

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
		    voltage_constant * (180 - fabs(heading_error)) / 360 +
		    (127 - voltage_constant) * (heading_error) / 360
		);
		drive_right.move(
		    voltage_constant * (180 - fabs(heading_error)) / 360 -
		    (127 - voltage_constant) * (heading_error) / 360
		);
		// delay to prevent brain crashing
		pros::delay(20);
	}

	drive_left.brake();
	drive_right.brake();
	points_x.clear();
	points_y.clear();
}
