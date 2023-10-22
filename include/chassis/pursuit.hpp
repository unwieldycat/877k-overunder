#pragma once
#include "devices.hpp"
#include "main.h"
#include "units.h"

namespace chassis {
namespace pursuit {

class Point {
  public:
	foot_t xCoord, yCoord;
	bool specify_angle;
	degree_t angle;
	static int with_angle;
	Point(foot_t x, foot_t y) : xCoord(x), yCoord(y) {}
	Point(foot_t x, foot_t y, bool sp_a, degree_t a)
	    : xCoord(x), yCoord(y), specify_angle(sp_a), angle(a) {}
	/**
	 * @brief Finds slope from point a to point b
	 *
	 * @param a beginning of line segment
	 * @param b end of line segment
	 * @return units::dimensionless::scalar_t
	 */
	static units::dimensionless::scalar_t calc_par_slope(Point a, Point b);
	/**
	 * @brief Finds slope perpendicular of point a to point b
	 *
	 * @param a beginning of line segment
	 * @param b end of line segment
	 * @return units::dimensionless::scalar_t
	 */
	static units::dimensionless::scalar_t calc_perp_slope(Point a, Point b);
	/**
	 * @brief Find constant of an equation in the form y = mx + b
	 *
	 * @param a random point on the line
	 * @param slope slope of line
	 * @return foot_t
	 */
	static foot_t calc_const(Point a, units::dimensionless::scalar_t slope);
};

/**
 * @brief Create vector of pairs to store points for the path with initial point 0, 0
 *
 */
extern std::vector<chassis::pursuit::Point> points;

/**
 * @brief adds a point to the end of the points for the path
 *
 * @param x_ft x coordinate in feet
 * @param y_ft y coordinate in feet
 */
void add_point(foot_t x_ft, foot_t y_ft, bool need_angle = false, degree_t = 0.0_deg);

/**
 * @brief Begins moving the robot following all the points stored previous to the running of this
 * function, clears the points after running
 *
 * @param lookahead_Distance How far away the point the robot is "Chasing" is
 * @param voltage_constant How fast the robot will be moving forward if it is traveling on a
 * straight path
 * @param lowest_x Bottom left x coordinate in feet of the restricted area
 * @param lowest_y Bottom left y coordinate in feet of the restricted area
 * @param highest_x Bottom right x coordinate in feet of the restricted area
 * @param highest_y Bottom right y coordinate in feet of the restricted area
 */
void pursuit(
    foot_t lookahead_distance, int voltage_constant, foot_t lowest_x = 0_ft, foot_t lowest_y = 0_ft,
    foot_t highest_x = 0_ft, foot_t highest_y = 0_ft
);

} // namespace pursuit
} // namespace chassis