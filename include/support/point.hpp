#pragma once
#include "units.h"

using namespace units::length;

class Point {
  public:
	foot_t x, y;
	units::dimensionless::scalar_t curvature;
	bool left_wing, right_wing;

	Point(foot_t x, foot_t y) : Point(x, y, 0, false, false) {}
	Point(foot_t x, foot_t y, units::dimensionless::scalar_t curvature)
	    : Point(x, y, curvature, false, false) {}
	Point(
	    foot_t x, foot_t y, units::dimensionless::scalar_t curvature, bool left_wing,
	    bool right_wing
	)
	    : x(x), y(y), curvature(curvature), left_wing(left_wing), right_wing(right_wing) {}

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
	static units::dimensionless::scalar_t calc_per_slope(Point a, Point b);

	/**
	 * @brief Find constant of an equation in the form y = mx + b
	 *
	 * @param a random point on the line
	 * @param slope slope of line
	 * @return foot_t
	 */
	static foot_t calc_const(Point a, units::dimensionless::scalar_t slope);

	static foot_t calc_dist(Point a, Point b);

	static std::pair<foot_t, foot_t> xy_dist(Point a, Point b);
};