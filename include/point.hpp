#pragma once
#include "main.h"
#include "units.h"

class Point {
  public:
	foot_t x, y;
	units::dimensionless::scalar_t curvature;
	bool left, right;

	Point(foot_t x, foot_t y) : x(x), y(y) {}
	Point(foot_t x, foot_t y, units::dimensionless::scalar_t curvature) : x(x), y(y), curvature(curvature) {}

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