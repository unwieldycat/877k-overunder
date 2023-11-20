#pragma once
#include "main.h"

class Point {
  public:
	foot_t x, y;
	bool specify_angle, left, right;
	degree_t angle;

	Point(foot_t x, foot_t y) : x(x), y(y) {}
	Point(foot_t x, foot_t y, bool sp_a, degree_t a) : x(x), y(y), specify_angle(sp_a), angle(a) {}

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