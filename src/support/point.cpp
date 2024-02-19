#include "point.hpp"
#include "main.h"
#include <utility>

using namespace units::math;

// All Constructors
Point::Point(foot_t x, foot_t y) : Point(x, y, 0, false, false) {}
Point::Point(foot_t x, foot_t y, units::angle::degree_t heading) : x(x), y(y), heading(heading) {}
Point::Point(foot_t x, foot_t y, units::dimensionless::scalar_t curvature)
    : Point(x, y, curvature, false, false) {}
Point::Point(
    foot_t x, foot_t y, units::dimensionless::scalar_t curvature, bool left_wing, bool right_wing
)
    : x(x), y(y), curvature(curvature), left_wing(left_wing), right_wing(right_wing) {}

// Calculation Methods
units::dimensionless::scalar_t Point::calc_par_slope(Point a, Point b) {
	return (b.y - a.y) / (b.x - a.x);
}
units::dimensionless::scalar_t Point::calc_per_slope(Point a, Point b) {
	return -(b.x - a.x) / (b.y - a.y);
}

foot_t Point::calc_const(Point a, units::dimensionless::scalar_t slope) {
	return a.y - slope * a.x;
}

foot_t Point::calc_dist(Point a, Point b) { return sqrt(pow<2>(b.x - a.x) + pow<2>(b.y - a.y)); }

std::pair<foot_t, foot_t> Point::xy_dist(Point a, Point b) {
	return std::make_pair(b.x - a.x, b.y - a.y);
}

// Mutator Functions
void Point::update(foot_t tx, foot_t ty, units::angle::degree_t theading) {
	x = tx;
	y = ty;
	heading = theading;
}
