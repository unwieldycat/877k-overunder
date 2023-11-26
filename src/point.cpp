#include "point.hpp"

units::dimensionless::scalar_t Point::calc_par_slope(Point a, Point b) {
	return (b.y - a.y) / (b.x - a.x);
}
units::dimensionless::scalar_t Point::calc_per_slope(Point a, Point b) {
	return -(b.x - a.x) / (b.y - a.y);
}

foot_t Point::calc_const(Point a, units::dimensionless::scalar_t slope) {
	return a.y - slope * a.x;
}