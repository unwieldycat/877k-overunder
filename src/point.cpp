#include "point.hpp"
#include <utility>
using namespace units::math;

units::dimensionless::scalar_t Point::calc_par_slope(Point a, Point b) {
	return (b.y - a.y) / (b.x - a.x);
}
units::dimensionless::scalar_t Point::calc_per_slope(Point a, Point b) {
	return -(b.x - a.x) / (b.y - a.y);
}

foot_t Point::calc_const(Point a, units::dimensionless::scalar_t slope) {
	return a.y - slope * a.x;
}

foot_t Point::calc_dist(Point a, Point b){
	return sqrt(pow<2>(b.x - a.x) + pow<2>(b.y - a.y));
}

std::pair<foot_t, foot_t> Point::xy_dist(Point a, Point b){
	return std::make_pair(b.x - a.x, b.y - a.y);
}
