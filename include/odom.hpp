#pragma once
#include "main.h"

namespace odom {

void initialize();
std::pair<double, double> local_to_global_coords(double local_x, double local_y, double robot_heading);
std::pair<double, double> global_to_local_coords(
    double global_x, double global_y, double robot_x, double robot_y, double robot_heading
);
void track_position();
void calibrate(double robot_x, double robot_y, double heading);
double get_x();
double get_y();

}; // namespace odom