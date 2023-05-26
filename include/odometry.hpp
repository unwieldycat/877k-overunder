#pragma once
#include "main.h"

namespace odom {

void initialize();
float local_to_global_coords(float local_x, float local_y, float robot_heading, bool return_x);
float global_to_local_coords(
    float global_x, float global_y, float robot_x, float robot_y, float robot_heading, bool return_x
);
void track_position();
void calibrate(float robot_x, float robot_y, float heading);
float get_x();
float get_y();

}; // namespace odom