#pragma once
#include "main.h"
#include "pid.hpp"

namespace chassis {

void drive(double distance);

void turn_abs(double heading);

void turn_rel(double degrees);

} // namespace chassis