#pragma once
#include "main.h"
#include "pid.hpp"

namespace chassis {

void drive(float distance);

void turn_abs(float heading);

void turn_rel(float degrees);

} // namespace chassis