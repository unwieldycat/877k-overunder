#pragma once
#include "main.h"

namespace drive {
enum class DriveMode { TANK, ARCADE, CURVE };

void tank_drive();
void arcade_drive();
void curvature_drive();
void drive();
void set_mode(DriveMode drive_mode);
DriveMode get_mode();
} // namespace drive