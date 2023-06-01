#pragma once
#include "main.h"

namespace drive {
void tank_drive();
void arcade_drive();
void curvature_drive();
void drive();
void select_drive_mode(int drive_mode);
void select_drive_mode(std::string drive_mode);
int get_drive_mode();
std::string get_drive_name();
} // namespace drive