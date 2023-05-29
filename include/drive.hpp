#pragma once
#include "main.h"

namespace drive {
	void select_drive_mode(std::string drive_mode);
	void tank_drive();
	void arcade_drive();
	void curvature_drive();
}