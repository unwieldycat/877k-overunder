#include "main.h"

namespace chassis {

typedef enum drive_mode { DRIVE_MODE_ARCADE, DRIVE_MODE_CURVE, DRIVE_MODE_TANK } drive_mode_t;

/**
 * @brief Set the drive mode
 */
void set_drive_mode(drive_mode_t mode);

/**
 * @brief User drive control function
 */
void user_drive();

/**
 * @brief Reverse the drive direction
 */
void reverse();

} // namespace chassis