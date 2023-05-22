#include "odometry.hpp"
#include "devices.hpp"

//Disregard for now, massive mess - will clean up
class Odometry {
private: 
	int odom_x = 0;
	int odom_y = 0;
	const float LEFT_OFFSET = 0;
	const float RIGHT_OFFSET = 0;
	const float REAR_OFFSET = 0;
	float left_odom_dist = 0;
	float right_odom_dist = 0;
	float rear_odom_dist = 0;
	float imu_heading = 0;
	float previous_heading = 0;
	float trackingX = 0;
	float trackingY = 0;
	float odomX = 0;
	float odomY = 0;

public:
	Odometry() {
		odom_left.reset_position();
		odom_right.reset_position();
		odom_rear.reset_position();
		odom_left.set_reversed(true);
		odom_right.set_reversed(false);
		odom_rear.set_reversed(true);
		imu.reset();
	}

	//Converts robot-centric coordinates to field-centric
	float localToGlobalCoords(float localX, float localY, float robotHeading, bool returnX = true) {
		float headingTraveled = robotHeading;
		float distanceTraveled = sqrt(pow(localX, 2) + pow(localY, 2));
		//printf("%f\n", distanceTraveled);
		if (localX >= 0 && localY > 0) {
			headingTraveled = (M_PI/2) - (robotHeading + (M_PI/2) - fabs(atan(localY / localX)));
		} else if (localX > 0 && localY <= 0) {
			headingTraveled = (M_PI/2) - (robotHeading + (M_PI/2) + fabs(atan(localY / localX)));
		} else if (localX <= 0 && localY < 0) {
			headingTraveled = (M_PI/2) - (robotHeading + (3*M_PI/2) - fabs(atan(localY / localX)));
		} else if (localX < 0 && localY >= 0) {
			headingTraveled = (M_PI/2) - (robotHeading + (3*M_PI/2) + fabs(atan(localY / localX)));
		} 
		while (headingTraveled >= 2*M_PI) {
			headingTraveled -= 2*M_PI;
		} 
		while (headingTraveled < 0) {
			headingTraveled += 2*M_PI;
		}

		float globalX = distanceTraveled * cos(headingTraveled);
		float globalY = distanceTraveled * sin(headingTraveled);

		//printf("X position: %f\nY position: %f\n", globalX, globalY);

		if (returnX) {
			return globalX;
		} else {
			return globalY;
		}
	}

	//Converts field-centric coordinates to robot-centric
	float globalToLocalCoords(float globalX, float globalY, float robotX, float robotY, float robotHeading, bool returnX = true) {
		float globalXDist = globalX - robotX;
		float globalYDist = globalY - robotY;
		float straightDist = sqrt(pow(globalXDist, 2) + pow(globalYDist, 2));
		float angleToTarget = 0;

		if (globalXDist >= 0 && globalYDist > 0) {          //Quadrant 1
			angleToTarget = fabs(atan(globalYDist / globalXDist)) + robotHeading;
		} else if (globalXDist > 0 && globalYDist <= 0) {   //Quadrant 4
			//printf("%f\n", fabs(atan(globalYDist / globalXDist)));
			angleToTarget = 2*M_PI - fabs(atan(globalYDist / globalXDist)) + robotHeading;
		} else if (globalXDist <= 0 && globalYDist < 0) {   //Quadrant 3
			angleToTarget = M_PI + fabs(atan(globalYDist / globalXDist)) - robotHeading;
		} else if (globalXDist < 0 && globalYDist >= 0) {   //Quadrant 2
			angleToTarget = M_PI - fabs(atan(globalYDist / globalXDist)) + robotHeading;
		} 

		float localX = straightDist * cos(angleToTarget);
		float localY = straightDist * sin(angleToTarget);
		//printf("%f\n", angleToTarget);

		//printf("localX: %f\nlocalY: %f\n", localX, localY);

		if (returnX) {
			return localX;
		} else {
			return localY;
		}
	}

	void track_position() {
		while (true) {
			left_odom_dist = odom_left.get_position()*100/360 * (2.75 * M_PI);   //distance traveled by left tracking wheel since last poll
			right_odom_dist = odom_left.get_position()*100/360 * (2.75 * M_PI);
			rear_odom_dist = odom_left.get_position()*100/360 * (2.75 * M_PI);

			imu_heading = imu.get_heading();
			float theta = imu_heading - previous_heading;
			//printf("IMU Heading: %f\n", imu_heading);

			float localXDist = 0;
			float localYDist = 0;
			if (theta != 0) {
			localXDist = 2 * (rear_odom_dist / theta + REAR_OFFSET) * (sin(theta / 2));
			localYDist = (left_odom_dist + right_odom_dist) / theta * (sin(theta / 2));
			} else {
			localXDist = rear_odom_dist;
			localYDist = (right_odom_dist + left_odom_dist) / 2;
			}

			trackingX += localToGlobalCoords(localXDist, localYDist, imu_heading, true);
			trackingY += localToGlobalCoords(localXDist, localYDist, imu_heading, false);

			odomX = trackingX + localToGlobalCoords(RIGHT_OFFSET - LEFT_OFFSET, REAR_OFFSET, imu_heading, true);
			odomY = trackingY + localToGlobalCoords(RIGHT_OFFSET - LEFT_OFFSET, REAR_OFFSET, imu_heading, false);

			//printf("X position: %f\nY position: %f\n", odomX, odomY);

			odom_left.reset_position();
			odom_right.reset_position();
			odom_rear.reset_position();
			previous_heading = imu_heading;

			pros::delay(100);
		}
	}

	int get_x() {
		return odomX;
	}
	int get_y() {
		return odomY;
	}
};