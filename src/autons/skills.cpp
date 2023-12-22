#include "chassis/pursuit.hpp"
#include "main.h"

const int launches = 50;

void skills() {
	chassis::drive(90);
	pros::delay(750);
	chassis::drive(0);
	chassis::turn_rel(-135_deg);
	chassis::drive(90);
	pros::delay(750);
	chassis::drive(32);

	while (true) {
		cata::prime();
		pros::delay(1000);
		cata::release();
	}

	/*
	// Drive to match loader
	chassis::drive(2_ft);
	chassis::turn_rel(-135_deg);
	chassis::drive(2_ft);
	chassis::drive(64);

	// Match load
	// TODO: Display countdown on screen & change LED colors

	cata_optical.set_led_pwm(100);
	int end_time = pros::millis() + 30000;

	for (int i = 0; i < launches; i++) {
	    cata::prime();
	    cata::release();
	    pros::delay(20);
	}

	// Prepare for pursuit
	chassis::drive(1_ft);
	chassis::turn_rel(-45_deg);
	chassis::drive(1.41_ft);
	chassis::turn_rel(45_deg);
	*/

	std::vector<double> x_coords = {
	    2,      2.5871, 3.1687, 3.7433, 4.3090, 4.8644, 5.4078, 5.9375, 6.4519, 6.9493, 7.4282,
	    7.8868, 8.3236, 8.7369, 9.125,  9.4863, 9.8192, 10.122, 10.393, 10.631, 10.834, 11,
	    11.128, 11.216, 11.262, 11.266, 11.224, 11.136, 11,     10.734, 10.313, 9.7344, 9};
	std::vector<double> y_coords = {
	    2,      2.9585, 3.7034, 4.2533, 4.6268, 4.8426, 4.9191, 4.875,  4.7289, 4.4993, 4.2048,
	    3.8641, 3.4956, 3.1181, 2.75,   2.41,   2.1166, 1.8885, 1.7442, 1.7023, 1.7813, 2,
	    2.3768, 2.9304, 3.6793, 4.6421, 5.8375, 7.2839, 9,      9.6562, 10.208, 10.656, 11};
	std::vector<double> kappas = {
	    0.0749, 0.1166, 0.182,  0.2767, 0.3911, 0.4861, 0.513,  0.4661, 0.3838, 0.3026, 0.2375,
	    0.1896, 0.1559, 0.1329, 0.1185, 0.1121, 0.1175, 0.1541, 0.3026, 0.7044, 0.7998, 0.4072,
	    0.1891, 0.0998, 0.0595, 0.0387, 0.0269, 0.0196, 0.2857, 0.3006, 0.2656, 0.2048, 0.1494};
	for (int i = 0; i < x_coords.size(); i++) {
		chassis::pursuit::add_point((foot_t)(x_coords[i]), (foot_t)(y_coords[i]), kappas[i]);
	}
	chassis::pursuit::pursuit(true);
	x_coords = {9,      9.352,  9.6248, 9.8236, 9.9538, 10.02,  10.029, 9.9841, 9.8917, 9.7566,
	            9.5842, 9.3796, 9.1481, 8.895,  8.6254, 8.3445, 8.0576, 7.77,   7.4868, 7.2132,
	            6.9546, 6.7167, 6.5029, 6.3202, 6.1734, 6.0676, 6.0081, 6};
	y_coords = {11,     10.735, 10.452, 10.157, 9.8515, 9.5416, 9.2309, 8.9238, 8.6244, 8.3369,
	            8.0655, 7.8145, 7.588,  7.3903, 7.2256, 7.098,  7.0119, 6.9713, 6.9806, 7.0439,
	            7.1654, 7.3493, 7.5999, 7.9214, 8.3179, 8.7937, 9.353,  10};
	kappas = {0.3091, 0.4215, 0.5549, 0.694,  0.8172, 0.905,  0.9636, 1.0778, 1.3615, 1.6734,
	          1.7021, 1.4092, 1.0105, 0.6714, 0.4327, 0.2772, 0.1789, 0.117,  0.0781, 0.0536,
	          0.0382, 0.0286, 0.0225, 0.0187, 0.0162, 0.0145, 0.0131, 0.2249};
	for (int i = 0; i < x_coords.size(); i++) {
		chassis::pursuit::add_point((foot_t)(x_coords[i]), (foot_t)(y_coords[i]), kappas[i]);
	}
	chassis::pursuit::pursuit();
}
