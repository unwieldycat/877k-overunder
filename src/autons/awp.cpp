#include "main.h"

void awp() {
	// TODO: Make it do stuff
	// Run push auton
	// Go to climbing bar and tap somehow
	chassis::pursuit("/usd/paths/awp_auton0.csv");
	chassis::pursuit("/usd/paths/awp_auton1p.csv", true);
}