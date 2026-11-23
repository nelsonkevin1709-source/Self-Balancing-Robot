/*****************
sim_main.cpp
Self-Balancing Robot (simulator only)

Creates the SIMULATED hardware and runs the app CMake built it with, for as
long as the app's scenario asks. Time is virtual, so it finishes quickly.
*****************/

#include "apps/App.h"
#include "comms/sim/ScriptedLink.h"
#include "hal/sim/SimHardware.h"
#include <cstdio>

int main() {
	// static, like the globals in esp32_main.cpp: they must outlive the device objects
	// the apps create (those are static too, and switch the motors off when destroyed).
	static SimHardware sim;
	static ScriptedLink phone(sim);
	static Platform platform{ sim, phone };

	simConfigure(sim, phone);
	std::printf("[sim] running for %.1f simulated seconds\n", sim.duration());

	appSetup(platform);
	while (sim.seconds() < sim.duration()) {
		appLoop(platform);
		sim.advance(50);   // make sure time always moves forward
	}

	std::printf("[sim] finished at t = %.2f s\n", sim.seconds());
	return 0;
}
