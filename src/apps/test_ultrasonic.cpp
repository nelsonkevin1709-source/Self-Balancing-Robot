/*****************
test_ultrasonic.cpp  -  talk to the HC-SR04

Wiring: VCC -> 5 V (the ESP32 board's 5V/VIN pin when on USB), GND -> GND,
TRIG -> GPIO27, ECHO -> 1 kOhm -> GPIO35, and 2 kOhm from GPIO35 to GND.

What it does:
	1. Checks an echo comes back
	2. Prints the distance 10 times a second with a bar graph
Move your hand in front of the sensor: the number should follow it (2 - 400 cm).
*****************/

#include "apps/App.h"
#include "apps/TestReport.h"
#include "config.h"
#include "devices/HCSR04.h"

namespace {
HCSR04* sonar = nullptr;

void printBar(HardwareInterface& hw, float cm) {
	char bar[41];
	int n = cm < 0 ? 0 : static_cast<int>(cm / 5.0f);   // one # per 5 cm
	if (n > 40) n = 40;
	for (int i = 0; i < n; i++) bar[i] = '#';
	bar[n] = '\0';
	if (cm < 0) hw.print("   no echo\n");
	else        hw.printf("%7.1f cm  %s\n", cm, bar);
}
}  // namespace

void appSetup(Platform& p) {
	HardwareInterface& hw = p.hw;
	static HCSR04 sensor(&hw, cfg::kPinUltrasonicTrig, cfg::kPinUltrasonicEcho, cfg::kUltrasonicTimeoutUs);
	sonar = &sensor;
	sonar->begin();

	TestReport report(hw, "HC-SR04");
	hw.print("      point the sensor at something 10 - 200 cm away\n");
	int echoes = 0;
	float firstCm = HCSR04::kNoEcho;
	for (int i = 0; i < 10; i++) {
		const float cm = sonar->readDistanceCm();
		if (cm > 0) {
			echoes++;
			if (firstCm < 0) firstCm = cm;
		}
		hw.delayMs(HCSR04::kMinIntervalMs);
	}
	report.check("echo received", echoes > 0, "%d of 10 pings", echoes);
	if (echoes == 0) {
		hw.printf("      check wiring: VCC -> 5V, TRIG -> GPIO%d, ECHO -> divider -> GPIO%d\n",
			cfg::kPinUltrasonicTrig, cfg::kPinUltrasonicEcho);
	}
	report.check("distance in sensor range (2-400 cm)", firstCm >= 2.0f && firstCm <= 400.0f, "%.1f cm", firstCm);
	report.summary();
}

void appLoop(Platform& p) {
	if (sonar == nullptr) return;
	printBar(p.hw, sonar->readDistanceCm());
	p.hw.delayMs(100);
}

#ifdef SBR_SIM
#include "comms/sim/ScriptedLink.h"
#include "hal/sim/SimHardware.h"
#include <cmath>

// Simulated world: a hand moves between 10 cm and 150 cm in front of the robot.
void simConfigure(SimHardware& sim, ScriptedLink& phone) {
	(void)phone;
	sim.setDuration(7.0);
	sim.physics().setBodyMode(BodyMode::Held);
	sim.setObstacleDistance(0.5);
	sim.setScenario([&sim](double t) {
		const double hand = (t < 2.0) ? 0.5 : 0.8 + 0.7 * std::sin(2.0 * 3.14159265 * 0.25 * (t - 2.0));
		sim.setObstacleDistance(hand);
	});
}
#endif
