/*****************
test_motors.cpp  -  talk to the DRV8833 and both motors

!!! LIFT THE ROBOT so the wheels are in the air before resetting !!!
Power the motors from the battery (DRV8833 VM), and connect GND to the ESP32 GND.

What it does:
	1. Spins each wheel forward then backward at half power and asks you (y/n in the
	   serial monitor) whether it went the right way. "Forward" = the way the robot
	   would drive forward.
	2. Spins both wheels forward together.
	3. Slowly ramps both wheels up: note the % where they START to turn
	   (that's the "deadband" we'll need for balancing).

If a wheel goes the wrong way: flip kLeftMotorInverted / kRightMotorInverted in config.h.
If a wheel doesn't move at all: check IN1/IN2 wiring, VM power, and that nSLEEP is at 3.3 V.
*****************/

#include "apps/App.h"
#include "apps/TestReport.h"
#include "config.h"
#include "devices/DcMotor.h"

namespace {
void spinAndAsk(TestReport& report, DcMotor& motor, float command, const char* label, const char* question, const char* expectation) {
	motor.setOutput(command);
	report.askYesNo(label, question, expectation);
	motor.coast();
}
}  // namespace

void appSetup(Platform& p) {
	HardwareInterface& hw = p.hw;
	static DcMotor left(&hw, cfg::kPinLeftIn1, cfg::kPinLeftIn2, cfg::kPwmChLeft1, cfg::kPwmChLeft2, cfg::kLeftMotorInverted);
	static DcMotor right(&hw, cfg::kPinRightIn1, cfg::kPinRightIn2, cfg::kPwmChRight1, cfg::kPwmChRight2, cfg::kRightMotorInverted);
	left.begin(cfg::kPwmFreqHz, cfg::kPwmBits);
	right.begin(cfg::kPwmFreqHz, cfg::kPwmBits);
	left.setMaxOutput(cfg::kMotorMaxOutput);
	right.setMaxOutput(cfg::kMotorMaxOutput);

	TestReport report(hw, "MOTORS");
	hw.print("      LIFT THE WHEELS OFF THE TABLE. Starting in 3 s...\n");
	hw.delayMs(3000);

	spinAndAsk(report, left, 0.5f, "left forward", "Is the LEFT wheel spinning FORWARD?", "L+");
	spinAndAsk(report, left, -0.5f, "left backward", "Is the LEFT wheel spinning BACKWARD?", "L-");
	spinAndAsk(report, right, 0.5f, "right forward", "Is the RIGHT wheel spinning FORWARD?", "R+");
	spinAndAsk(report, right, -0.5f, "right backward", "Is the RIGHT wheel spinning BACKWARD?", "R-");

	left.setOutput(0.5f);
	right.setOutput(0.5f);
	report.askYesNo("both forward", "Are BOTH wheels spinning FORWARD?", "L+R+");
	left.coast();
	right.coast();
	hw.delayMs(500);

	hw.print("      Ramp test: watch the wheels and note the % where they start turning\n");
	for (int percent = 0; percent <= 100; percent += 5) {
		left.setOutput(percent / 100.0f);
		right.setOutput(percent / 100.0f);
		hw.printf("      power %3d %%\n", percent);
		hw.delayMs(400);
	}
	left.brake();
	right.brake();
	hw.delayMs(300);
	left.coast();
	right.coast();

	report.summary();
	hw.print("      done. Press the EN/RESET button to run again.\n");
}

void appLoop(Platform& p) {
	p.hw.delayMs(100);   // nothing more to do
}

#ifdef SBR_SIM
#include "comms/sim/ScriptedLink.h"
#include "hal/sim/SimHardware.h"

// Simulated world: robot held upright with its wheels in the air.
// Every time the motor power changes, it prints how fast the simulated wheels turn.
void simConfigure(SimHardware& sim, ScriptedLink& phone) {
	(void)phone;
	sim.setDuration(19.0);
	sim.physics().setBodyMode(BodyMode::Held);
	sim.physics().setWheelsOnGround(false);
	sim.setScenario([&sim](double t) {
		static double lastDrive[2] = { 0, 0 };
		static double changedAt = -1;
		RobotPhysics& robot = sim.physics();
		if (robot.motorDrive(0) != lastDrive[0] || robot.motorDrive(1) != lastDrive[1]) {
			lastDrive[0] = robot.motorDrive(0);
			lastDrive[1] = robot.motorDrive(1);
			changedAt = t;
		}
		if (changedAt >= 0 && t - changedAt >= 0.3) {   // let the wheels speed up first
			changedAt = -1;
			const double toRpm = 60.0 / (2.0 * 3.14159265);
			sim.printf("      [sim] wheel speed  L %6.1f rpm   R %6.1f rpm\n",
				robot.wheelSpeed(0) * toRpm, robot.wheelSpeed(1) * toRpm);
		}
	});
}
#endif
