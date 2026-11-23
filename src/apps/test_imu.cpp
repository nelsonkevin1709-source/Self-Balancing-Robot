/*****************
test_imu.cpp  -  talk to the MPU6050

What it does:
	1. Finds the chip on I2C and checks its ID
	2. Checks gravity reads ~1 g and the temperature is sensible
	3. Calibrates the gyro (KEEP THE ROBOT STILL for ~2 s after reset)
	4. Prints readings 5 times a second

Then tilt the robot forward and back by hand and watch "pitch" and "rate":
	- if "pitch" changes when you lean it forward, the board is mounted the way we assume
	- if "roll" changes instead, tell me and we'll change the axis in config.h
Leaning FORWARD should make pitch positive. If it goes negative, the chip is facing backwards.
*****************/

#include "apps/App.h"
#include "apps/TestReport.h"
#include "config.h"
#include "devices/MPU6050.h"
#include <math.h>

namespace {
constexpr float kRadToDeg = 57.2957795f;

MPU6050* imu = nullptr;
uint32_t lastPrintMs = 0;
}  // namespace

void appSetup(Platform& p) {
	HardwareInterface& hw = p.hw;
	static MPU6050 sensor(&hw, cfg::kImuAddress);
	imu = &sensor;
	TestReport report(hw, "MPU6050");

	hw.i2cBegin(cfg::kPinSda, cfg::kPinScl, cfg::kI2cHz);
	const uint8_t id = imu->whoAmI();
	report.check("chip answers on I2C", id != 0, "WHO_AM_I = 0x%02X", id);
	if (id == 0) {
		hw.printf("      check wiring: SDA -> GPIO%d, SCL -> GPIO%d, VCC -> 3V3, GND -> GND\n", cfg::kPinSda, cfg::kPinScl);
	}
	if (id != 0 && id != 0x68) {
		hw.printf("      note: 0x%02X is a compatible clone chip, that's OK\n", id);
	}
	if (!report.check("configure (wake up, +/-2 g, +/-500 deg/s)", imu->begin())) {
		report.summary();
		hw.print("Fix the wiring and press the EN/RESET button to try again.\n");
		return;
	}

	ImuReading r;
	const bool readOk = imu->read(r);
	report.check("read data", readOk);
	const float g = sqrtf(r.accelG[0] * r.accelG[0] + r.accelG[1] * r.accelG[1] + r.accelG[2] * r.accelG[2]);
	report.check("gravity is about 1 g", readOk && g > 0.85f && g < 1.15f, "%.3f g", g);
	report.check("temperature is sensible", readOk && r.temperatureC > 0 && r.temperatureC < 60, "%.1f C", r.temperatureC);

	hw.print("      calibrating gyro, keep the robot STILL...\n");
	const bool calibrated = imu->calibrateGyro(500);
	float bias[3];
	imu->getGyroBias(bias);
	report.check("gyro calibration", calibrated, "offset X %.2f  Y %.2f  Z %.2f deg/s%s", bias[0], bias[1], bias[2],
		calibrated ? "" : " (robot moved? reset and try again)");
	if (calibrated) {
		imu->read(r);
		const float still = fabsf(r.gyroDps[0]) + fabsf(r.gyroDps[1]) + fabsf(r.gyroDps[2]);
		report.check("gyro reads ~0 when still", still < 3.0f, "sum %.2f deg/s", still);
	}
	report.summary();
	hw.print("   ax     ay     az  |   gx     gy     gz  | pitch   roll  rate(gy)\n");
}

void appLoop(Platform& p) {
	HardwareInterface& hw = p.hw;
	if (imu == nullptr || !imu->isReady()) {
		hw.delayMs(100);
		return;
	}
	const uint32_t now = hw.millis();
	if (now - lastPrintMs < 200) {
		hw.delayMs(5);
		return;
	}
	lastPrintMs = now;

	ImuReading r;
	if (!imu->read(r)) {
		hw.print("I2C read failed\n");
		return;
	}
	// Tilt from gravity alone (only correct when not accelerating; fine for this test).
	const float pitch = atan2f(-r.accelG[0], r.accelG[2]) * kRadToDeg;
	const float roll = atan2f(r.accelG[1], r.accelG[2]) * kRadToDeg;
	hw.printf("%5.2f  %5.2f  %5.2f | %6.1f %6.1f %6.1f | %6.1f %6.1f %6.1f\n",
		r.accelG[0], r.accelG[1], r.accelG[2], r.gyroDps[0], r.gyroDps[1], r.gyroDps[2], pitch, roll, r.gyroDps[1]);
}

#ifdef SBR_SIM
#include "comms/sim/ScriptedLink.h"
#include "hal/sim/SimHardware.h"

// Simulated world: robot held still for calibration, then rocked by hand +/-20 deg,
// then let go so you can watch it fall over.
void simConfigure(SimHardware& sim, ScriptedLink& phone) {
	(void)phone;
	sim.setDuration(9.0);
	sim.physics().setBodyMode(BodyMode::Held);
	sim.setScenario([&sim](double t) {
		RobotPhysics& robot = sim.physics();
		if (t < 4.0) {
			robot.setHeldAngle(0.0);
		} else if (t < 7.0) {
			robot.setHeldAngle(0.35 * sin(2.0 * 3.14159265 * 0.5 * (t - 4.0)));   // +/-20 deg rocking
		} else if (robot.bodyMode() == BodyMode::Held) {
			sim.print("[sim] letting go of the robot (motors off) - it will fall forward\n");
			robot.setTheta(0.03);
			robot.setBodyMode(BodyMode::Free);
		}
	});
}
#endif
