/*****************
SimHardware.h
Self-Balancing Robot (simulator only)

The SIMULATED implementation of HardwareInterface (like EmbeddedFunctions(true)
in MTRN3500). Instead of real pins it has:
  - a virtual clock: time only moves when the code waits or reads the clock,
    so a 10 second test finishes in a fraction of a second on your PC
  - a fake MPU6050 that answers I2C register reads using the physics model
  - a fake DRV8833: PWM duty on the motor pins becomes motor voltage
  - a fake HC-SR04: an echo pulse whose length matches the distance to a wall
  - "keyboard" input you can script, and an observer that answers yes/no questions

The device classes cannot tell the difference: they only see HardwareInterface.
*****************/

#pragma once
#include "hal/HardwareInterface.h"
#include "hal/sim/RobotPhysics.h"
#include <deque>
#include <functional>
#include <random>

class SimHardware : public HardwareInterface {
public:
	explicit SimHardware(const RobotParams& params = RobotParams(), uint32_t randomSeed = 1);

	// ---------------- HardwareInterface ----------------
	uint32_t millis() override;
	uint32_t micros() override;
	void delayMs(uint32_t ms) override;
	void delayUs(uint32_t us) override;

	void pinOutput(int pin) override;
	void pinInput(int pin) override;
	void digitalWrite(int pin, bool high) override;
	bool digitalRead(int pin) override;
	uint32_t pulseInHigh(int pin, uint32_t timeoutUs) override;

	void pwmAttach(int pin, int channel, uint32_t freqHz, uint8_t bits) override;
	void pwmWrite(int channel, uint32_t duty) override;

	bool i2cBegin(int sdaPin, int sclPin, uint32_t clockHz) override;
	bool i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value) override;
	bool i2cReadRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, size_t length) override;

	void print(const char* text) override;
	int readChar() override;
	int observe(const char* expectation) override;

	// ---------------- simulator controls (used by the test scenarios) ----------------
	RobotPhysics& physics() { return physics_; }
	double seconds() const { return nowUs_ * 1e-6; }
	void advance(uint64_t us);                          // move virtual time forward

	// Called every 1 ms of simulated time, with the time in seconds. This is where a
	// scenario moves the "hand", the wall, etc.
	void setScenario(std::function<void(double)> scenario) { scenario_ = scenario; }
	void setDuration(double seconds) { duration_ = seconds; }
	double duration() const { return duration_; }

	void setObstacleDistance(double metres) { obstacleDistance_ = metres; }  // wall ahead of the start point
	void typeText(const char* text);                                         // pretend the user typed this

private:
	double pinDuty(int pin) const;      // 0..1 output level of a pin (PWM or digital)
	void updateMotorsFromPins();
	void fillImuRegisters();
	void resetImu();

	RobotPhysics physics_;
	std::function<void(double)> scenario_;
	double duration_ = 10.0;

	uint64_t nowUs_ = 0;
	uint64_t physicsDoneUs_ = 0;
	uint64_t nextScenarioUs_ = 0;

	// pins
	static constexpr int kMaxPins = 40;
	static constexpr int kMaxChannels = 16;
	bool pinIsOutput_[kMaxPins] = {};
	bool pinLevel_[kMaxPins] = {};
	int pinChannel_[kMaxPins];
	uint32_t channelDuty_[kMaxChannels] = {};
	uint8_t channelBits_[kMaxChannels] = {};

	// ultrasonic
	double obstacleDistance_ = 1.0;
	uint64_t triggerRiseUs_ = 0;
	bool triggerArmed_ = false;

	// MPU6050
	bool i2cStarted_ = false;
	uint8_t imuRegisters_[128] = {};
	double gyroBiasDps_[3] = { 1.2, -0.8, 0.5 };    // every real gyro has a small offset
	double accelBiasG_[3] = { 0.01, -0.005, 0.02 };

	std::deque<char> keyboard_;
	std::mt19937 rng_;
	std::normal_distribution<double> gauss_{ 0.0, 1.0 };
};
