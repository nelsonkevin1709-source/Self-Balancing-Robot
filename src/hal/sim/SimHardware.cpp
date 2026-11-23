#include "hal/sim/SimHardware.h"
#include "config.h"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
constexpr double kPhysicsStepS = 0.00025;    // 0.25 ms
constexpr uint64_t kPhysicsStepUs = 250;
constexpr uint64_t kScenarioPeriodUs = 1000; // scenario runs every 1 ms
constexpr double kSpeedOfSound = 343.0;      // m/s
constexpr double kRadToDeg = 57.29577951308232;

// MPU6050 registers the fake chip understands
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_DATA_START = 0x3B;     // ACCEL_XOUT_H, 14 bytes of data follow
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

void putInt16(uint8_t* registers, uint8_t reg, double value) {
	if (value > 32767) value = 32767;
	if (value < -32768) value = -32768;
	int16_t v = static_cast<int16_t>(std::lround(value));
	registers[reg] = static_cast<uint8_t>((v >> 8) & 0xFF);       // MPU6050 is big-endian: high byte first
	registers[reg + 1] = static_cast<uint8_t>(v & 0xFF);
}
}  // namespace

SimHardware::SimHardware(const RobotParams& params, uint32_t randomSeed)
	: physics_(params), rng_(randomSeed) {
	for (int i = 0; i < kMaxPins; i++) pinChannel_[i] = -1;
	resetImu();
}

// ---------------- time ----------------

void SimHardware::advance(uint64_t us) {
	const uint64_t target = nowUs_ + us;
	while (physicsDoneUs_ + kPhysicsStepUs <= target) {
		physicsDoneUs_ += kPhysicsStepUs;
		nowUs_ = physicsDoneUs_;
		if (scenario_ && nowUs_ >= nextScenarioUs_) {
			nextScenarioUs_ += kScenarioPeriodUs;
			scenario_(seconds());
		}
		updateMotorsFromPins();
		physics_.step(kPhysicsStepS);
	}
	nowUs_ = target;
}

// Reading the clock costs a tiny bit of time, like on a real CPU. This also means a
// "while (millis() < t) {}" loop still finishes in the simulator.
uint32_t SimHardware::millis() { advance(1); return static_cast<uint32_t>(nowUs_ / 1000); }
uint32_t SimHardware::micros() { advance(1); return static_cast<uint32_t>(nowUs_); }
void SimHardware::delayMs(uint32_t ms) { advance(static_cast<uint64_t>(ms) * 1000); }
void SimHardware::delayUs(uint32_t us) { advance(us); }

// ---------------- digital pins ----------------

void SimHardware::pinOutput(int pin) { if (pin >= 0 && pin < kMaxPins) pinIsOutput_[pin] = true; }
void SimHardware::pinInput(int pin) { if (pin >= 0 && pin < kMaxPins) pinIsOutput_[pin] = false; }

void SimHardware::digitalWrite(int pin, bool high) {
	if (pin < 0 || pin >= kMaxPins || !pinIsOutput_[pin]) return;   // like real hardware: pinMode first!
	if (pin == cfg::kPinUltrasonicTrig) {
		if (high && !pinLevel_[pin]) {
			triggerRiseUs_ = nowUs_;
		}
		if (!high && pinLevel_[pin] && nowUs_ - triggerRiseUs_ >= 10) {
			triggerArmed_ = true;     // HC-SR04 needs a >= 10 us trigger pulse
		}
	}
	pinLevel_[pin] = high;
}

bool SimHardware::digitalRead(int pin) {
	if (pin < 0 || pin >= kMaxPins) return false;
	return pinLevel_[pin];
}

uint32_t SimHardware::pulseInHigh(int pin, uint32_t timeoutUs) {
	if (pin != cfg::kPinUltrasonicEcho || !triggerArmed_) {
		advance(timeoutUs);     // nothing will ever pulse: wait out the timeout
		return 0;
	}
	triggerArmed_ = false;
	const double distance = obstacleDistance_ - physics_.x() + 0.002 * gauss_(rng_);
	if (distance < 0.02 || distance > 4.0) {
		advance(timeoutUs);     // too close or too far: no echo
		return 0;
	}
	const double echoUs = 2.0 * distance / kSpeedOfSound * 1e6;
	if (echoUs > timeoutUs) {
		advance(timeoutUs);
		return 0;
	}
	advance(450 + static_cast<uint64_t>(echoUs));   // ~450 us for the sensor to send its burst
	return static_cast<uint32_t>(echoUs);
}

// ---------------- PWM ----------------

void SimHardware::pwmAttach(int pin, int channel, uint32_t freqHz, uint8_t bits) {
	(void)freqHz;
	if (pin < 0 || pin >= kMaxPins || channel < 0 || channel >= kMaxChannels) return;
	pinChannel_[pin] = channel;
	pinIsOutput_[pin] = true;
	channelBits_[channel] = bits;
	channelDuty_[channel] = 0;
}

void SimHardware::pwmWrite(int channel, uint32_t duty) {
	if (channel < 0 || channel >= kMaxChannels) return;
	channelDuty_[channel] = duty;
}

double SimHardware::pinDuty(int pin) const {
	if (pin < 0 || pin >= kMaxPins || !pinIsOutput_[pin]) return 0.0;
	const int ch = pinChannel_[pin];
	if (ch >= 0) {
		const double fullScale = static_cast<double>((1u << channelBits_[ch]) - 1);
		const double duty = channelDuty_[ch] / fullScale;
		return duty > 1.0 ? 1.0 : duty;
	}
	return pinLevel_[pin] ? 1.0 : 0.0;
}

// DRV8833: the average motor voltage is (IN1 level - IN2 level) x supply.
// Both inputs LOW = coast (motor disconnected).
void SimHardware::updateMotorsFromPins() {
	const int pins[2][2] = { { cfg::kPinLeftIn1, cfg::kPinLeftIn2 }, { cfg::kPinRightIn1, cfg::kPinRightIn2 } };
	for (int m = 0; m < 2; m++) {
		const double in1 = pinDuty(pins[m][0]);
		const double in2 = pinDuty(pins[m][1]);
		physics_.setMotorDrive(m, in1 - in2, in1 == 0.0 && in2 == 0.0);
	}
}

// ---------------- I2C / fake MPU6050 ----------------

void SimHardware::resetImu() {
	std::memset(imuRegisters_, 0, sizeof(imuRegisters_));
	imuRegisters_[REG_PWR_MGMT_1] = 0x40;   // the real chip powers up ASLEEP
	imuRegisters_[REG_WHO_AM_I] = 0x68;
}

bool SimHardware::i2cBegin(int sdaPin, int sclPin, uint32_t clockHz) {
	(void)clockHz;
	i2cStarted_ = (sdaPin == cfg::kPinSda && sclPin == cfg::kPinScl);   // wrong pins = no device found
	return true;
}

bool SimHardware::i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value) {
	advance(70);   // 3 bytes at 400 kHz
	if (!i2cStarted_ || address != cfg::kImuAddress || reg >= sizeof(imuRegisters_)) return false;
	if (reg == REG_PWR_MGMT_1 && (value & 0x80)) {
		resetImu();    // DEVICE_RESET bit
		return true;
	}
	if (reg == REG_WHO_AM_I) return true;   // read-only
	imuRegisters_[reg] = value;
	return true;
}

bool SimHardware::i2cReadRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, size_t length) {
	advance(50 + 23 * length);   // each byte takes ~23 us at 400 kHz
	if (!i2cStarted_ || address != cfg::kImuAddress || reg + length > sizeof(imuRegisters_)) return false;
	fillImuRegisters();
	std::memcpy(buffer, &imuRegisters_[reg], length);
	return true;
}

// Work out what a real MPU6050 on the robot would measure right now.
void SimHardware::fillImuRegisters() {
	const bool asleep = (imuRegisters_[REG_PWR_MGMT_1] & 0x40) != 0;
	if (asleep) {
		for (uint8_t r = REG_DATA_START; r < REG_DATA_START + 14; r++) imuRegisters_[r] = 0;
		return;
	}
	const RobotParams& p = physics_.params();
	const double th = physics_.theta();
	const double thd = physics_.thetaDot();
	const double thdd = physics_.thetaDDot();
	const double h = p.imuHeight;
	const double g = p.gravity;

	// Acceleration of the sensor in the world (x forward, z up), minus gravity
	// (an accelerometer feels +1 g upwards when sitting still).
	const double ax = physics_.xDDot() + h * (thdd * std::cos(th) - thd * thd * std::sin(th));
	const double az = -h * (thdd * std::sin(th) + thd * thd * std::cos(th)) + g;
	// Rotate into the sensor's own axes (X forward, Y left, Z up when upright).
	const double sensorX = ax * std::cos(th) - az * std::sin(th);
	const double sensorZ = ax * std::sin(th) + az * std::cos(th);

	// Motors shake the chassis: more noise when they work hard.
	const double vibration = 0.01 + 0.04 * std::fabs(physics_.wheelSpeed(0) / p.motorNoLoadSpeed);
	const double accelG[3] = {
		sensorX / g + accelBiasG_[0] + vibration * gauss_(rng_),
		0.0 + accelBiasG_[1] + vibration * gauss_(rng_),
		sensorZ / g + accelBiasG_[2] + vibration * gauss_(rng_),
	};
	const double gyroDps[3] = {
		gyroBiasDps_[0] + 0.05 * gauss_(rng_),
		thd * kRadToDeg + gyroBiasDps_[1] + 0.05 * gauss_(rng_),   // leaning forward = +Y rotation
		gyroBiasDps_[2] + 0.05 * gauss_(rng_),
	};

	const int accelRange = (imuRegisters_[REG_ACCEL_CONFIG] >> 3) & 0x03;
	const int gyroRange = (imuRegisters_[REG_GYRO_CONFIG] >> 3) & 0x03;
	const double accelLsbPerG = 16384.0 / (1 << accelRange);
	const double gyroLsbPerDps = 131.0 / (1 << gyroRange);
	const double temperatureC = 28.0;

	for (int i = 0; i < 3; i++) putInt16(imuRegisters_, REG_DATA_START + 2 * i, accelG[i] * accelLsbPerG);
	putInt16(imuRegisters_, REG_DATA_START + 6, (temperatureC - 36.53) * 340.0);
	for (int i = 0; i < 3; i++) putInt16(imuRegisters_, REG_DATA_START + 8 + 2 * i, gyroDps[i] * gyroLsbPerDps);
}

// ---------------- text I/O ----------------

void SimHardware::print(const char* text) { std::fputs(text, stdout); }

int SimHardware::readChar() {
	if (keyboard_.empty()) return -1;
	char c = keyboard_.front();
	keyboard_.pop_front();
	return static_cast<unsigned char>(c);
}

void SimHardware::typeText(const char* text) {
	for (const char* c = text; *c; c++) keyboard_.push_back(*c);
}

// Expectation codes: "L+" left wheel forward, "R-" right wheel backward, "L+R+" both forward.
// A wheel not mentioned must be (nearly) still.
int SimHardware::observe(const char* expectation) {
	const double kSpinning = 3.0;   // rad/s
	int expected[2] = { 0, 0 };
	for (const char* c = expectation; c[0] && c[1]; c += 2) {
		const int wheel = (c[0] == 'L') ? 0 : (c[0] == 'R') ? 1 : -1;
		if (wheel >= 0) expected[wheel] = (c[1] == '+') ? 1 : -1;
	}
	for (int m = 0; m < 2; m++) {
		const double w = physics_.wheelSpeed(m);
		const int actual = (w > kSpinning) ? 1 : (w < -kSpinning) ? -1 : 0;
		if (actual != expected[m]) return 0;
	}
	return 1;
}
