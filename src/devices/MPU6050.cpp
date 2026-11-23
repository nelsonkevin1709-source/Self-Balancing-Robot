#include "devices/MPU6050.h"
#include <math.h>

namespace {
// Register addresses from the MPU-6000/6050 Register Map document
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;   // first of 14 data bytes
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
}  // namespace

MPU6050::MPU6050(HardwareInterface* hw, uint8_t address) : hw_(hw), address_(address) {}

bool MPU6050::isKnownWhoAmI(uint8_t id) {
	// 0x68 genuine MPU6050. The others are compatible chips often sold as "MPU6050".
	return id == 0x68 || id == 0x70 || id == 0x71 || id == 0x72 || id == 0x73 || id == 0x98;
}

uint8_t MPU6050::whoAmI() {
	uint8_t id = 0;
	if (!hw_->i2cReadRegisters(address_, REG_WHO_AM_I, &id, 1)) {
		return 0;
	}
	return id;
}

bool MPU6050::writeRegister(uint8_t reg, uint8_t value) {
	return hw_->i2cWriteRegister(address_, reg, value);
}

bool MPU6050::begin(AccelRange accelRange, GyroRange gyroRange, uint8_t dlpf) {
	ready_ = false;
	if (!isKnownWhoAmI(whoAmI())) {
		return false;
	}
	// Reset, then wake up using the X gyro as clock (more stable than the internal one).
	if (!writeRegister(REG_PWR_MGMT_1, 0x80)) return false;
	hw_->delayMs(100);
	if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;
	hw_->delayMs(10);

	if (dlpf > 6) dlpf = 6;
	const uint8_t accelBits = static_cast<uint8_t>(accelRange);
	const uint8_t gyroBits = static_cast<uint8_t>(gyroRange);
	if (!writeRegister(REG_CONFIG, dlpf)) return false;
	if (!writeRegister(REG_SMPLRT_DIV, 0)) return false;              // 1 kHz sample rate with DLPF on
	if (!writeRegister(REG_GYRO_CONFIG, gyroBits << 3)) return false;  // range lives in bits 4:3
	if (!writeRegister(REG_ACCEL_CONFIG, accelBits << 3)) return false;

	accelLsbPerG_ = 16384.0f / static_cast<float>(1 << accelBits);   // 16384, 8192, 4096, 2048
	gyroLsbPerDps_ = 131.0f / static_cast<float>(1 << gyroBits);     // 131, 65.5, 32.8, 16.4
	ready_ = true;
	return true;
}

bool MPU6050::readRaw(int16_t accel[3], int16_t& temperature, int16_t gyro[3]) {
	uint8_t b[14];
	if (!hw_->i2cReadRegisters(address_, REG_ACCEL_XOUT_H, b, sizeof(b))) {
		return false;
	}
	// Each value is 16 bits, high byte first.
	for (int i = 0; i < 3; i++) {
		accel[i] = static_cast<int16_t>((b[2 * i] << 8) | b[2 * i + 1]);
		gyro[i] = static_cast<int16_t>((b[8 + 2 * i] << 8) | b[8 + 2 * i + 1]);
	}
	temperature = static_cast<int16_t>((b[6] << 8) | b[7]);
	return true;
}

bool MPU6050::read(ImuReading& out) {
	if (!ready_) return false;
	int16_t accel[3], gyro[3], temperature;
	if (!readRaw(accel, temperature, gyro)) {
		return false;
	}
	for (int i = 0; i < 3; i++) {
		out.accelG[i] = accel[i] / accelLsbPerG_;
		out.gyroDps[i] = gyro[i] / gyroLsbPerDps_ - gyroBiasDps_[i];
	}
	out.temperatureC = temperature / 340.0f + 36.53f;   // formula from the register map
	out.timestampUs = hw_->micros();
	return true;
}

bool MPU6050::calibrateGyro(uint16_t samples) {
	if (!ready_ || samples == 0) return false;
	double sum[3] = { 0, 0, 0 };
	double sumSquares[3] = { 0, 0, 0 };
	for (uint16_t n = 0; n < samples; n++) {
		int16_t accel[3], gyro[3], temperature;
		if (!readRaw(accel, temperature, gyro)) {
			return false;
		}
		for (int i = 0; i < 3; i++) {
			const double dps = gyro[i] / gyroLsbPerDps_;
			sum[i] += dps;
			sumSquares[i] += dps * dps;
		}
		hw_->delayMs(2);
	}
	// If the readings jumped around a lot, someone moved the robot: don't trust it.
	for (int i = 0; i < 3; i++) {
		const double mean = sum[i] / samples;
		const double variance = sumSquares[i] / samples - mean * mean;
		if (variance > 4.0) {   // standard deviation above 2 deg/s
			return false;
		}
	}
	for (int i = 0; i < 3; i++) {
		gyroBiasDps_[i] = static_cast<float>(sum[i] / samples);
	}
	return true;
}

void MPU6050::setGyroBias(const float biasDps[3]) {
	for (int i = 0; i < 3; i++) gyroBiasDps_[i] = biasDps[i];
}

void MPU6050::getGyroBias(float biasDps[3]) const {
	for (int i = 0; i < 3; i++) biasDps[i] = gyroBiasDps_[i];
}
