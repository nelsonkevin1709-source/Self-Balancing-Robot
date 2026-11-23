/*****************
MPU6050.h
Self-Balancing Robot

Driver for the MPU6050 accelerometer + gyroscope (I2C).
Written from the datasheet / register map, no external library, so you can see
exactly what is sent to the chip.

Like Galil in MTRN3500, it does NOT touch hardware itself: every I2C transfer
goes through the HardwareInterface pointer given to the constructor, so the
same class works on the robot and in the simulator.

Typical use:
	MPU6050 imu(&hw);
	imu.begin();               // wake up + configure
	imu.calibrateGyro();       // keep the robot STILL for ~1 s
	ImuReading r;
	if (imu.read(r)) { ... r.accelG[0] ... r.gyroDps[1] ... }

Axes (printed on the board): X, Y, Z. Accel in g (1 g = 9.81 m/s^2), gyro in deg/s.
*****************/

#pragma once
#include "hal/HardwareInterface.h"
#include <stdint.h>

struct ImuReading {
	float accelG[3] = { 0, 0, 0 };    // X, Y, Z acceleration [g]
	float gyroDps[3] = { 0, 0, 0 };   // X, Y, Z rotation rate [deg/s], calibration offset removed
	float temperatureC = 0;           // chip temperature [deg C]
	uint32_t timestampUs = 0;         // when the reading was taken
};

class MPU6050 {
public:
	// Full-scale ranges. Smaller range = finer resolution. The value is what goes in the register.
	enum class AccelRange : uint8_t { G2 = 0, G4 = 1, G8 = 2, G16 = 3 };
	enum class GyroRange : uint8_t { Dps250 = 0, Dps500 = 1, Dps1000 = 2, Dps2000 = 3 };

	MPU6050(HardwareInterface* hw, uint8_t address = 0x68);
	MPU6050(const MPU6050&) = delete;               // one object per physical chip
	MPU6050& operator=(const MPU6050&) = delete;

	// Wakes the chip and sets ranges + low-pass filter. Returns false if no MPU6050 answers.
	// dlpf: 0 (off, 260 Hz) .. 6 (5 Hz). 3 = 44 Hz, good for balancing.
	bool begin(AccelRange accelRange = AccelRange::G2, GyroRange gyroRange = GyroRange::Dps500, uint8_t dlpf = 3);

	uint8_t whoAmI();                 // 0x68 for a genuine chip (clones may say 0x70, 0x72, 0x98...)
	static bool isKnownWhoAmI(uint8_t id);

	bool read(ImuReading& out);       // one reading of all 7 values. false = I2C failed

	// Averages 'samples' readings while the robot is still, and stores the gyro offset
	// so later readings are ~0 deg/s at rest. Returns false if I2C failed or the robot moved.
	bool calibrateGyro(uint16_t samples = 500);
	void setGyroBias(const float biasDps[3]);
	void getGyroBias(float biasDps[3]) const;

	bool isReady() const { return ready_; }
	float accelLsbPerG() const { return accelLsbPerG_; }
	float gyroLsbPerDps() const { return gyroLsbPerDps_; }

private:
	bool writeRegister(uint8_t reg, uint8_t value);
	bool readRaw(int16_t accel[3], int16_t& temperature, int16_t gyro[3]);

	HardwareInterface* hw_;
	uint8_t address_;
	bool ready_ = false;
	float accelLsbPerG_ = 16384.0f;
	float gyroLsbPerDps_ = 65.5f;
	float gyroBiasDps_[3] = { 0, 0, 0 };
};
