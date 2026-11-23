#pragma once
#include <Arduino.h>
#include <Module.h>
#include <Interfaces.h>

struct ImuConfig {
  int      sdaPin         = 21;
  int      sclPin         = 22;
  uint32_t i2cHz          = 400000;
  uint8_t  address        = 0x68;     // 0x68 normally (0x69 if the AD0 pin is tied to 3V3)
  uint8_t  forwardAxis    = 1;        // sensor axis that points the way the robot drives: 1 = X, 2 = Y
  int8_t   angleSign      = 1;        // flip to -1 if "leaning forward" reads negative
  float    filterAlpha    = 0.98f;    // complementary filter: gyro weight (rest is accelerometer)
  uint16_t gyroCalSamples = 500;      // samples averaged at start-up to remove gyro drift
};

// Reads the MPU6050 and works out the robot's lean angle by blending the
// gyroscope (smooth but drifts) with the accelerometer (noisy but never drifts).
class Imu : public Module, public TiltSource {
 public:
  explicit Imu(const ImuConfig& cfg, uint32_t updateIntervalUs = 5000);

  const char* name() const override { return "imu"; }
  bool begin() override;
  void update() override;
  void report(Print& out) override;

  // TiltSource
  bool  ok() const override { return _ok; }
  float pitchDeg() const override { return _pitchDeg; }
  float pitchRateDps() const override { return _rateDps; }

  // Measures the gyro's resting offset. The robot must be held STILL.
  bool calibrateGyro();

 private:
  struct Raw { int16_t ax, ay, az, gx, gy, gz; };

  bool writeReg(uint8_t reg, uint8_t value);
  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
  bool readRaw(Raw& r);
  void computeAngles(const Raw& r, float& accelDeg, float& gyroRateDps) const;

  ImuConfig _cfg;
  bool      _ok = false;
  bool      _seeded = false;
  float     _pitchDeg = 0;
  float     _rateDps = 0;
  float     _gyroBiasX = 0;   // raw units
  float     _gyroBiasY = 0;
  uint32_t  _lastUs = 0;
};
