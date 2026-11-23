#include "Imu.h"
#include <Wire.h>
#include <math.h>

namespace {
// MPU6050 register addresses (from the datasheet's register map)
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

constexpr float GYRO_LSB_PER_DPS = 65.5f;      // gyro range set to +-500 deg/s
constexpr float RADIANS_TO_DEGREES = 57.2957795f;
}  // namespace

Imu::Imu(const ImuConfig& cfg, uint32_t updateIntervalUs)
    : Module(updateIntervalUs), _cfg(cfg) {}

bool Imu::writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_cfg.address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Imu::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(_cfg.address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(_cfg.address, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool Imu::readRaw(Raw& r) {
  uint8_t b[14];  // accel XYZ, temperature, gyro XYZ - each a 16-bit number
  if (!readRegs(REG_ACCEL_XOUT_H, b, 14)) return false;
  r.ax = (int16_t)((b[0] << 8) | b[1]);
  r.ay = (int16_t)((b[2] << 8) | b[3]);
  r.az = (int16_t)((b[4] << 8) | b[5]);
  // b[6], b[7] = temperature (not used)
  r.gx = (int16_t)((b[8] << 8) | b[9]);
  r.gy = (int16_t)((b[10] << 8) | b[11]);
  // gz (b[12], b[13]) is the spin around the vertical axis - not needed for balancing yet
  r.gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

// Turns raw sensor numbers into "lean angle" (degrees, + = leaning forward)
// and "lean speed" (degrees/second). Both use the same sign convention, which
// is what lets the filter blend them correctly.
void Imu::computeAngles(const Raw& r, float& accelDeg, float& gyroRateDps) const {
  if (_cfg.forwardAxis == 2) {
    accelDeg    = -atan2f((float)r.ay, (float)r.az) * RADIANS_TO_DEGREES;
    gyroRateDps = -((float)r.gx - _gyroBiasX) / GYRO_LSB_PER_DPS;
  } else {
    accelDeg    = -atan2f((float)r.ax, (float)r.az) * RADIANS_TO_DEGREES;
    gyroRateDps = ((float)r.gy - _gyroBiasY) / GYRO_LSB_PER_DPS;
  }
  accelDeg    *= _cfg.angleSign;
  gyroRateDps *= _cfg.angleSign;
}

bool Imu::calibrateGyro() {
  Serial.println("[imu] calibrating gyro - keep the robot STILL...");
  long sumX = 0, sumY = 0;
  uint16_t good = 0;
  _gyroBiasX = 0;
  _gyroBiasY = 0;
  for (uint16_t i = 0; i < _cfg.gyroCalSamples; i++) {
    Raw r;
    if (readRaw(r)) {
      sumX += r.gx;
      sumY += r.gy;
      good++;
    }
    delay(2);
  }
  if (good < _cfg.gyroCalSamples / 2) {
    Serial.println("[imu] calibration failed - too many I2C read errors");
    return false;
  }
  _gyroBiasX = (float)sumX / good;
  _gyroBiasY = (float)sumY / good;
  Serial.printf("[imu] gyro offsets: x=%.1f y=%.1f (raw units)\n", _gyroBiasX, _gyroBiasY);
  return true;
}

bool Imu::begin() {
  Wire.begin(_cfg.sdaPin, _cfg.sclPin);
  Wire.setClock(_cfg.i2cHz);

  uint8_t who = 0;
  if (!readRegs(REG_WHO_AM_I, &who, 1)) {
    Serial.println("[imu] MPU6050 not found on I2C. Check the wiring: VCC->3V3, GND->GND, SDA, SCL.");
    return false;
  }
  if (who != 0x68) {
    Serial.printf("[imu] WHO_AM_I = 0x%02X (expected 0x68). Probably a clone chip - continuing.\n", who);
  }

  bool ok = true;
  ok &= writeReg(REG_PWR_MGMT_1, 0x01);    // wake up, use the gyro as the clock
  delay(10);
  ok &= writeReg(REG_SMPLRT_DIV, 0x00);    // sample at 1 kHz (we read the newest value)
  ok &= writeReg(REG_CONFIG, 0x02);        // low-pass filter ~98 Hz to calm vibration
  ok &= writeReg(REG_GYRO_CONFIG, 0x08);   // gyro range +-500 deg/s
  ok &= writeReg(REG_ACCEL_CONFIG, 0x00);  // accelerometer range +-2 g
  if (!ok) {
    Serial.println("[imu] failed to write settings to the MPU6050");
    return false;
  }
  delay(50);

  if (!calibrateGyro()) return false;

  // Start the angle from what the accelerometer says right now.
  Raw r;
  if (!readRaw(r)) return false;
  float rate;
  computeAngles(r, _pitchDeg, rate);
  _rateDps = 0;
  _seeded = true;
  _lastUs = micros();
  _ok = true;
  return true;
}

void Imu::update() {
  Raw r;
  if (!readRaw(r)) {
    _ok = false;  // Balance sees this and switches the motors off
    return;
  }
  _ok = true;

  const uint32_t now = micros();
  float dt = (now - _lastUs) * 1e-6f;
  _lastUs = now;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.005f;  // first run or a stall: assume the normal step

  float accelDeg, rateDps;
  computeAngles(r, accelDeg, rateDps);

  // Complementary filter: trust the gyro for quick changes, the accelerometer for the long run.
  _pitchDeg = _cfg.filterAlpha * (_pitchDeg + rateDps * dt) + (1.0f - _cfg.filterAlpha) * accelDeg;
  _rateDps = rateDps;
}

void Imu::report(Print& out) {
  out.printf("[imu] %s pitch=%.2f deg rate=%.1f dps\n", _ok ? "ok" : "ERROR", _pitchDeg, _rateDps);
}
