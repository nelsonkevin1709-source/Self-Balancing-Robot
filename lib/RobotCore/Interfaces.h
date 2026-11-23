#pragma once

// ---------------------------------------------------------------------
//  Small "plug shapes" so the balancing code does not care WHICH
//  sensor or motor driver is behind it. Today the Imu class provides
//  TiltSource and the MotorDriver class provides DriveOutput. If you
//  later swap the MPU6050 for another IMU, you only write a new class
//  that provides TiltSource - Balance stays untouched.
// ---------------------------------------------------------------------

// Something that can tell the robot how far it is leaning.
class TiltSource {
 public:
  virtual ~TiltSource() {}
  virtual bool  ok() const = 0;             // true if the latest reading is valid
  virtual float pitchDeg() const = 0;       // lean angle, POSITIVE = leaning forward
  virtual float pitchRateDps() const = 0;   // how fast the lean changes (degrees/second)
};

// Something that can drive the two wheels.
class DriveOutput {
 public:
  virtual ~DriveOutput() {}
  // Power in percent: -100 (full reverse) .. 0 (stop) .. +100 (full forward)
  virtual void setSpeeds(float leftPercent, float rightPercent) = 0;
  virtual void stop() = 0;
};
