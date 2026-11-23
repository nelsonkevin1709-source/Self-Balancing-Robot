#pragma once
#include <Arduino.h>
#include <Module.h>
#include <Interfaces.h>

struct BalanceConfig {
  float    kp = 12.0f;             // % power per degree of lean
  float    ki = 40.0f;             // % power per (degree * second) of accumulated lean
  float    kd = 2.0f;              // % power per (degree/second) of lean speed
  float    trimDeg = 0.0f;         // the angle at which the robot actually balances
  float    maxLeanDeg = 2.0f;      // lean used to drive at full forward/back command
  float    turnPercent = 25.0f;    // wheel power difference at full turn command
  float    fallAngleDeg = 30.0f;   // beyond this: motors off
  float    startAngleDeg = 3.0f;   // must be this close to upright to (re)start
  uint32_t startHoldMs = 500;      // ...for this long
  float    integralLimit = 60.0f;  // anti-windup clamp for the Ki term (% power)
  float    integralLeakPerSec = 0.15f;  // how fast the Ki term fades (0 = never). Lets it coast to a stop.
};

// PID balance controller.
// Idle  : motors off. Waits until the robot is held upright for a moment.
// Balancing: keeps the robot upright by driving the wheels under it.
// If the robot falls past fallAngleDeg, it goes back to Idle.
class Balance : public Module {
 public:
  Balance(TiltSource& tilt, DriveOutput& drive, const BalanceConfig& cfg,
          uint32_t updateIntervalUs = 5000)
      : Module(updateIntervalUs), _tilt(tilt), _drive(drive), _cfg(cfg) {}

  const char* name() const override { return "balance"; }
  bool begin() override;
  void update() override;
  void report(Print& out) override;

  // throttle: -1 (backward) .. +1 (forward).  turn: -1 (left) .. +1 (right).
  void setDriveInput(float throttle, float turn);

  BalanceConfig& config() { return _cfg; }   // lets main.cpp change gains live
  bool  isBalancing() const { return _state == State::Balancing; }
  float lastOutput() const { return _lastOutput; }

 private:
  enum class State { Idle, Balancing };
  void goIdle();

  TiltSource&   _tilt;
  DriveOutput&  _drive;
  BalanceConfig _cfg;

  State    _state = State::Idle;
  float    _throttle = 0;
  float    _turn = 0;
  float    _integral = 0;
  float    _lastOutput = 0;
  uint32_t _lastUs = 0;
  bool     _uprightTiming = false;
  uint32_t _uprightSinceMs = 0;
};
