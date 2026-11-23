#include "Balance.h"
#include <math.h>

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

bool Balance::begin() {
  goIdle();
  _lastUs = micros();
  return true;
}

void Balance::goIdle() {
  _drive.stop();
  _state = State::Idle;
  _integral = 0;
  _lastOutput = 0;
  _uprightTiming = false;
}

void Balance::setDriveInput(float throttle, float turn) {
  _throttle = clampf(throttle, -1.0f, 1.0f);
  _turn = clampf(turn, -1.0f, 1.0f);
}

void Balance::update() {
  const uint32_t nowUs = micros();
  float dt = (nowUs - _lastUs) * 1e-6f;
  _lastUs = nowUs;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.005f;

  // No trustworthy angle = no driving.
  if (!_tilt.ok()) {
    goIdle();
    return;
  }

  const float angle = _tilt.pitchDeg();        // + = leaning forward
  const float rate  = _tilt.pitchRateDps();    // + = tipping further forward

  if (_state == State::Idle) {
    _drive.stop();
    // Wait until someone holds the robot upright for a moment, then take over.
    if (fabsf(angle - _cfg.trimDeg) < _cfg.startAngleDeg) {
      const uint32_t nowMs = millis();
      if (!_uprightTiming) {
        _uprightTiming = true;
        _uprightSinceMs = nowMs;
      } else if (nowMs - _uprightSinceMs >= _cfg.startHoldMs) {
        _state = State::Balancing;
        _integral = 0;
      }
    } else {
      _uprightTiming = false;
    }
    return;
  }

  // --- Balancing ---
  if (fabsf(angle) > _cfg.fallAngleDeg) {
    goIdle();   // it fell over: stop the wheels
    return;
  }

  // Driving forward means aiming to lean slightly forward.
  const float setpoint = _cfg.trimDeg + _throttle * _cfg.maxLeanDeg;
  const float error = angle - setpoint;   // + = leaning further forward than we want

  // I: the integral looks at the lean relative to TRIM, not relative to the drive setpoint.
  // That way a held "forward" command settles at a steady speed instead of fighting itself.
  // The small leak lets the robot slow back down by itself after you let go.
  _integral += (_cfg.ki * (angle - _cfg.trimDeg) - _cfg.integralLeakPerSec * _integral) * dt;
  _integral = clampf(_integral, -_cfg.integralLimit, _cfg.integralLimit);

  // P: push back in proportion to the lean.  D: use the gyro's lean SPEED directly
  // (cleaner than differentiating the angle).  Leaning forward -> wheels drive forward.
  float out = _cfg.kp * error + _integral + _cfg.kd * rate;
  out = clampf(out, -100.0f, 100.0f);
  _lastOutput = out;

  const float turnMix = _turn * _cfg.turnPercent;   // + = turn right: left wheel faster
  _drive.setSpeeds(out + turnMix, out - turnMix);
}

void Balance::report(Print& out) {
  out.printf("[balance] %s Kp=%.2f Ki=%.2f Kd=%.2f trim=%.2f out=%.1f\n",
             _state == State::Balancing ? "BALANCING" : "idle",
             _cfg.kp, _cfg.ki, _cfg.kd, _cfg.trimDeg, _lastOutput);
}
