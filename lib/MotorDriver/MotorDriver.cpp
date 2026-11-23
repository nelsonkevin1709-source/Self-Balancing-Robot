#include "MotorDriver.h"

namespace {
constexpr uint8_t  PWM_BITS = 10;                       // power resolution: 0..1023
constexpr uint32_t MAX_DUTY = (1u << PWM_BITS) - 1;
}  // namespace

// The ESP32 Arduino core changed its PWM ("LEDC") functions between version 2 and 3.
// The #if blocks below make this file work with either one.

bool MotorDriver::begin() {
  _pins[0] = _cfg.leftIn1;
  _pins[1] = _cfg.leftIn2;
  _pins[2] = _cfg.rightIn1;
  _pins[3] = _cfg.rightIn2;

  if (_cfg.sleepPin >= 0) {
    pinMode(_cfg.sleepPin, OUTPUT);
    digitalWrite(_cfg.sleepPin, HIGH);   // HIGH = driver awake
  }

  for (int i = 0; i < 4; i++) {
    pinMode(_pins[i], OUTPUT);
    digitalWrite(_pins[i], LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(_pins[i], _cfg.pwmHz, PWM_BITS);
#else
    ledcSetup(i, _cfg.pwmHz, PWM_BITS);   // channel i
    ledcAttachPin(_pins[i], i);
#endif
    writePwm(i, 0);
  }
  _ready = true;
  return true;
}

void MotorDriver::writePwm(int index, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(_pins[index], duty);
#else
  ledcWrite(index, duty);
#endif
}

void MotorDriver::setSpeeds(float leftPercent, float rightPercent) {
  setMotor(0, leftPercent);
  setMotor(1, rightPercent);
}

void MotorDriver::setMotor(int side, float percent) {
  if (!_ready) return;
  _lastPercent[side] = percent;

  const bool invert = (side == 0) ? _cfg.invertLeft : _cfg.invertRight;
  if (invert) percent = -percent;
  if (percent > 100.0f) percent = 100.0f;
  if (percent < -100.0f) percent = -100.0f;

  // Tiny commands are treated as "stop". Anything bigger is stretched so that
  // 1% means "just enough to move" (minPercent) and 100% stays 100%.
  const float magnitude = fabsf(percent);
  uint32_t duty = 0;
  if (magnitude >= 0.5f) {
    const float effective = _cfg.minPercent + (100.0f - _cfg.minPercent) * magnitude / 100.0f;
    duty = (uint32_t)(effective * MAX_DUTY / 100.0f + 0.5f);
    if (duty > MAX_DUTY) duty = MAX_DUTY;
  }

  const int a = side * 2;       // this motor's IN1
  const int b = side * 2 + 1;   // this motor's IN2
  if (percent >= 0) {
    writePwm(a, duty);
    writePwm(b, 0);
  } else {
    writePwm(a, 0);
    writePwm(b, duty);
  }
}

void MotorDriver::report(Print& out) {
  out.printf("[motors] left=%.0f%% right=%.0f%%\n", _lastPercent[0], _lastPercent[1]);
}
