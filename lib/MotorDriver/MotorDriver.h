#pragma once
#include <Arduino.h>
#include <Module.h>
#include <Interfaces.h>

struct MotorConfig {
  uint8_t  leftIn1  = 25;
  uint8_t  leftIn2  = 26;
  uint8_t  rightIn1 = 32;
  uint8_t  rightIn2 = 33;
  int8_t   sleepPin = -1;        // DRV8833 nSLEEP pin, or -1 if tied to 3V3
  bool     invertLeft  = false;  // flip if the wheel turns the wrong way
  bool     invertRight = false;
  uint32_t pwmHz = 20000;
  float    minPercent = 0.0f;    // power needed to overcome friction (see stage 3)
};

// Drives two DC motors through a DRV8833.
// The DRV8833 has two input pins per motor:
//    IN1 = PWM, IN2 = low  -> motor turns one way
//    IN1 = low, IN2 = PWM  -> motor turns the other way
//    both low              -> motor coasts (stopped)
class MotorDriver : public Module, public DriveOutput {
 public:
  explicit MotorDriver(const MotorConfig& cfg) : Module(0), _cfg(cfg) {}

  const char* name() const override { return "motors"; }
  bool begin() override;
  void update() override {}   // nothing to do each loop; power is set the moment you ask for it
  void report(Print& out) override;

  // DriveOutput
  void setSpeeds(float leftPercent, float rightPercent) override;
  void stop() override { setSpeeds(0, 0); }

  void setLeft(float percent)  { setMotor(0, percent); }
  void setRight(float percent) { setMotor(1, percent); }

 private:
  void setMotor(int side, float percent);   // side 0 = left, 1 = right
  void writePwm(int index, uint32_t duty);

  MotorConfig _cfg;
  uint8_t _pins[4] = {0, 0, 0, 0};   // left IN1, left IN2, right IN1, right IN2
  float   _lastPercent[2] = {0, 0};
  bool    _ready = false;
};
