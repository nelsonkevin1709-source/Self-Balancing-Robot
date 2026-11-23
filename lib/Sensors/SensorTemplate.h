#pragma once
#include <Arduino.h>
#include <Module.h>

// =====================================================================
//  SensorTemplate - copy this file to add a new sensor.
//
//  Example: an ultrasonic distance sensor, a wheel encoder, a battery
//  voltage monitor, a line sensor...
//
//  STEPS
//   1. Make a folder  lib/MySensor/  and copy this file into it as
//      MySensor.h. Rename the class (SensorTemplate -> MySensor).
//   2. Fill in begin(), update() and (optionally) report().
//   3. In src/main.cpp:
//         #include <MySensor.h>
//         MySensor mySensor(PIN_FOR_IT);            // near the other modules
//         ...
//         registry.add(mySensor);                   // in setup(), where marked
//   4. That's it. The sensor now starts, updates at its own speed, and
//      shows up when you send "?" over Bluetooth.
//
//  To make the robot REACT to the sensor, use the value in main.cpp,
//  next to the line that feeds Bluetooth input into Balance, e.g.
//         if (mySensor.value() < 200) throttle = 0;   // stop for obstacles
//  The balancing code never needs to know the sensor exists.
//
//  RULES FOR update():
//   - It must be quick. Never use delay() inside it.
//   - Choose a sensible interval below so a slow sensor can't slow
//     down the balancing loop.
// =====================================================================
class SensorTemplate : public Module {
 public:
  // 50000 microseconds = update 20 times per second.
  explicit SensorTemplate(uint8_t pin) : Module(50000), _pin(pin) {}

  const char* name() const override { return "template"; }

  bool begin() override {
    pinMode(_pin, INPUT);
    return true;           // return false if the hardware is missing / not answering
  }

  void update() override {
    _value = analogRead(_pin);   // replace with your sensor reading
  }

  void report(Print& out) override {
    out.printf("[template] value=%d\n", _value);
  }

  int value() const { return _value; }

 private:
  uint8_t _pin;
  int _value = 0;
};
