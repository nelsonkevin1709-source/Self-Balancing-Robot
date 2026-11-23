#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------
//  Module - the building block of the whole robot.
//
//  Every part (IMU, motors, Bluetooth, and any sensor you add later)
//  is a class that inherits from Module and fills in four small
//  functions. The main program only ever talks to this interface, so
//  adding a new sensor never means touching the balancing code.
// ---------------------------------------------------------------------
class Module {
 public:
  // updateIntervalUs: how often update() should run, in microseconds.
  //   5000  = 200 times per second (fast: IMU, balancing)
  //   50000 = 20 times per second (fine for a distance sensor)
  //   0     = every pass through loop()
  explicit Module(uint32_t updateIntervalUs = 0) : _intervalUs(updateIntervalUs) {}
  virtual ~Module() {}

  virtual const char* name() const = 0;   // short name used in log messages
  virtual bool begin() = 0;               // start the hardware. Return false if it failed.
  virtual void update() = 0;              // do one step of work. NEVER call delay() in here.
  virtual void report(Print& out) { (void)out; }   // optional: print a one-line status

  // Used by ModuleRegistry to decide whether it is time to run update().
  bool due(uint32_t nowUs) {
    if (_intervalUs == 0) return true;
    if ((uint32_t)(nowUs - _lastUs) < _intervalUs) return false;
    _lastUs += _intervalUs;
    if ((uint32_t)(nowUs - _lastUs) >= _intervalUs) _lastUs = nowUs;  // fell behind: don't catch up
    return true;
  }

 private:
  uint32_t _intervalUs;
  uint32_t _lastUs = 0;
};
