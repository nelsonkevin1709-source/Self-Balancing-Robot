/*****************
config.h
Self-Balancing Robot

Every pin number and tuning constant lives here, so nothing is "hard coded"
inside the classes. If you rewire something, this is the only file to change.
*****************/

#pragma once
#include <stdint.h>

namespace cfg {

// ---------------- MPU6050 (I2C) ----------------
constexpr int      kPinSda       = 21;
constexpr int      kPinScl       = 22;
constexpr uint32_t kI2cHz        = 400000;   // 400 kHz "fast mode"
constexpr uint8_t  kImuAddress   = 0x68;     // 0x69 if the AD0 pin is tied high

// ---------------- DRV8833 motor driver ----------------
// nSLEEP is tied to 3V3 (always awake). Each motor uses two pins (IN1/IN2),
// and each pin gets its own PWM channel on the ESP32.
constexpr int kPinLeftIn1  = 25;
constexpr int kPinLeftIn2  = 26;
constexpr int kPinRightIn1 = 32;
constexpr int kPinRightIn2 = 33;

constexpr int kPwmChLeft1  = 0;
constexpr int kPwmChLeft2  = 1;
constexpr int kPwmChRight1 = 2;
constexpr int kPwmChRight2 = 3;

constexpr uint32_t kPwmFreqHz = 20000;  // 20 kHz: above hearing, so no motor whine
constexpr uint8_t  kPwmBits   = 10;     // duty 0..1023

// The two motors face opposite ways, so the same command spins them in opposite
// directions. Flip these if the motor test says a wheel goes the wrong way.
constexpr bool kLeftMotorInverted  = false;
constexpr bool kRightMotorInverted = true;

// Largest motor command allowed (0..1). TT motors are rated 6 V:
// 4xAA (6 V) -> 1.0.  2S lithium (8.4 V full) -> 0.7.
constexpr float kMotorMaxOutput = 1.0f;

// ---------------- HC-SR04 ultrasonic ----------------
// Echo is 5 V: it MUST go through a divider (1k from Echo, 2k to GND) before GPIO35.
constexpr int      kPinUltrasonicTrig = 27;
constexpr int      kPinUltrasonicEcho = 35;  // input-only pin, fine for Echo
constexpr uint32_t kUltrasonicTimeoutUs = 25000;  // ~4 m round trip

// ---------------- Misc ----------------
constexpr int  kPinStatusLed      = 2;
constexpr const char* kBluetoothName = "BalanceBot";
constexpr uint32_t kSerialBaud    = 115200;

}  // namespace cfg
