#pragma once
#include <Arduino.h>
#include <Module.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class NimBLECharacteristic;   // defined by the NimBLE library; only Comms.cpp needs the details

// Bluetooth Low Energy link to a phone, using the standard "Nordic UART" service
// (a text pipe both ways). Works with iPhone and Android.
//
// Single-letter drive commands are handled here:
//    F = forward   B = backward   L = left   R = right   S = stop (clears both)
// Anything else (e.g. "P12.5") is handed to the function you give setLineHandler().
//
// Bluetooth callbacks arrive on a different task than loop(), so they only put text
// into a queue; update() reads the queue on the main task. That avoids nasty timing bugs.
class Comms : public Module {
 public:
  typedef void (*LineHandler)(const char* line);

  explicit Comms(const char* deviceName = "BalanceBot", uint32_t updateIntervalUs = 10000)
      : Module(updateIntervalUs), _deviceName(deviceName) {}

  const char* name() const override { return "comms"; }
  bool begin() override;
  void update() override;
  void report(Print& out) override;

  void  setLineHandler(LineHandler handler) { _handler = handler; }
  float throttle() const { return _throttle; }   // -1 (backward) .. +1 (forward)
  float turn() const { return _turn; }           // -1 (left) .. +1 (right)
  bool  connected() const { return _connected; }

  // Queue text to be sent to the phone. Sent a few bytes per update(), so this never blocks.
  void send(const char* text);

  // Called by the Bluetooth library callbacks. Not for general use.
  void onBleWrite(const uint8_t* data, size_t len);
  void onBleConnection(bool connected);

 private:
  void handleLine(const char* line);

  static const size_t kLineMax = 32;
  static const size_t kQueueLen = 8;
  static const size_t kTxSize = 512;
  static const size_t kNotifyChunk = 20;   // safe size for a default BLE packet

  const char*          _deviceName;
  LineHandler          _handler = nullptr;
  QueueHandle_t        _queue = nullptr;
  NimBLECharacteristic* _txChar = nullptr;

  volatile bool  _connected = false;
  volatile float _throttle = 0;
  volatile float _turn = 0;

  char     _txBuf[kTxSize];
  uint16_t _txHead = 0;
  uint16_t _txTail = 0;
};
