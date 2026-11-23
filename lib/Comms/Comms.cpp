#include "Comms.h"
#include <NimBLEDevice.h>
#include <ctype.h>
#include <string.h>

namespace {
// Standard Nordic UART Service UUIDs - recognised by many phone apps.
const char* const NUS_SERVICE = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* const NUS_RX      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";  // phone -> robot
const char* const NUS_TX      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";  // robot -> phone

Comms* g_comms = nullptr;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*) override {
    if (g_comms) g_comms->onBleConnection(true);
  }
  void onDisconnect(NimBLEServer*) override {
    if (g_comms) g_comms->onBleConnection(false);
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic) override {
    if (!g_comms) return;
    const std::string value = characteristic->getValue();
    g_comms->onBleWrite((const uint8_t*)value.data(), value.size());
  }
};
}  // namespace

bool Comms::begin() {
  _queue = xQueueCreate(kQueueLen, kLineMax);
  if (!_queue) return false;
  g_comms = this;

  NimBLEDevice::init(_deviceName);
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(NUS_SERVICE);
  NimBLECharacteristic* rx = service->createCharacteristic(
      NUS_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  _txChar = service->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  adv->start();

  Serial.printf("[comms] Bluetooth LE advertising as \"%s\"\n", _deviceName);
  return true;
}

// Runs on the Bluetooth task: split the incoming bytes into commands and queue them.
void Comms::onBleWrite(const uint8_t* data, size_t len) {
  if (!_queue) return;
  char line[kLineMax];
  size_t n = 0;
  for (size_t i = 0; i <= len; i++) {
    // The end of a packet counts as the end of a command, so apps that send "F"
    // without a newline still work.
    const char c = (i < len) ? (char)data[i] : '\n';
    if (c == '\n' || c == '\r') {
      if (n > 0) {
        line[n] = '\0';
        xQueueSend(_queue, line, 0);
        n = 0;
      }
    } else if (n < kLineMax - 1) {
      line[n++] = c;
    }
  }
}

void Comms::onBleConnection(bool connected) {
  _connected = connected;
  if (!connected) {   // lost the phone: stand still
    _throttle = 0;
    _turn = 0;
  }
}

void Comms::update() {
  // 1. Handle commands that arrived from the phone.
  char line[kLineMax];
  while (_queue && xQueueReceive(_queue, line, 0) == pdTRUE) handleLine(line);

  // 2. Send queued text to the phone, one small packet per update.
  if (!_connected) {
    _txTail = _txHead;   // nobody listening: discard
  } else if (_txTail != _txHead && _txChar) {
    uint8_t chunk[kNotifyChunk];
    size_t n = 0;
    while (n < kNotifyChunk && _txTail != _txHead) {
      chunk[n++] = (uint8_t)_txBuf[_txTail];
      _txTail = (_txTail + 1) % kTxSize;
    }
    _txChar->setValue(chunk, n);
    _txChar->notify();
  }

  // 3. If the phone disconnected, make sure we are visible again.
  if (!_connected) {
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    if (!adv->isAdvertising()) adv->start();
  }
}

void Comms::handleLine(const char* line) {
  while (*line == ' ') line++;
  if (*line == '\0') return;

  // A lone letter is a drive command.
  if (line[1] == '\0') {
    switch (toupper((unsigned char)line[0])) {
      case 'F': _throttle = 1.0f;  return;
      case 'B': _throttle = -1.0f; return;
      case 'L': _turn = -1.0f;     return;
      case 'R': _turn = 1.0f;      return;
      case 'S': _throttle = 0.0f; _turn = 0.0f; return;
      default: break;
    }
  }
  if (_handler) _handler(line);
}

void Comms::send(const char* text) {
  if (!_connected) return;
  for (const char* p = text; *p; p++) {
    const uint16_t next = (_txHead + 1) % kTxSize;
    if (next == _txTail) break;   // buffer full: drop the rest
    _txBuf[_txHead] = *p;
    _txHead = next;
  }
}

void Comms::report(Print& out) {
  out.printf("[comms] %s throttle=%.0f turn=%.0f\n",
             _connected ? "phone connected" : "waiting for phone", _throttle, _turn);
}
