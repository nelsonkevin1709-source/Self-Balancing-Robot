/*****************
BleUartLink.h
Self-Balancing Robot (ESP32 only)

Bluetooth Low Energy using the "Nordic UART Service" (NUS): a standard way to
send text over BLE. Works with iPhone AND Android using a free app, e.g.
	- nRF Toolbox (UART)  or  nRF Connect          (iPhone + Android)
	- Bluefruit Connect (UART)                      (iPhone + Android)
	- Serial Bluetooth Terminal                     (Android)

Uses the ESP32's built-in Bluetooth (no extra module) and the NimBLE-Arduino library.

Bluetooth messages arrive on a different task (thread) than loop(), so incoming
characters go into a small ring buffer protected by a lock.
*****************/

#pragma once
#include "comms/CommandLink.h"
#include <stdint.h>

class NimBLECharacteristic;

class BleUartLink : public CommandLink {
public:
	BleUartLink() = default;
	BleUartLink(const BleUartLink&) = delete;
	BleUartLink& operator=(const BleUartLink&) = delete;

	bool begin(const char* deviceName) override;
	bool isConnected() override;
	bool readLine(char* out, size_t maxLength) override;
	void send(const char* text) override;

	// Called from the Bluetooth task (by the callback classes in the .cpp). Not for you to call.
	void onReceived(const uint8_t* data, size_t length);
	void onConnectionChanged(bool connected);

private:
	bool popChar(char& c);

	static constexpr size_t kBufferSize = 256;
	char buffer_[kBufferSize] = {};
	volatile size_t head_ = 0;   // next write position
	volatile size_t tail_ = 0;   // next read position
	volatile bool connected_ = false;

	char line_[128] = {};
	size_t lineLength_ = 0;

	NimBLECharacteristic* txCharacteristic_ = nullptr;
	bool started_ = false;
};
