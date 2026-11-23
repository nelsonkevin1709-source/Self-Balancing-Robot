#include "comms/esp32/BleUartLink.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string.h>

namespace {
// Standard Nordic UART Service UUIDs (phone apps look for these)
const char* kServiceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* kRxUuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";   // phone -> robot
const char* kTxUuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";   // robot -> phone

portMUX_TYPE bufferLock = portMUX_INITIALIZER_UNLOCKED;

class ServerCallbacks : public NimBLEServerCallbacks {
public:
	explicit ServerCallbacks(BleUartLink* link) : link_(link) {}
	void onConnect(NimBLEServer* server) override { (void)server; link_->onConnectionChanged(true); }
	void onDisconnect(NimBLEServer* server) override {
		(void)server;
		link_->onConnectionChanged(false);
		NimBLEDevice::startAdvertising();   // let the phone reconnect
	}
private:
	BleUartLink* link_;
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
public:
	explicit RxCallbacks(BleUartLink* link) : link_(link) {}
	void onWrite(NimBLECharacteristic* characteristic) override {
		std::string value = characteristic->getValue();
		link_->onReceived(reinterpret_cast<const uint8_t*>(value.data()), value.size());
	}
private:
	BleUartLink* link_;
};
}  // namespace

bool BleUartLink::begin(const char* deviceName) {
	if (started_) return true;
	NimBLEDevice::init(deviceName);
	NimBLEServer* server = NimBLEDevice::createServer();
	server->setCallbacks(new ServerCallbacks(this));

	NimBLEService* service = server->createService(kServiceUuid);
	txCharacteristic_ = service->createCharacteristic(kTxUuid, NIMBLE_PROPERTY::NOTIFY);
	NimBLECharacteristic* rx = service->createCharacteristic(kRxUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
	rx->setCallbacks(new RxCallbacks(this));
	service->start();

	NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
	advertising->addServiceUUID(kServiceUuid);
	advertising->setScanResponse(true);   // room for the device name
	started_ = advertising->start();
	return started_;
}

bool BleUartLink::isConnected() { return connected_; }

void BleUartLink::onConnectionChanged(bool connected) { connected_ = connected; }

// Bluetooth task: copy bytes into the ring buffer. Each phone message also counts as a
// complete line, because many apps don't add a newline.
void BleUartLink::onReceived(const uint8_t* data, size_t length) {
	portENTER_CRITICAL(&bufferLock);
	for (size_t i = 0; i <= length; i++) {
		const char c = (i < length) ? static_cast<char>(data[i]) : '\n';
		const size_t next = (head_ + 1) % kBufferSize;
		if (next == tail_) break;    // buffer full: drop the rest
		buffer_[head_] = c;
		head_ = next;
	}
	portEXIT_CRITICAL(&bufferLock);
}

bool BleUartLink::popChar(char& c) {
	bool got = false;
	portENTER_CRITICAL(&bufferLock);
	if (tail_ != head_) {
		c = buffer_[tail_];
		tail_ = (tail_ + 1) % kBufferSize;
		got = true;
	}
	portEXIT_CRITICAL(&bufferLock);
	return got;
}

bool BleUartLink::readLine(char* out, size_t maxLength) {
	char c;
	while (popChar(c)) {
		if (c == '\r' || c == '\n') {
			if (lineLength_ == 0) continue;     // ignore empty lines ("\r\n" gives two ends)
			line_[lineLength_] = '\0';
			strncpy(out, line_, maxLength);
			out[maxLength - 1] = '\0';
			lineLength_ = 0;
			return true;
		}
		if (lineLength_ < sizeof(line_) - 1) {
			line_[lineLength_++] = c;
		}
	}
	return false;
}

void BleUartLink::send(const char* text) {
	if (!started_ || !connected_ || txCharacteristic_ == nullptr) return;
	// BLE packets carry 20 bytes unless the phone negotiates more, so send in pieces.
	const size_t length = strlen(text);
	for (size_t start = 0; start < length; start += 20) {
		const size_t chunk = (length - start < 20) ? (length - start) : 20;
		txCharacteristic_->setValue(reinterpret_cast<const uint8_t*>(text + start), chunk);
		txCharacteristic_->notify();
		delay(5);
	}
}
