#include "hal/esp32/Esp32Hardware.h"
#include <Arduino.h>
#include <Wire.h>

// Note the "::" in front of Arduino functions: it means "the global Arduino
// function", not our member function of the same name.

void Esp32Hardware::begin(uint32_t serialBaud) {
	Serial.begin(serialBaud);
	::delay(200);   // give the USB serial a moment so the first lines are not lost
}

uint32_t Esp32Hardware::millis() { return ::millis(); }
uint32_t Esp32Hardware::micros() { return ::micros(); }
void Esp32Hardware::delayMs(uint32_t ms) { ::delay(ms); }
void Esp32Hardware::delayUs(uint32_t us) { ::delayMicroseconds(us); }

void Esp32Hardware::pinOutput(int pin) { ::pinMode(pin, OUTPUT); }
void Esp32Hardware::pinInput(int pin) { ::pinMode(pin, INPUT); }
void Esp32Hardware::digitalWrite(int pin, bool high) { ::digitalWrite(pin, high ? HIGH : LOW); }
bool Esp32Hardware::digitalRead(int pin) { return ::digitalRead(pin) == HIGH; }

uint32_t Esp32Hardware::pulseInHigh(int pin, uint32_t timeoutUs) {
	return ::pulseIn(pin, HIGH, timeoutUs);
}

// Arduino-ESP32 2.x PWM ("LEDC") API. (Version 3.x renamed these functions;
// platformio.ini pins the platform to a 2.x core.)
void Esp32Hardware::pwmAttach(int pin, int channel, uint32_t freqHz, uint8_t bits) {
	ledcSetup(channel, freqHz, bits);
	ledcAttachPin(pin, channel);
}

void Esp32Hardware::pwmWrite(int channel, uint32_t duty) { ledcWrite(channel, duty); }

bool Esp32Hardware::i2cBegin(int sdaPin, int sclPin, uint32_t clockHz) {
	return Wire.begin(sdaPin, sclPin, clockHz);
}

bool Esp32Hardware::i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value) {
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(value);
	return Wire.endTransmission() == 0;    // 0 = device acknowledged
}

bool Esp32Hardware::i2cReadRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, size_t length) {
	Wire.beginTransmission(address);
	Wire.write(reg);
	if (Wire.endTransmission(false) != 0) {    // false = keep the bus for the read
		return false;
	}
	size_t received = Wire.requestFrom(static_cast<int>(address), static_cast<int>(length));
	if (received != length) {
		return false;
	}
	for (size_t i = 0; i < length; i++) {
		buffer[i] = static_cast<uint8_t>(Wire.read());
	}
	return true;
}

void Esp32Hardware::print(const char* text) { Serial.print(text); }
int Esp32Hardware::readChar() { return Serial.available() > 0 ? Serial.read() : -1; }
