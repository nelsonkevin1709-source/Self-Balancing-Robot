/*****************
Esp32Hardware.h
Self-Balancing Robot

The REAL implementation of HardwareInterface: each function just forwards to the
Arduino-ESP32 library (pinMode, ledcWrite, Wire, Serial, ...).
Only compiled for the ESP32 (PlatformIO), never for the simulator.
*****************/

#pragma once
#include "hal/HardwareInterface.h"

class Esp32Hardware : public HardwareInterface {
public:
	Esp32Hardware() = default;
	Esp32Hardware(const Esp32Hardware&) = delete;             // there is only one ESP32
	Esp32Hardware& operator=(const Esp32Hardware&) = delete;

	void begin(uint32_t serialBaud);    // call once at the top of setup()

	uint32_t millis() override;
	uint32_t micros() override;
	void delayMs(uint32_t ms) override;
	void delayUs(uint32_t us) override;

	void pinOutput(int pin) override;
	void pinInput(int pin) override;
	void digitalWrite(int pin, bool high) override;
	bool digitalRead(int pin) override;
	uint32_t pulseInHigh(int pin, uint32_t timeoutUs) override;

	void pwmAttach(int pin, int channel, uint32_t freqHz, uint8_t bits) override;
	void pwmWrite(int channel, uint32_t duty) override;

	bool i2cBegin(int sdaPin, int sclPin, uint32_t clockHz) override;
	bool i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value) override;
	bool i2cReadRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, size_t length) override;

	void print(const char* text) override;
	int readChar() override;
};
