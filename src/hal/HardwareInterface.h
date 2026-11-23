/*****************
HardwareInterface.h
Self-Balancing Robot

This is our version of MTRN3500's EmbeddedFunctions.

Every class that talks to hardware (MPU6050, DcMotor, HCSR04, ...) is given a
pointer to a HardwareInterface and does ALL its pin / I2C / timing work through
it. It never calls Arduino functions directly.

Why? Because there are two implementations:
    Esp32Hardware  -> real pins on the real ESP32           (src/hal/esp32/)
    SimHardware    -> a physics model of the robot on your PC (src/hal/sim/)

So the exact same MPU6050 / motor / ultrasonic code runs on the robot AND in
the simulator, just like Galil ran against the real board or the simulator
depending on which EmbeddedFunctions it was given.

All functions are pure virtual (= 0): this class only describes WHAT the
hardware can do. The derived classes decide HOW.
*****************/

#pragma once
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

class HardwareInterface {
public:
	virtual ~HardwareInterface() = default;

	// ---------------- TIME ----------------
	virtual uint32_t millis() = 0;                  // ms since start-up
	virtual uint32_t micros() = 0;                  // us since start-up
	virtual void delayMs(uint32_t ms) = 0;          // wait (blocking)
	virtual void delayUs(uint32_t us) = 0;

	// ---------------- DIGITAL PINS ----------------
	virtual void pinOutput(int pin) = 0;            // configure pin as output
	virtual void pinInput(int pin) = 0;             // configure pin as input
	virtual void digitalWrite(int pin, bool high) = 0;
	virtual bool digitalRead(int pin) = 0;
	// Wait for a HIGH pulse on 'pin' and return its length in us (0 = timed out).
	virtual uint32_t pulseInHigh(int pin, uint32_t timeoutUs) = 0;

	// ---------------- PWM ----------------
	// Connect 'pin' to PWM 'channel' (ESP32 has 16 channels) at freqHz with 'bits' resolution.
	virtual void pwmAttach(int pin, int channel, uint32_t freqHz, uint8_t bits) = 0;
	// duty: 0 .. (2^bits - 1)
	virtual void pwmWrite(int channel, uint32_t duty) = 0;

	// ---------------- I2C ----------------
	virtual bool i2cBegin(int sdaPin, int sclPin, uint32_t clockHz) = 0;
	// Write one byte 'value' into register 'reg' of device 'address'. true = device acknowledged.
	virtual bool i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value) = 0;
	// Read 'length' bytes starting at register 'reg'. true = all bytes received.
	virtual bool i2cReadRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, size_t length) = 0;

	// ---------------- TEXT I/O (USB serial on the ESP32, the terminal in the simulator) ----------------
	virtual void print(const char* text) = 0;
	virtual int readChar() = 0;                     // next typed character, or -1 if none

	// printf-style helper built on print(). Not virtual: works for every derived class.
	void printf(const char* format, ...) {
		char buffer[256];
		va_list args;
		va_start(args, format);
		vsnprintf(buffer, sizeof(buffer), format, args);
		va_end(args);
		print(buffer);
	}

	// ---------------- SIMULATOR OBSERVER ----------------
	// Real hardware cannot see the robot, so a human answers yes/no questions in the
	// tests ("is the left wheel spinning forward?"). The simulator CAN see its own
	// physics, so it overrides this to answer automatically.
	// 'expectation' is a short code such as "L+" (left forward) or "L+R+" (both forward).
	// Returns 1 = yes, 0 = no, -1 = "I can't tell, ask the human" (the default).
	virtual int observe(const char* expectation) { (void)expectation; return -1; }
};
