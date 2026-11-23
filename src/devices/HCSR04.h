/*****************
HCSR04.h
Self-Balancing Robot

Driver for the HC-SR04 ultrasonic distance sensor.

How it works: we send a 10 us pulse on TRIG. The sensor sends a burst of sound,
then holds ECHO high for as long as the sound took to bounce back.
	distance = echo time x speed of sound / 2   (there and back)

WIRING WARNING: the HC-SR04 runs on 5 V and its ECHO pin outputs 5 V. The ESP32
only takes 3.3 V. Put a divider on ECHO: 1 kOhm from ECHO to the ESP32 pin,
2 kOhm from the ESP32 pin to GND.

Note: readDistanceCm() waits for the echo (up to ~25 ms). Fine for testing.
When we connect everything together we'll move it off the balance loop.
*****************/

#pragma once
#include "hal/HardwareInterface.h"
#include <stdint.h>

class HCSR04 {
public:
	HCSR04(HardwareInterface* hw, int trigPin, int echoPin, uint32_t timeoutUs = 25000);
	HCSR04(const HCSR04&) = delete;
	HCSR04& operator=(const HCSR04&) = delete;

	void begin();

	// Distance in cm, or a negative number if there was no echo (nothing in range, or wiring problem).
	// Calls closer than 60 ms apart return the previous value (the sensor needs time for echoes to die out).
	float readDistanceCm();
	bool lastReadingValid() const { return lastValid_; }
	uint32_t lastEchoUs() const { return lastEchoUs_; }

	void setTemperatureC(float celsius);   // speed of sound changes ~0.6 m/s per degree

	static constexpr float kNoEcho = -1.0f;
	static constexpr uint32_t kMinIntervalMs = 60;

private:
	HardwareInterface* hw_;
	int trigPin_, echoPin_;
	uint32_t timeoutUs_;
	float speedOfSoundCmPerUs_ = 0.0343f;   // at ~20 deg C
	bool started_ = false;
	bool hasReading_ = false;
	bool lastValid_ = false;
	float lastCm_ = kNoEcho;
	uint32_t lastEchoUs_ = 0;
	uint32_t lastPingMs_ = 0;
};
