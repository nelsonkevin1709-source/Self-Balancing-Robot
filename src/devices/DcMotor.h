/*****************
DcMotor.h
Self-Balancing Robot

One DC motor on one channel of the DRV8833 driver. Make one object per wheel.

The DRV8833 has two inputs per motor (IN1, IN2):
	IN1     IN2     motor
	PWM     LOW     forward, "fast decay" (coasts between pulses)
	LOW     PWM     reverse, fast decay
	HIGH    PWM     forward, "slow decay" (brakes between pulses)
	PWM     HIGH    reverse, slow decay
	LOW     LOW     coast (free spinning)
	HIGH    HIGH    brake (wheel resists turning)

Slow decay is the default: speed follows the command more linearly, which helps
balancing. You only ever call setOutput(-1 .. +1); the class works out the pins.
*****************/

#pragma once
#include "hal/HardwareInterface.h"
#include <stdint.h>

class DcMotor {
public:
	enum class DecayMode { Fast, Slow };

	// in1Pin/in2Pin: ESP32 pins wired to the DRV8833 IN1/IN2 of this motor.
	// pwmChannel1/2: two free ESP32 PWM channels (0..15), one per pin.
	// inverted: true if this motor is mounted the other way round.
	DcMotor(HardwareInterface* hw, int in1Pin, int in2Pin, int pwmChannel1, int pwmChannel2, bool inverted = false);
	DcMotor(const DcMotor&) = delete;
	DcMotor& operator=(const DcMotor&) = delete;
	~DcMotor();                         // stops the motor (coast)

	void begin(uint32_t pwmFreqHz = 20000, uint8_t pwmBits = 10);

	// -1.0 = full reverse, 0 = stop (coast), +1.0 = full forward. Clipped to +/- maxOutput.
	void setOutput(float command);
	float getOutput() const { return output_; }

	void coast();                       // both inputs LOW
	void brake();                       // both inputs HIGH

	void setMaxOutput(float maxOutput); // e.g. 0.7 to protect 6 V motors on an 8.4 V battery
	float getMaxOutput() const { return maxOutput_; }
	void setDecayMode(DecayMode mode) { decay_ = mode; }
	void setInverted(bool inverted) { inverted_ = inverted; }
	bool isInverted() const { return inverted_; }

private:
	void writeDuties(float in1Level, float in2Level);   // 0..1 for each pin

	HardwareInterface* hw_;
	int in1Pin_, in2Pin_;
	int channel1_, channel2_;
	bool inverted_;
	bool started_ = false;
	uint32_t maxDuty_ = 1023;
	float maxOutput_ = 1.0f;
	float output_ = 0.0f;
	DecayMode decay_ = DecayMode::Slow;
};
