#include "devices/DcMotor.h"

DcMotor::DcMotor(HardwareInterface* hw, int in1Pin, int in2Pin, int pwmChannel1, int pwmChannel2, bool inverted)
	: hw_(hw), in1Pin_(in1Pin), in2Pin_(in2Pin), channel1_(pwmChannel1), channel2_(pwmChannel2), inverted_(inverted) {}

DcMotor::~DcMotor() {
	if (started_) {
		coast();
	}
}

void DcMotor::begin(uint32_t pwmFreqHz, uint8_t pwmBits) {
	hw_->pwmAttach(in1Pin_, channel1_, pwmFreqHz, pwmBits);
	hw_->pwmAttach(in2Pin_, channel2_, pwmFreqHz, pwmBits);
	maxDuty_ = (1u << pwmBits) - 1;
	started_ = true;
	coast();
}

void DcMotor::setMaxOutput(float maxOutput) {
	if (maxOutput < 0.0f) maxOutput = 0.0f;
	if (maxOutput > 1.0f) maxOutput = 1.0f;
	maxOutput_ = maxOutput;
}

void DcMotor::writeDuties(float in1Level, float in2Level) {
	if (!started_) return;
	hw_->pwmWrite(channel1_, static_cast<uint32_t>(in1Level * maxDuty_ + 0.5f));
	hw_->pwmWrite(channel2_, static_cast<uint32_t>(in2Level * maxDuty_ + 0.5f));
}

void DcMotor::setOutput(float command) {
	if (command > maxOutput_) command = maxOutput_;
	if (command < -maxOutput_) command = -maxOutput_;
	output_ = command;

	if (inverted_) command = -command;
	if (command == 0.0f) {
		writeDuties(0.0f, 0.0f);   // coast
		return;
	}
	const bool forward = command > 0.0f;
	const float magnitude = forward ? command : -command;

	if (decay_ == DecayMode::Fast) {
		// One input PWM, the other LOW.
		if (forward) writeDuties(magnitude, 0.0f);
		else         writeDuties(0.0f, magnitude);
	} else {
		// One input HIGH, the other PWM with the duty "upside down".
		if (forward) writeDuties(1.0f, 1.0f - magnitude);
		else         writeDuties(1.0f - magnitude, 1.0f);
	}
}

void DcMotor::coast() {
	output_ = 0.0f;
	writeDuties(0.0f, 0.0f);
}

void DcMotor::brake() {
	output_ = 0.0f;
	writeDuties(1.0f, 1.0f);
}
