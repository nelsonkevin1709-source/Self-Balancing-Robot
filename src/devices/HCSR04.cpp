#include "devices/HCSR04.h"

HCSR04::HCSR04(HardwareInterface* hw, int trigPin, int echoPin, uint32_t timeoutUs)
	: hw_(hw), trigPin_(trigPin), echoPin_(echoPin), timeoutUs_(timeoutUs) {}

void HCSR04::begin() {
	hw_->pinOutput(trigPin_);
	hw_->pinInput(echoPin_);
	hw_->digitalWrite(trigPin_, false);
	started_ = true;
}

void HCSR04::setTemperatureC(float celsius) {
	speedOfSoundCmPerUs_ = (331.3f + 0.606f * celsius) * 100.0f / 1e6f;
}

float HCSR04::readDistanceCm() {
	if (!started_) return kNoEcho;

	const uint32_t now = hw_->millis();
	if (hasReading_ && now - lastPingMs_ < kMinIntervalMs) {
		return lastCm_;
	}
	lastPingMs_ = now;
	hasReading_ = true;

	// 10 us trigger pulse
	hw_->digitalWrite(trigPin_, false);
	hw_->delayUs(2);
	hw_->digitalWrite(trigPin_, true);
	hw_->delayUs(10);
	hw_->digitalWrite(trigPin_, false);

	lastEchoUs_ = hw_->pulseInHigh(echoPin_, timeoutUs_);
	lastValid_ = lastEchoUs_ > 0;
	lastCm_ = lastValid_ ? lastEchoUs_ * speedOfSoundCmPerUs_ / 2.0f : kNoEcho;
	return lastCm_;
}
