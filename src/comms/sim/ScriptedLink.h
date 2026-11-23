/*****************
ScriptedLink.h
Self-Balancing Robot (simulator only)

A pretend phone. A test scenario schedules messages ("at 3.0 s the phone sends F"),
and anything the robot sends to the phone is printed with a [phone] prefix.
*****************/

#pragma once
#include "comms/CommandLink.h"
#include "hal/sim/SimHardware.h"
#include <string>
#include <vector>

class ScriptedLink : public CommandLink {
public:
	explicit ScriptedLink(SimHardware& sim) : sim_(sim) {}

	// Phone connects at this time (seconds). Default: never.
	void connectAt(double seconds) { connectTime_ = seconds; }
	// Phone sends 'text' at this time. Add messages in time order.
	void sendAt(double seconds, const std::string& text) { script_.push_back({ seconds, text }); }

	bool begin(const char* deviceName) override;
	bool isConnected() override;
	bool readLine(char* out, size_t maxLength) override;
	void send(const char* text) override;

private:
	struct Message { double time; std::string text; };
	SimHardware& sim_;
	std::vector<Message> script_;
	size_t next_ = 0;
	double connectTime_ = 1e9;
	bool started_ = false;
	bool announced_ = false;
};
