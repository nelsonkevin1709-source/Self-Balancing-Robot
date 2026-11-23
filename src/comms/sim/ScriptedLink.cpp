#include "comms/sim/ScriptedLink.h"
#include <cstring>

bool ScriptedLink::begin(const char* deviceName) {
	started_ = true;
	sim_.printf("[phone] can now see a Bluetooth device called \"%s\"\n", deviceName);
	return true;
}

bool ScriptedLink::isConnected() {
	const bool connected = started_ && sim_.seconds() >= connectTime_;
	if (connected && !announced_) {
		announced_ = true;
		sim_.print("[phone] connected\n");
	}
	return connected;
}

bool ScriptedLink::readLine(char* out, size_t maxLength) {
	if (!isConnected() || next_ >= script_.size() || sim_.seconds() < script_[next_].time) {
		return false;
	}
	sim_.printf("[phone] sends \"%s\"\n", script_[next_].text.c_str());
	std::strncpy(out, script_[next_].text.c_str(), maxLength);
	out[maxLength - 1] = '\0';
	next_++;
	return true;
}

void ScriptedLink::send(const char* text) {
	if (!isConnected()) return;
	sim_.printf("[phone] receives: %s", text);
	const size_t n = std::strlen(text);
	if (n == 0 || text[n - 1] != '\n') sim_.print("\n");
}
