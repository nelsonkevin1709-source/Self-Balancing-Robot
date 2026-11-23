/*****************
CommandLink.h
Self-Balancing Robot

A text link to the phone. Same idea as HardwareInterface: the rest of the code
only uses this interface, and there are two implementations:
	BleUartLink   -> real Bluetooth Low Energy on the ESP32 (src/comms/esp32/)
	ScriptedLink  -> pretend phone messages in the simulator (src/comms/sim/)

Messages are lines of text, e.g. "F" (forward) or "P12.5" (set Kp).
*****************/

#pragma once
#include <stddef.h>

class CommandLink {
public:
	virtual ~CommandLink() = default;

	virtual bool begin(const char* deviceName) = 0;   // start advertising / listening
	virtual bool isConnected() = 0;                   // is a phone connected right now?
	// Copies the next complete line (without the newline) into 'out'. false = nothing waiting.
	virtual bool readLine(char* out, size_t maxLength) = 0;
	virtual void send(const char* text) = 0;          // text to the phone
};
