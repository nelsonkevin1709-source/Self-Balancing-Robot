/*****************
test_bluetooth.cpp  -  talk to your phone over Bluetooth (BLE)

Phone app (free): "nRF Toolbox" -> UART, or "Bluefruit Connect" -> UART (iPhone and Android),
or "Serial Bluetooth Terminal" (Android). Scan, connect to "BalanceBot".
You do NOT pair it in the phone's Bluetooth settings: connect from inside the app.

What it does:
	1. Starts advertising as "BalanceBot"
	2. Waits up to 60 s for your phone to connect
	3. Waits for you to send any message, and echoes it back
Then try the drive letters F B L R S: it tells you what each one will do later.
*****************/

#include "apps/App.h"
#include "apps/TestReport.h"
#include "config.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

namespace {
const char* describe(char c) {
	switch (toupper(static_cast<unsigned char>(c))) {
	case 'F': return "drive forward";
	case 'B': return "drive backward";
	case 'L': return "turn left";
	case 'R': return "turn right";
	case 'S': return "stop";
	case '?': return "status";
	default:  return "unknown command";
	}
}

bool waitFor(Platform& p, bool (*condition)(Platform&), uint32_t timeoutMs) {
	const uint32_t start = p.hw.millis();
	while (p.hw.millis() - start < timeoutMs) {
		if (condition(p)) return true;
		p.hw.delayMs(50);
	}
	return false;
}

char firstMessage[64] = "";
}  // namespace

void appSetup(Platform& p) {
	HardwareInterface& hw = p.hw;
	TestReport report(hw, "BLUETOOTH");

	report.check("Bluetooth started", p.bluetooth.begin(cfg::kBluetoothName), "name \"%s\"", cfg::kBluetoothName);
	hw.print("      open the phone app and connect to BalanceBot (waiting 60 s)...\n");
	const bool connected = waitFor(p, [](Platform& q) { return q.bluetooth.isConnected(); }, 60000);
	report.check("phone connected", connected);

	if (connected) {
		p.bluetooth.send("Hello from BalanceBot! Send me anything.\n");
		hw.print("      send any message from the phone (waiting 60 s)...\n");
		const bool got = waitFor(p, [](Platform& q) { return q.bluetooth.readLine(firstMessage, sizeof(firstMessage)); }, 60000);
		report.check("message received", got, "\"%s\"", firstMessage);
		if (got) {
			char reply[96];
			snprintf(reply, sizeof(reply), "got \"%s\"\n", firstMessage);
			p.bluetooth.send(reply);
		}
	}
	report.summary();
	hw.print("      now try F B L R S ? from the phone\n");
}

void appLoop(Platform& p) {
	char line[64];
	if (p.bluetooth.readLine(line, sizeof(line))) {
		char reply[96];
		snprintf(reply, sizeof(reply), "'%s' = %s\n", line, describe(line[0]));
		p.hw.print(reply);
		p.bluetooth.send(reply);
	}
	p.hw.delayMs(20);
}

#ifdef SBR_SIM
#include "comms/sim/ScriptedLink.h"
#include "hal/sim/SimHardware.h"

// Simulated world: a phone connects after 2 s and sends a few messages.
void simConfigure(SimHardware& sim, ScriptedLink& phone) {
	sim.setDuration(8.0);
	phone.connectAt(2.0);
	phone.sendAt(3.0, "hello");
	phone.sendAt(4.5, "F");
	phone.sendAt(5.0, "S");
	phone.sendAt(6.0, "?");
}
#endif
