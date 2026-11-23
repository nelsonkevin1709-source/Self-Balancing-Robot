/*****************
esp32_main.cpp
Self-Balancing Robot (ESP32 only)

Creates the REAL hardware objects and runs whichever app platformio.ini selected.
You should not need to change this file.
*****************/

#include <Arduino.h>
#include "apps/App.h"
#include "comms/esp32/BleUartLink.h"
#include "config.h"
#include "hal/esp32/Esp32Hardware.h"

static Esp32Hardware hardware;
static BleUartLink bluetooth;
static Platform platform{ hardware, bluetooth };

void setup() {
	hardware.begin(cfg::kSerialBaud);
	appSetup(platform);
}

void loop() {
	appLoop(platform);
}
