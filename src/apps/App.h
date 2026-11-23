/*****************
App.h
Self-Balancing Robot

Each program we can run (test_imu, test_motors, ... and later the full robot) is
an "app" that defines appSetup() and appLoop(). The platform main file creates
the right hardware (real or simulated) and calls them, the same way Arduino
calls setup() and loop().

	ESP32:      src/main/esp32_main.cpp  ->  Esp32Hardware + BleUartLink
	Simulator:  src/main/sim_main.cpp    ->  SimHardware   + ScriptedLink

Which app gets built is chosen in platformio.ini (ESP32) or CMakeLists.txt (simulator).
*****************/

#pragma once
#include "comms/CommandLink.h"
#include "hal/HardwareInterface.h"

struct Platform {
	HardwareInterface& hw;
	CommandLink& bluetooth;
};

void appSetup(Platform& platform);
void appLoop(Platform& platform);

#ifdef SBR_SIM
class SimHardware;
class ScriptedLink;
// Each app also describes what happens in the simulated world while it runs
// (who holds the robot, where the wall is, what the phone sends, how long to run).
void simConfigure(SimHardware& sim, ScriptedLink& phone);
#endif
