# Self Balancing Robot

ESP32 + MPU6050 + DRV8833 + 2x yellow TT motors + HC-SR04, driven from a phone over Bluetooth.

Built the MTRN3500 way: every part is its own class with a documented header, and
**no class touches hardware directly**. They all go through `HardwareInterface`
(our `EmbeddedFunctions`), which has a real version and a simulated version.
So the same code runs on the robot and on your PC.

## Where we are

| Stage | What | Status |
|---|---|---|
| 1 | Talk to each component on its own (test programs) | **written, passes in the simulator, not yet run on hardware** |
| 2 | Connect them: tilt filter, balance PID, Bluetooth driving, module manager | next |
| 3 | Add-ons: ultrasonic obstacle stop, battery monitor, encoders... | later |

## Folder layout

```
include/config.h            every pin and constant (the only file to edit when rewiring)
src/hal/                    HardwareInterface = our EmbeddedFunctions
    HardwareInterface.h       what hardware can do (pure virtual)
    esp32/Esp32Hardware       real pins (Arduino functions)
    sim/SimHardware           simulated pins + fake chips
    sim/RobotPhysics          physics model of the robot (falls, drives, balances)
src/devices/                one class per component, like Galil
    MPU6050                   accelerometer + gyro (I2C)
    DcMotor                   one motor on the DRV8833 (make two)
    HCSR04                    ultrasonic distance
src/comms/                  CommandLink = text link to the phone
    esp32/BleUartLink         real Bluetooth Low Energy (built into the ESP32)
    sim/ScriptedLink          pretend phone for the simulator
src/apps/                   programs you can run
    test_imu / test_motors / test_ultrasonic / test_bluetooth
    TestReport                PASS / FAIL helper (like check() in GalilMain.cpp)
src/main/                   esp32_main.cpp and sim_main.cpp: create the hardware, run the app
```

| MTRN3500 | Here |
|---|---|
| `EmbeddedFunctions` (virtual functions) | `HardwareInterface` |
| `EmbeddedFunctions(true)` simulator | `SimHardware` |
| `Galil` class, given an `EmbeddedFunctions*` | `MPU6050`, `DcMotor`, `HCSR04`, each given a `HardwareInterface*` |
| `GalilMain.cpp` with `check()` PASS/FAIL | `src/apps/test_*.cpp` with `TestReport` |
| copy constructor rules | device classes delete copying: one object per physical part |

## Wiring for the tests

| Part | Pin | ESP32 |
|---|---|---|
| MPU6050 | VCC / GND | 3V3 / GND |
| | SDA / SCL | GPIO21 / GPIO22 |
| DRV8833 | IN1 / IN2 (left motor, AIN) | GPIO25 / GPIO26 |
| | IN3 / IN4 (right motor, BIN) | GPIO32 / GPIO33 |
| | nSLEEP (EEP) | 3V3 |
| | VM / GND | battery + / battery - **and** ESP32 GND |
| HC-SR04 | VCC / GND | 5V (VIN pin, on USB) / GND |
| | TRIG | GPIO27 |
| | ECHO | **1 kOhm** to GPIO35, plus **2 kOhm** from GPIO35 to GND |

All grounds must be connected together.

## Running a test on the robot (VS Code + PlatformIO)

1. Open this folder in VS Code.
2. In the blue bar at the bottom, click the environment name and pick `env:test_imu`
   (or `test_motors`, `test_ultrasonic`, `test_bluetooth`).
3. Plug in the ESP32 and click **Upload** (the -> arrow), then **Serial Monitor** (plug icon).
4. Press the ESP32's **EN** (reset) button to see the start of the test.

Suggested order: `test_imu` -> `test_ultrasonic` -> `test_bluetooth` -> `test_motors` (wheels in the air!).

The motor test asks questions: type `y` or `n` in the serial monitor.

## Running a test in the simulator (your PC)

Visual Studio 2022: **File > Open > Folder...**, pick this folder, wait for CMake to finish, choose
`sim_test_imu.exe` (etc.) from the start-up dropdown, press Run.

Or from a terminal with CMake + a compiler:
```
cmake -S . -B build
cmake --build build
./build/sim_test_imu        (Windows: build\Debug\sim_test_imu.exe)
```

Each test file ends with a `simConfigure()` section describing the simulated world:
someone holds the robot and rocks it, a hand moves in front of the sensor, a phone sends
messages, and so on. Time is simulated, so a 20 s test finishes instantly.

## Notes

- Bluetooth: use a BLE app (nRF Toolbox UART, Bluefruit Connect, or Serial Bluetooth Terminal on Android)
  and connect from inside the app, not the phone's Bluetooth settings. Works on iPhone and Android.
- The simulator's numbers (masses, motor torque) are estimates. It's for checking logic, not exact tuning.
