# BalanceBot

A two-wheeled self-balancing robot driven from a phone over Bluetooth.

**Hardware:** ESP32 (WROOM-DA) · MPU6050 gyro/accelerometer · DRV8833 motor driver · 2 DC gear motors

## Project layout

```
platformio.ini      board + library versions (shared by everyone)
include/config.h    pins and tuning numbers - the ONE file you edit most
src/main.cpp        connects the modules together (kept short)
lib/
  RobotCore/        the Module interface every part follows
  Imu/              MPU6050 reading + lean-angle filter
  MotorDriver/      DRV8833 control
  Balance/          PID balancing controller
  Comms/            Bluetooth LE link to the phone
  Sensors/          SensorTemplate.h - copy it to add new sensors
```

Each part is a **Module** with four functions: `name()`, `begin()`, `update()`, `report()`.
`main.cpp` starts them all and calls `update()` at each module's own speed.

## Wiring (first draft - match it to `include/config.h`)

| MPU6050 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

| DRV8833 | Connect to |
|---|---|
| AIN1 (or IN1) | GPIO 25 |
| AIN2 (or IN2) | GPIO 26 |
| BIN1 (or IN3) | GPIO 32 |
| BIN2 (or IN4) | GPIO 33 |
| nSLEEP (SLP) | 3V3 (or a GPIO, then set `PIN_MOTOR_SLEEP`) |
| GND | ESP32 GND **and** battery − (all grounds joined) |
| VIN / VM | battery + (must suit the driver's 2.7-10.8 V range and your motors) |
| AOUT1, AOUT2 | left motor |
| BOUT1, BOUT2 | right motor |

While developing, power the ESP32 from USB and the motors from the battery, with the grounds connected.

## Getting started

1. Install **VS Code** and its **PlatformIO IDE** extension.
2. Clone this repo with GitHub Desktop, then in VS Code: *File → Open Folder* and pick the repo folder (the one containing `platformio.ini`). The first open downloads tools - give it a few minutes.
3. Plug in the ESP32 and click the **→ Upload** arrow in the bottom status bar. If it sits on `Connecting...`, hold the **BOOT** button on the board until it starts.
4. Click the **plug icon** (Serial Monitor) to see the robot's messages.

## Bring-up stages

Don't try to make it balance on day one. Work through the stages in `include/config.h` (`BRINGUP_STAGE`), uploading after each change. Only move on when a stage works.

| Stage | What it does | You should see |
|---|---|---|
| 1 | Blinks the LED | LED blinking, "alive" messages |
| 2 | Reads the MPU6050 | `angle:` numbers. Tilt the robot **forward**: angle goes **positive**. If not, set `IMU_ANGLE_SIGN -1` (or change `IMU_FORWARD_AXIS`) |
| 3 | Spins each motor | **Wheels off the ground!** Each wheel ramps forward then back. If one spins the wrong way, flip `MOTOR_INVERT_LEFT/RIGHT`. Note the % where a wheel starts moving and put it in `MOTOR_MIN_PERCENT` |
| 4 | Bluetooth drives motors | **Wheels off the ground!** Send `F`, `B`, `L`, `R`, `S` from the phone |
| 5 | Balancing | Hold it upright; after half a second the LED goes solid and it takes over |

## Bluetooth

The robot shows up as **BalanceBot** using the standard Nordic UART service, so no phone pairing is needed - just connect from an app. Any "BLE UART" / "Nordic UART" app works on iPhone and Android (for example nRF Toolbox → UART, which can also make buttons that send letters).

| Send | Does |
|---|---|
| `F` `B` `L` `R` | drive forward / back / left / right (they combine) |
| `S` | stop (clears forward and turn) |
| `P12.5` `I40` `D2` | set Kp / Ki / Kd live (send just `P` to read it) |
| `T-1.5` | set trim angle |
| `C` | recalibrate gyro (robot must be still, not balancing) |
| `?` | status of every module |
| `H` | help |

If the phone disconnects, the robot stops driving but keeps balancing.

## Tuning the balance (stage 5)

The starting values in `config.h` (Kp 12, Ki 40, Kd 2) kept a *simulated* small robot upright, but your real robot has its own weight, motors and wheels, so expect to tune. Change one thing at a time, using the Bluetooth commands so you don't have to re-upload:

1. Set `I0` (Ki off) and raise **Kp** until the robot fights falling but starts to oscillate, then back off a little.
2. Raise **Kd** to calm the wobble. Too much Kd makes it buzz or shake.
3. Adjust **trim** (`T`) if it keeps leaning one way.
4. Bring **Ki** back up (try 20-40). It stops the slow drift where the robot gradually runs away in one direction. In simulation, a robot with Ki = 0 always ran away eventually.

Live changes are lost on reset - copy the good numbers into `config.h`.

**Known limitation:** with only an IMU the robot cannot measure how fast it is rolling. Holding `F` makes it roll forward at a roughly steady speed, and after `S` it only slowly coasts down - tap the opposite direction (`B`) to brake. Adding **wheel encoders** later gives it real speed control; that is exactly the kind of module this layout is built for.

**Safety:** lay the robot on its side (more than 30° tilt) and the motors switch off. Keep fingers clear of the wheels, and test on a flat floor.

## Adding a new sensor

Copy `lib/Sensors/SensorTemplate.h` into a new `lib/<YourSensor>/` folder, fill in the four functions, then add two lines to `main.cpp` (create it, `registry.add(...)`). The balancing code never changes. Full steps are in the header comments of `SensorTemplate.h`.

## Working together on GitHub

- Pull before you start, commit small changes with clear messages ("Add stage 3 motor test"), push when it works.
- Avoid editing the same file at the same time. `config.h` is the likeliest conflict - agree who changes it.
- Never commit the `.pio/` folder (the `.gitignore` takes care of it).
