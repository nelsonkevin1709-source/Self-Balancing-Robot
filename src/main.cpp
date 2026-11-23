// =====================================================================
//  BalanceBot - main.cpp
//  This file only CONNECTS the modules together. The real work lives in
//  lib/ (Imu, MotorDriver, Balance, Comms, ...). Pins and tuning numbers
//  live in include/config.h.
// =====================================================================
#include <Arduino.h>
#include <stdarg.h>

#include "config.h"

#include <Module.h>
#include <ModuleRegistry.h>
#include <StringPrint.h>
#include <Imu.h>
#include <MotorDriver.h>
#include <Balance.h>
#include <Comms.h>
// #include <MySensor.h>            // <- future sensors get included here

// ---------------------------------------------------------------------
//  1. Build the modules. (Only stores settings - nothing touches the hardware yet.)
// ---------------------------------------------------------------------
static ImuConfig makeImuConfig() {
  ImuConfig c;
  c.sdaPin = PIN_I2C_SDA;
  c.sclPin = PIN_I2C_SCL;
  c.forwardAxis = IMU_FORWARD_AXIS;
  c.angleSign = IMU_ANGLE_SIGN;
  c.filterAlpha = IMU_FILTER_ALPHA;
  return c;
}

static MotorConfig makeMotorConfig() {
  MotorConfig c;
  c.leftIn1 = PIN_MOTOR_L_IN1;
  c.leftIn2 = PIN_MOTOR_L_IN2;
  c.rightIn1 = PIN_MOTOR_R_IN1;
  c.rightIn2 = PIN_MOTOR_R_IN2;
  c.sleepPin = PIN_MOTOR_SLEEP;
  c.invertLeft = MOTOR_INVERT_LEFT;
  c.invertRight = MOTOR_INVERT_RIGHT;
  c.pwmHz = MOTOR_PWM_HZ;
  c.minPercent = MOTOR_MIN_PERCENT;
  return c;
}

static BalanceConfig makeBalanceConfig() {
  BalanceConfig c;
  c.kp = BAL_KP;
  c.ki = BAL_KI;
  c.kd = BAL_KD;
  c.trimDeg = BAL_TRIM_DEG;
  c.maxLeanDeg = BAL_MAX_LEAN_DEG;
  c.turnPercent = BAL_TURN_PERCENT;
  c.fallAngleDeg = BAL_FALL_ANGLE_DEG;
  c.startAngleDeg = BAL_START_ANGLE_DEG;
  c.startHoldMs = BAL_START_HOLD_MS;
  c.integralLimit = BAL_INTEGRAL_LIMIT;
  c.integralLeakPerSec = BAL_INTEGRAL_LEAK;
  return c;
}

Imu         imu(makeImuConfig(), CONTROL_LOOP_US);
MotorDriver motors(makeMotorConfig());
Balance     balance(imu, motors, makeBalanceConfig(), CONTROL_LOOP_US);
Comms       comms(BLE_DEVICE_NAME);
// MySensor mySensor(PIN_MY_SENSOR);   // <- future sensors get created here

ModuleRegistry registry;
static bool systemOk = true;

// ---------------------------------------------------------------------
//  2. Talking to the user: replies go to the USB serial monitor AND the phone.
// ---------------------------------------------------------------------
static void replyf(const char* fmt, ...) {
  char buf[96];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf) - 1, fmt, args);
  va_end(args);
  const size_t n = strlen(buf);
  buf[n] = '\n';
  buf[n + 1] = '\0';
  Serial.print(buf);
  comms.send(buf);
}

// Commands from the phone that are not single-letter drive commands.
// Send just the letter (e.g. "P") to read the current value.
static void onCommand(const char* line) {
  const char cmd = toupper((unsigned char)line[0]);
  const bool hasValue = line[1] != '\0';
  const float value = atof(line + 1);
  BalanceConfig& b = balance.config();

  switch (cmd) {
    case 'P': if (hasValue) b.kp = value;      replyf("Kp=%.3f", b.kp); break;
    case 'I': if (hasValue) b.ki = value;      replyf("Ki=%.3f", b.ki); break;
    case 'D': if (hasValue) b.kd = value;      replyf("Kd=%.3f", b.kd); break;
    case 'T': if (hasValue) b.trimDeg = value; replyf("trim=%.2f deg", b.trimDeg); break;
    case 'C':
      if (balance.isBalancing()) {
        replyf("can't calibrate while balancing");
      } else {
        replyf(imu.calibrateGyro() ? "gyro calibrated" : "calibration failed");
      }
      break;
    case '?': {
      StringPrint status;
      registry.reportAll(status);
      Serial.print(status.s);
      comms.send(status.s.c_str());
      break;
    }
    case 'H':
      replyf("F B L R S drive/stop");
      replyf("P I D <num> set gains");
      replyf("T <num> trim  C calibrate");
      replyf("? status");
      break;
    default:
      replyf("unknown: %s (H = help)", line);
      break;
  }
}

// ---------------------------------------------------------------------
//  3. Stage helpers
// ---------------------------------------------------------------------
// Stage 3: ramp each motor up in both directions so you can check wiring.
static void motorSelfTest() {
  Serial.println("\n*** MOTOR TEST - keep the wheels OFF THE GROUND! Starting in 5 seconds ***");
  delay(5000);
  for (int side = 0; side < 2; side++) {
    for (int dir = 1; dir >= -1; dir -= 2) {
      Serial.printf("%s wheel, %s - ramping up. Note the %% where it first moves.\n",
                    side == 0 ? "LEFT" : "RIGHT", dir > 0 ? "FORWARD" : "BACKWARD");
      for (int p = 0; p <= 60; p += 5) {
        Serial.printf("  %d%%\n", p);
        if (side == 0) motors.setLeft((float)(p * dir)); else motors.setRight((float)(p * dir));
        delay(400);
      }
      motors.stop();
      delay(600);
    }
  }
  Serial.println("Motor test finished. Reset the board to run it again.");
}

static void updateStatusLed() {
  static uint32_t lastToggle = 0;
  static bool on = false;
  if (BRINGUP_STAGE >= 5 && systemOk && balance.isBalancing()) {
    digitalWrite(PIN_STATUS_LED, HIGH);   // solid = actively balancing
    return;
  }
  const uint32_t now = millis();
  const uint32_t half = systemOk ? 500 : 100;   // slow blink = fine, fast blink = error
  if (now - lastToggle >= half) {
    lastToggle = now;
    on = !on;
    digitalWrite(PIN_STATUS_LED, on);
  }
}

static void printTelemetry() {
  if (SERIAL_TELEMETRY_MS == 0) return;
  if (BRINGUP_STAGE != 2 && BRINGUP_STAGE != 5) return;
  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last < SERIAL_TELEMETRY_MS) return;
  last = now;
  // "name:value" format draws nicely in the PlatformIO / Arduino serial plotter.
  Serial.printf("angle:%.2f rate:%.1f out:%.1f\n", imu.pitchDeg(), imu.pitchRateDps(),
                balance.lastOutput());
}

// ---------------------------------------------------------------------
//  4. setup() and loop()
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_STATUS_LED, OUTPUT);
  Serial.printf("\n=== BalanceBot starting - bring-up stage %d ===\n", BRINGUP_STAGE);

  // Register the modules that should run at this stage, in the order they should update.
  if (BRINGUP_STAGE >= 2) registry.add(imu);
  if (BRINGUP_STAGE >= 3) registry.add(motors);
  if (BRINGUP_STAGE >= 4) registry.add(comms);
  if (BRINGUP_STAGE >= 5) registry.add(balance);
  // >>> FUTURE SENSORS: register them here, e.g.   registry.add(mySensor);

  systemOk = registry.beginAll(Serial);
  if (!systemOk) {
    Serial.println("A module failed to start - read the messages above. Motors stay off.");
    return;
  }

  comms.setLineHandler(onCommand);
  if (BRINGUP_STAGE == 3) motorSelfTest();
  Serial.println("Ready.");
}

void loop() {
  updateStatusLed();

  if (!systemOk) {   // something failed at start-up: just blink fast
    static uint32_t lastMsg = 0;
    if (millis() - lastMsg > 3000) {
      lastMsg = millis();
      Serial.println("ERROR: a module failed to start (see the top of this log). Fix it and reset.");
    }
    return;
  }

  // Glue: phone input -> robot behaviour. This is where you'd also let a
  // future sensor influence the robot (e.g. cap the throttle near obstacles).
  if (BRINGUP_STAGE >= 5) {
    balance.setDriveInput(comms.throttle(), comms.turn());
  } else if (BRINGUP_STAGE == 4) {
    // Open loop test: prove Bluetooth -> motors works. No balancing!
    static uint32_t lastUs = 0;
    const uint32_t nowUs = micros();
    if (nowUs - lastUs >= 20000) {
      lastUs = nowUs;
      const float fwd = comms.throttle() * TEST_DRIVE_PERCENT;
      const float turn = comms.turn() * TEST_DRIVE_PERCENT;
      motors.setSpeeds(fwd + turn, fwd - turn);
    }
  }

  registry.updateAll(micros());
  printTelemetry();

  if (BRINGUP_STAGE == 1) {
    static uint32_t lastBeat = 0;
    if (millis() - lastBeat >= 1000) {
      lastBeat = millis();
      Serial.printf("alive for %lu s - upload works!\n", (unsigned long)(millis() / 1000));
    }
  }
}
