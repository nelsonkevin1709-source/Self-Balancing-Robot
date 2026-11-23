#pragma once
// =====================================================================
//  config.h - the ONE place for pin numbers and tuning values.
//  Rewired something? Want to tweak behaviour? Change it here.
// =====================================================================

// ---------------------------------------------------------------------
//  BRING-UP STAGE - raise this number as you finish each step.
//    1 = blink the LED only                      (does uploading work?)
//    2 = + read the MPU6050, print the angle     (does the sensor work?)
//    3 = + spin each motor one at a time         (do the motors work?)   WHEELS OFF THE GROUND
//    4 = + Bluetooth drives the motors directly  (does Bluetooth work?)  WHEELS OFF THE GROUND
//    5 = + balancing                             (the real robot)
// ---------------------------------------------------------------------
#ifndef BRINGUP_STAGE
#define BRINGUP_STAGE 3
#endif

// ---------------------------------------------------------------------
//  PINS (GPIO numbers, not the pin positions on the board)
//  These were chosen because they are safe to use on an ESP32 - avoid
//  GPIO 0, 2, 12, 15 (boot pins), 6-11 (flash) and 34-39 (input only).
// ---------------------------------------------------------------------
#define PIN_STATUS_LED   2    // onboard LED on most ESP32 dev boards

#define PIN_I2C_SDA      21   // MPU6050 SDA
#define PIN_I2C_SCL      22   // MPU6050 SCL

#define PIN_MOTOR_L_IN1  25   // DRV8833 AIN1 (or IN1)
#define PIN_MOTOR_L_IN2  26   // DRV8833 AIN2 (or IN2)
#define PIN_MOTOR_R_IN1  32   // DRV8833 BIN1 (or IN3)
#define PIN_MOTOR_R_IN2  33   // DRV8833 BIN2 (or IN4)
#define PIN_MOTOR_SLEEP  -1   // DRV8833 nSLEEP. -1 = you wired it to 3V3 yourself.
                              // Or pick a free GPIO (e.g. 27) and the code drives it HIGH.

// ---------------------------------------------------------------------
//  MOTORS
// ---------------------------------------------------------------------
#define MOTOR_INVERT_LEFT   false  // set to true if the left wheel spins the wrong way
#define MOTOR_INVERT_RIGHT  false  // set to true if the right wheel spins the wrong way
#define MOTOR_MIN_PERCENT   0.0f   // smallest power (%) that makes a wheel move. Stage 3 helps
                                   // you find it. Typical: 5-20. Compensates for friction.
#define MOTOR_PWM_HZ        20000  // 20 kHz is above hearing range, so no whine

// ---------------------------------------------------------------------
//  IMU (MPU6050) MOUNTING
//  Which sensor axis (printed on the little board) points the way the
//  robot drives?  1 = X axis, 2 = Y axis.
//  After stage 2 works: lean the robot FORWARD. The angle should become
//  POSITIVE. If it goes negative, change IMU_ANGLE_SIGN to -1.
// ---------------------------------------------------------------------
#define IMU_FORWARD_AXIS    1
#define IMU_ANGLE_SIGN      1
#define IMU_FILTER_ALPHA    0.98f  // 0.98 = trust the gyro 98%, accelerometer 2%

// ---------------------------------------------------------------------
//  BALANCE TUNING (stage 5) - these are only STARTING VALUES.
//  You can change them live over Bluetooth (see README), then copy the
//  numbers that work back into this file.
//   Kp = how hard to push back per degree of lean
//   Kd = damping - resists fast falling/wobbling
//   Ki = stops the robot slowly "running away" in one direction. Needed:
//        in simulation a robot with Ki = 0 drifts off and falls within seconds.
//   The values below stayed upright in a simulation of several small robots.
//   YOUR robot will need its own tuning - treat them as a starting point.
// ---------------------------------------------------------------------
#define BAL_KP                  12.0f
#define BAL_KI                  40.0f
#define BAL_KD                  2.0f
#define BAL_TRIM_DEG            0.0f   // angle at which the robot balances (centre-of-mass offset)
#define BAL_MAX_LEAN_DEG        2.0f   // how far to lean to drive forward/back (more = faster)
#define BAL_TURN_PERCENT        25.0f  // extra power difference between wheels when turning
#define BAL_FALL_ANGLE_DEG      30.0f  // beyond this the motors switch off
#define BAL_START_ANGLE_DEG     3.0f   // hold the robot within this angle of upright to start
#define BAL_START_HOLD_MS       500    // ...for this long
#define BAL_INTEGRAL_LIMIT      60.0f  // anti-windup limit for the Ki term (% power)
#define BAL_INTEGRAL_LEAK       0.15f  // how quickly the Ki term fades (per second); lets it slow to a stop

// ---------------------------------------------------------------------
//  LOOP SPEED, BLUETOOTH, TELEMETRY
// ---------------------------------------------------------------------
#define CONTROL_LOOP_US      5000   // 5000 us = 200 updates per second
#define BLE_DEVICE_NAME      "BalanceBot"
#define SERIAL_TELEMETRY_MS  50     // print angle info this often (stages 2 and 5). 0 = off
#define TEST_DRIVE_PERCENT   40.0f  // stage 4 only: motor power for the F/B/L/R test
