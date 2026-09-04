/**
 * ============================================================================
 * File: BluetoothControl.ino
 * Platform: Kidsbits Multi-Purpose Coding Robot (Model KD0003)
 * Target Microcontroller: Atmel ATmega328P @ 16.0 MHz (Arduino Uno)
 *
 * Description:
 *   Wireless Bluetooth Remote Control firmware with safety deadman watchdog.
 *   Receives directional characters over Bluetooth Serial (9600 baud):
 *     - 'F' : Drive Forward
 *     - 'B' : Drive Reverse
 *     - 'L' : Pivot Left
 *     - 'R' : Pivot Right
 *     - 'S' : Stop / Quiescent Brake
 *
 * 🛑 SAFETY DEADMAN WATCHDOG:
 *   A strict 350ms deadman timer halts all PWM motors unless directional
 *   commands are actively refreshed by the client. If Bluetooth drops,
 *   the phone locks, or the client crashes, the robot halts immediately.
 *
 * 🛑 LINK-LOSS & BANNER DRAIN:
 *   HM-10 BLE status banners (e.g. "+CONNECTED", "OK+LOST", "DISCONNECTED")
 *   and malformed byte streams are drained to newline and treated as an
 *   unconditional stop, preventing embedded letters from executing as motion.
 *
 * ⚠️ HARDWARE LIMITATIONS & CAVEATS:
 *   This is a lightweight direct RC firmware sketch. Unlike MultiModeSuperRobot.ino,
 *   this sketch does NOT monitor ultrasonic sonar or underside cliff sensors.
 *   Operate strictly with line-of-sight visual supervision on flat ground.
 *   For cliff edge detection and obstacle avoidance, use MultiModeSuperRobot.ino.
 *
 * IMPORTANT HARDWARE SWITCH:
 *   1. Upload this code with the physical "BT" switch turned OFF.
 *   2. Once uploaded, slide the "BT" switch ON to enable wireless control.
 * ============================================================================
 */

#include <Arduino.h>

// --- Hardware Pin Definitions ---
const uint8_t kPinMotorLeftDir  = 8;   // Left Motor Direction
const uint8_t kPinMotorLeftPwm  = 9;   // Left Motor PWM Speed
const uint8_t kPinMotorRightPwm = 10;  // Right Motor PWM Speed
const uint8_t kPinMotorRightDir = A1;  // Right Motor Direction
const uint8_t kPinStatusLed     = LED_BUILTIN;

const uint8_t kDriveSpeed = 160;
const uint8_t kTurnSpeed  = 130;

// Deadman Watchdog State (Halt if no refreshed motion command within 350ms)
unsigned long gLastMotionCmdTime = 0;
bool gMotorActive = false;
const unsigned long kDeadmanTimeoutMs = 350;

void haltMotors() {
  analogWrite(kPinMotorLeftPwm, 0);
  analogWrite(kPinMotorRightPwm, 0);
  digitalWrite(kPinMotorLeftDir, HIGH);
  digitalWrite(kPinMotorRightDir, HIGH);
  digitalWrite(kPinStatusLed, LOW);
  gMotorActive = false;
}

void driveForward() {
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, LOW);
  analogWrite(kPinMotorLeftPwm, kDriveSpeed);
  analogWrite(kPinMotorRightPwm, kDriveSpeed);
  digitalWrite(kPinStatusLed, HIGH);
}

void driveBackward() {
  digitalWrite(kPinMotorLeftDir, HIGH);
  digitalWrite(kPinMotorRightDir, HIGH);
  analogWrite(kPinMotorLeftPwm, kDriveSpeed);
  analogWrite(kPinMotorRightPwm, kDriveSpeed);
  digitalWrite(kPinStatusLed, HIGH);
}

void pivotLeft() {
  digitalWrite(kPinMotorLeftDir, HIGH);
  digitalWrite(kPinMotorRightDir, LOW);
  analogWrite(kPinMotorLeftPwm, kTurnSpeed);
  analogWrite(kPinMotorRightPwm, kTurnSpeed);
  digitalWrite(kPinStatusLed, HIGH);
}

void pivotRight() {
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, HIGH);
  analogWrite(kPinMotorLeftPwm, kTurnSpeed);
  analogWrite(kPinMotorRightPwm, kTurnSpeed);
  digitalWrite(kPinStatusLed, HIGH);
}

void setup() {
  Serial.begin(9600);

  pinMode(kPinMotorLeftDir, OUTPUT);
  pinMode(kPinMotorLeftPwm, OUTPUT);
  pinMode(kPinMotorRightDir, OUTPUT);
  pinMode(kPinMotorRightPwm, OUTPUT);
  pinMode(kPinStatusLed, OUTPUT);

  haltMotors();
  Serial.println(F("BLUETOOTH_CONTROLLER_READY"));
}

void loop() {
  // 🛑 Universal Deadman Watchdog: Halt immediately if link drops or client stops sending heartbeats
  if (gMotorActive && (millis() - gLastMotionCmdTime > kDeadmanTimeoutMs)) {
    haltMotors();
  }

  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // 🛑 0. Filter HM-10 BLE link-status banners and multi-char strings
    if (cmd == '+' || cmd == 'O' || cmd == 'o' || cmd == 'D' || cmd == 'C' || cmd == 'K') {
      haltMotors();
      unsigned long drainStart = millis();
      while (millis() - drainStart < 40) {
        if (Serial.available()) {
          drainStart = millis();
          char b = Serial.read();
          if (b == '\n' || b == '\r') break;
        }
      }
      return;
    }

    switch (cmd) {
      case 'F':
      case 'f':
        gLastMotionCmdTime = millis();
        gMotorActive = true;
        driveForward();
        break;

      case 'B':
      case 'b':
        gLastMotionCmdTime = millis();
        gMotorActive = true;
        driveBackward();
        break;

      case 'L':
      case 'l':
        gLastMotionCmdTime = millis();
        gMotorActive = true;
        pivotLeft();
        break;

      case 'R':
      case 'r':
        gLastMotionCmdTime = millis();
        gMotorActive = true;
        pivotRight();
        break;

      case 'S':
      case 's':
      default:
        haltMotors();
        break;
    }
  }
}
