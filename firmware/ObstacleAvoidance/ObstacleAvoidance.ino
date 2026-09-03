/**
 * ============================================================================
 * SHIVANSH MECHA OS • Autonomous Robotics Platform
 * Embedded Systems Architecture & Autonomous Robotics
 * ============================================================================
 * File: ObstacleAvoidance.ino
 * Author: Pilot Shivansh & Antigravity AI Pair-Programmer
 * Target Microcontroller: Atmel ATmega328P @ 16.0 MHz (Arduino Uno / Nano)
 * Platform: Kidsbits Multi-Purpose Coding Robot (Model KD0003)
 *
 * Description:
 *   Autonomous reactive obstacle avoidance firmware using ultrasonic acoustic
 *   ranging and differential look-around spatial path clearance calculation.
 *
 * Safety Lockout:
 *   Boots into Safe Standby mode (motors 0 RPM) until armed by the user.
 * ============================================================================
 */

#include <Arduino.h>

/* ============================================================================
 * Hardware Pin Definitions
 * ============================================================================
 */
const uint8_t kPinUltrasonicTrig = 2;
const uint8_t kPinUltrasonicEcho = 3;
const uint8_t kPinBuzzer1        = 6;
const uint8_t kPinBuzzer2        = 7;
const uint8_t kPinMotorLeftDir   = 8;
const uint8_t kPinMotorLeftPwm   = 9;
const uint8_t kPinMotorRightPwm  = 10;
const uint8_t kPinMotorRightDir  = A1;
const uint8_t kPinStatusLed      = LED_BUILTIN;

/* ============================================================================
 * Kinematic Setpoints & Safety Thresholds
 * ============================================================================
 */
const uint8_t kSpeedCruise     = 160;  // Normal translation PWM
const uint8_t kSpeedTurn       = 140;  // Pivot turn PWM
const float   kSafeDistanceCm  = 25.0f; // Look-around threshold
const float   kCriticalDistCm  = 12.0f; // Emergency reverse threshold

bool gIsArmed = false; // Desk-safe arming state

/* ============================================================================
 * Function Prototypes
 * ============================================================================
 */
void setupHardware();
void haltMotors();
void driveForward(uint8_t speed);
void driveBackward(uint8_t speed);
void pivotLeft(uint8_t speed);
void pivotRight(uint8_t speed);
float getDistanceCm();
void playTone(unsigned int freqHz, unsigned long durationMs);

/* ============================================================================
 * Main Setup & Dispatcher Loop
 * ============================================================================
 */

void setup() {
  Serial.begin(9600);
  setupHardware();

  Serial.println(F("========================================================"));
  Serial.println(F(" SHIVANSH MECHA OS: Autonomous Obstacle Avoidance Engine      "));
  Serial.println(F(" Send '1' or 'G' to ARM • Send 'S' for EMERGENCY BRAKE  "));
  Serial.println(F("========================================================"));
}

void loop() {
  // Check for arming/emergency brake command over Serial
  if (Serial.available() > 0) {
    char ch = Serial.read();
    // 🛑 'S' / 's' is UNCONDITIONALLY EMERGENCY STOP across all sketches!
    if (ch == 's' || ch == 'S') {
      gIsArmed = false;
      haltMotors();
      Serial.println(F("[STOP] >>> EMERGENCY BRAKE ENGAGED! Standby."));
      playTone(400, 150);
    } else if (ch == '1' || ch == 'g' || ch == 'G') {
      gIsArmed = true;
      haltMotors();
      Serial.println(F("[SYSTEM] >>> Robot ARMED! Starting Navigation..."));
      playTone(1000, 100);
      delay(100);
      playTone(1500, 150);
    }
  }

  // Standby mode: do not move
  if (!gIsArmed) {
    haltMotors();
    digitalWrite(kPinStatusLed, (millis() / 500) % 2);
    delay(100);
    return;
  }

  // Autonomous Navigation Loop
  float dist = getDistanceCm();

  // Critical Zone: Emergency Reverse & Pivot
  if (dist > 0.0f && dist < kCriticalDistCm) {
    haltMotors();
    playTone(450, 80);
    driveBackward(kSpeedCruise);
    delay(300);
    haltMotors();
    pivotRight(kSpeedTurn);
    delay(400);
    haltMotors();
  }
  // Warning Zone: Look-Around Spatial Scan
  else if (dist > 0.0f && dist < kSafeDistanceCm) {
    haltMotors();
    playTone(650, 60);

    // Look Left
    pivotLeft(kSpeedTurn);
    delay(350);
    haltMotors();
    delay(100);
    float leftDist = getDistanceCm();

    // Look Right
    pivotRight(kSpeedTurn);
    delay(700);
    haltMotors();
    delay(100);
    float rightDist = getDistanceCm();

    // Pivot toward free corridor
    if (leftDist > rightDist && leftDist > kSafeDistanceCm) {
      pivotLeft(kSpeedTurn);
      delay(700);
      haltMotors();
    }
  }
  // Clear Corridor: Forward Cruise
  else {
    driveForward(kSpeedCruise);
    delay(40);
  }
}

/* ============================================================================
 * Low-Level Hardware Drivers
 * ============================================================================
 */

void setupHardware() {
  pinMode(kPinMotorLeftDir, OUTPUT);
  pinMode(kPinMotorLeftPwm, OUTPUT);
  pinMode(kPinMotorRightDir, OUTPUT);
  pinMode(kPinMotorRightPwm, OUTPUT);

  pinMode(kPinUltrasonicTrig, OUTPUT);
  pinMode(kPinUltrasonicEcho, INPUT);

  pinMode(kPinBuzzer1, OUTPUT);
  pinMode(kPinBuzzer2, OUTPUT);
  pinMode(kPinStatusLed, OUTPUT);

  haltMotors();
}

void haltMotors() {
  analogWrite(kPinMotorLeftPwm, 0);
  analogWrite(kPinMotorRightPwm, 0);
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, LOW);
}

void driveForward(uint8_t speed) {
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, LOW);
  analogWrite(kPinMotorLeftPwm, speed);
  analogWrite(kPinMotorRightPwm, speed);
}

void driveBackward(uint8_t speed) {
  digitalWrite(kPinMotorLeftDir, HIGH);
  digitalWrite(kPinMotorRightDir, HIGH);
  analogWrite(kPinMotorLeftPwm, speed);
  analogWrite(kPinMotorRightPwm, speed);
}

void pivotLeft(uint8_t speed) {
  digitalWrite(kPinMotorLeftDir, HIGH);
  digitalWrite(kPinMotorRightDir, LOW);
  analogWrite(kPinMotorLeftPwm, speed);
  analogWrite(kPinMotorRightPwm, speed);
}

void pivotRight(uint8_t speed) {
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, HIGH);
  analogWrite(kPinMotorLeftPwm, speed);
  analogWrite(kPinMotorRightPwm, speed);
}

float getDistanceCm() {
  digitalWrite(kPinUltrasonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(kPinUltrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(kPinUltrasonicTrig, LOW);

  unsigned long duration = pulseIn(kPinUltrasonicEcho, HIGH, 25000UL);
  if (duration == 0UL) return 0.0f; // 🛑 0.0f signifies timeout/disconnect fault
  return (float)duration / 58.0f;
}

void playTone(unsigned int freqHz, unsigned long durationMs) {
  tone(kPinBuzzer1, freqHz);
  delay(durationMs);
  noTone(kPinBuzzer1);
}
