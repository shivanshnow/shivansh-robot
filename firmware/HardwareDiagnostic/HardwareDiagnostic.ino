/**
 * ============================================================================
 * SHIVANSH MECHA OS • Autonomous Robotics Platform
 * Embedded Systems Architecture & Hardware Verification
 * ============================================================================
 * File: HardwareDiagnostic.ino
 * Author: Pilot Shivansh & Antigravity AI Pair-Programmer
 * Target Microcontroller: Atmel ATmega328P @ 16.0 MHz (Arduino Uno / Nano)
 * Platform: Kidsbits Multi-Purpose Coding Robot (Model KD0003)
 *
 * Description:
 *   Comprehensive Hardware Diagnostic & Analog Voltage Telemetry Suite.
 *   Validates I2C communication with the HT16K33 matrix driver, captures
 *   acoustic time-of-flight distances, reads optical phototransistors, and
 *   monitors internal ADC analog rails (A0 - A7).
 *
 * Safety Lockout:
 *   All motor outputs are locked at 0 RPM to prevent bench movement.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>

/* ============================================================================
 * Hardware Pin Definitions
 * ============================================================================
 */
const uint8_t kPinUltrasonicTrig = 2;
const uint8_t kPinUltrasonicEcho = 3;
const uint8_t kPinLineLeft       = 4;
const uint8_t kPinLineRight      = 5;

// Motor Driver Outputs
const uint8_t kPinMotorLeftDir   = 8;
const uint8_t kPinMotorLeftPwm   = 9;
const uint8_t kPinMotorRightPwm  = 10;
const uint8_t kPinMotorRightDir  = A1;

/* ============================================================================
 * Function Prototypes
 * ============================================================================
 */
void scanI2CDevices();
float readUltrasonic();

/* ============================================================================
 * Main Setup & Dispatcher Loop
 * ============================================================================
 */

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setWireTimeout(3000, true);

  // Enforce desk-safe motor lockout
  pinMode(kPinMotorLeftDir, OUTPUT);
  pinMode(kPinMotorLeftPwm, OUTPUT);
  pinMode(kPinMotorRightPwm, OUTPUT);
  pinMode(kPinMotorRightDir, OUTPUT);
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, LOW);
  analogWrite(kPinMotorLeftPwm, 0);
  analogWrite(kPinMotorRightPwm, 0);

  pinMode(kPinUltrasonicTrig, OUTPUT);
  pinMode(kPinUltrasonicEcho, INPUT);
  pinMode(kPinLineLeft, INPUT);
  pinMode(kPinLineRight, INPUT);

  Serial.println(F("======================================================"));
  Serial.println(F(" SHIVANSH MECHA OS: Kidsbits KD0003 Hardware Diagnostic OS  "));
  Serial.println(F("======================================================"));

  scanI2CDevices();
}

void loop() {
  Serial.println(F("\n================= SENSOR & PIN READOUT ================="));

  // 1. Acoustic Distance Telemetry
  float distance = readUltrasonic();
  Serial.print(F("  Ultrasonic Distance : "));
  if (distance < 0) Serial.println(F("NO ECHO (Check Sensor)"));
  else {
    Serial.print(distance, 1);
    Serial.println(F(" cm"));
  }

  // 2. Optical Infrared Line Sensors
  int lineL = digitalRead(kPinLineLeft);
  int lineR = digitalRead(kPinLineRight);
  Serial.print(F("  Line Tracking Left  : "));
  Serial.println(lineL == 0 ? F("BLACK (0)") : F("WHITE (1)"));
  Serial.print(F("  Line Tracking Right : "));
  Serial.println(lineR == 0 ? F("BLACK (0)") : F("WHITE (1)"));

  // 3. ADC Analog Rail Voltages (A0 - A7)
  Serial.println(F("  Analog Pin Voltages :"));
  for (int pin = 0; pin <= 7; pin++) {
    int raw = analogRead(pin);
    float voltage = raw * (5.0f / 1023.0f);
    Serial.print(F("    A"));
    Serial.print(pin);
    Serial.print(F(" : "));
    Serial.print(voltage, 2);
    Serial.print(F(" V (Raw: "));
    Serial.print(raw);
    Serial.println(F(")"));
  }

  Serial.println(F("========================================================"));
  delay(2000);
}

/* ============================================================================
 * Diagnostic Helper Routines
 * ============================================================================
 */

/**
 * Function: scanI2CDevices
 * Description: Scans all 127 7-bit I2C addresses, transmitting an ACK query to
 *              detect responsive peripherals (HT16K33, EEPROM, sensors).
 */
void scanI2CDevices() {
  byte error, address;
  int nDevices = 0;
  Serial.println(F("\n--- [1] Scanning I2C Bus ---"));
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print(F("  [✔] Found I2C Device at 0x"));
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      if (address == 0x70) Serial.println(F(" (HT16K33 8x8 LED Matrix Display)"));
      else Serial.println();
      nDevices++;
    }
  }
  if (nDevices == 0) Serial.println(F("  [-] No I2C devices found."));
}

/**
 * Function: readUltrasonic
 * Description: Emits a 10µs ultrasonic burst and calculates distance in cm.
 * Returns:
 *   Distance in cm (or -1.0 if timeout occurs).
 */
float readUltrasonic() {
  digitalWrite(kPinUltrasonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(kPinUltrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(kPinUltrasonicTrig, LOW);
  unsigned long duration = pulseIn(kPinUltrasonicEcho, HIGH, 30000UL);
  if (duration == 0) return -1.0f;
  return duration / 58.0f;
}
