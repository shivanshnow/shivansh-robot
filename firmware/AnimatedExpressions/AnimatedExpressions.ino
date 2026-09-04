/**
 * ============================================================================
 * SHIVANSH MECHA OS • Autonomous Robotics Platform
 * Embedded Systems Architecture & Micro-Graphics
 * ============================================================================
 * File: AnimatedExpressions.ino
 * Author: Pilot Shivansh & Antigravity AI Pair-Programmer
 * Target Microcontroller: Atmel ATmega328P @ 16.0 MHz (Arduino Uno / Nano)
 * Platform: Kidsbits Multi-Purpose Coding Robot (Model KD0003)
 *
 * Description:
 *   Autonomous Animated Emoji & Expression Engine for 8x8 LED Dot Matrix
 *   driven by the Holtek HT16K33 RAM controller over the I2C bus (Addr 0x70).
 *
 * Design Highlights:
 *   - Desk-Safe Architecture: Motors are permanently initialized to 0 RPM.
 *   - PROGMEM Flash Bitmaps: Frame buffers stored in flash to conserve SRAM.
 *   - Synchronous Acoustic Feedback: Synthesizes matching frequency chirps
 *     and heartbeats via dual resonant piezo transducers.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>

/* ============================================================================
 * Hardware Constants & Memory Definitions
 * ============================================================================
 */

#define MATRIX_I2C_ADDR 0x70
const uint8_t kPinBuzzer = 6;

// Motor Pin Protection (Desk Safety)
const uint8_t kPinMotorLeftDir  = 8;
const uint8_t kPinMotorLeftPwm  = 9;
const uint8_t kPinMotorRightPwm = 10;
const uint8_t kPinMotorRightDir = A1;

/* ============================================================================
 * 8x8 Frame Bitmaps (PROGMEM)
 * ============================================================================
 */

// Happy Face & Natural Eye Blink
const uint8_t PROGMEM kFaceHappy[8] = {
  0b00111100, 0b01000010, 0b10100101, 0b10000001,
  0b10100101, 0b10011001, 0b01000010, 0b00111100
};

const uint8_t PROGMEM kFaceBlink[8] = {
  0b00111100, 0b01000010, 0b10000001, 0b10111101,
  0b10100101, 0b10011001, 0b01000010, 0b00111100
};

// Pulsing Beating Heart Frames
const uint8_t PROGMEM kHeartBig[8] = {
  0b00000000, 0b01100110, 0b11111111, 0b11111111,
  0b01111110, 0b00111100, 0b00011000, 0b00000000
};

const uint8_t PROGMEM kHeartSmall[8] = {
  0b00000000, 0b00000000, 0b00100100, 0b01111110,
  0b00111100, 0b00011000, 0b00000000, 0b00000000
};

// Playful Wink Face
const uint8_t PROGMEM kFaceWink[8] = {
  0b00111100, 0b01000010, 0b10100001, 0b10001101,
  0b10100101, 0b10011001, 0b01000010, 0b00111100
};

// Surprised / Wide-Eyed Emoji
const uint8_t PROGMEM kFaceSurprised[8] = {
  0b00111100, 0b01100110, 0b01100110, 0b00000000,
  0b00011000, 0b00100100, 0b00011000, 0b00111100
};

// Cool Sunglasses Emoji
const uint8_t PROGMEM kFaceCool[8] = {
  0b00111100, 0b01111110, 0b11111111, 0b10111101,
  0b10000001, 0b10011101, 0b01000010, 0b00111100
};

/* ============================================================================
 * Function Prototypes
 * ============================================================================
 */
void matrixInit();
void displayFrame(const uint8_t* bitmapProgmem);
void clearMatrix();
void animateHappyBlink();
void animateHeartbeat();
void animateWink();
void animateSurprised();
void animateCoolShades();

/* ============================================================================
 * Main Setup & Dispatcher Loop
 * ============================================================================
 */

void setup() {
  Serial.begin(9600);
  pinMode(kPinBuzzer, OUTPUT);
  
  // Enforce desk-safe motor lockout
  pinMode(kPinMotorLeftDir, OUTPUT);
  pinMode(kPinMotorLeftPwm, OUTPUT);
  pinMode(kPinMotorRightPwm, OUTPUT);
  pinMode(kPinMotorRightDir, OUTPUT);
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, LOW);
  analogWrite(kPinMotorLeftPwm, 0);
  analogWrite(kPinMotorRightPwm, 0);

  matrixInit();
  clearMatrix();

  Serial.println(F("===================================================="));
  Serial.println(F(" SHIVANSH MECHA OS: Animated Expressions LED Matrix Show  "));
  Serial.println(F("===================================================="));
}

void loop() {
  Serial.println(F("[Expression] Happy & Natural Blinking..."));
  animateHappyBlink();

  Serial.println(F("[Expression] Playful Wink..."));
  animateWink();

  Serial.println(F("[Expression] Synchronized Heartbeat..."));
  animateHeartbeat();

  Serial.println(F("[Expression] Surprised Face..."));
  animateSurprised();

  Serial.println(F("[Expression] Cool Sunglasses Face..."));
  animateCoolShades();
}

/* ============================================================================
 * Low-Level HT16K33 Hardware Driver
 * ============================================================================
 */

void matrixInit() {
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  
  // Turn on system oscillator
  Wire.beginTransmission(MATRIX_I2C_ADDR);
  Wire.write(0x21);
  Wire.endTransmission();

  // Set maximum display brightness
  Wire.beginTransmission(MATRIX_I2C_ADDR);
  Wire.write(0xEF);
  Wire.endTransmission();

  // Activate display, no hardware blinking
  Wire.beginTransmission(MATRIX_I2C_ADDR);
  Wire.write(0x81);
  Wire.endTransmission();
}

void displayFrame(const uint8_t* bitmapProgmem) {
  Wire.beginTransmission(MATRIX_I2C_ADDR);
  Wire.write(0x00);

  for (uint8_t row = 0; row < 8; row++) {
    uint8_t rowData = pgm_read_byte(&bitmapProgmem[row]);
    uint8_t mappedByte = 0;
    
    // Reverse bit sequence for column mapping
    for (int b = 0; b < 8; b++) {
      if (rowData & (1 << b)) {
        mappedByte |= (1 << (7 - b));
      }
    }
    Wire.write(mappedByte);
    Wire.write(0x00);
  }
  Wire.endTransmission();
}

void clearMatrix() {
  Wire.beginTransmission(MATRIX_I2C_ADDR);
  Wire.write(0x00);
  for (uint8_t i = 0; i < 16; i++) {
    Wire.write(0x00);
  }
  Wire.endTransmission();
}

/* ============================================================================
 * Animation Sequences
 * ============================================================================
 */

void animateHappyBlink() {
  displayFrame(kFaceHappy);
  delay(1600);
  
  // Single Blink
  displayFrame(kFaceBlink);
  delay(180);
  displayFrame(kFaceHappy);
  delay(1200);

  // Double Blink
  displayFrame(kFaceBlink);
  delay(120);
  displayFrame(kFaceHappy);
  delay(120);
  displayFrame(kFaceBlink);
  delay(140);
  displayFrame(kFaceHappy);
  delay(1500);
}

void animateHeartbeat() {
  for (int i = 0; i < 4; i++) {
    displayFrame(kHeartSmall);
    delay(200);
    
    // Lub pulse
    displayFrame(kHeartBig);
    tone(kPinBuzzer, 520, 80);
    delay(150);
    
    displayFrame(kHeartSmall);
    delay(100);
    
    // Dub pulse
    displayFrame(kHeartBig);
    tone(kPinBuzzer, 580, 80);
    delay(200);
    
    displayFrame(kHeartSmall);
    delay(400);
  }
}

void animateWink() {
  displayFrame(kFaceHappy);
  delay(800);
  displayFrame(kFaceWink);
  tone(kPinBuzzer, 1200, 100);
  delay(700);
  displayFrame(kFaceHappy);
  delay(1000);
}

void animateSurprised() {
  displayFrame(kFaceSurprised);
  tone(kPinBuzzer, 900, 150);
  delay(200);
  tone(kPinBuzzer, 1400, 200);
  delay(1800);
}

void animateCoolShades() {
  displayFrame(kFaceCool);
  delay(2200);
}
