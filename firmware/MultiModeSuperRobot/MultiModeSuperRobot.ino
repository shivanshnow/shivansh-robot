/**
 * ============================================================================
 * MECHA OS 4.3 • S2-R2-D2 Autonomous Robotics Platform
 * Universal Astromech Acoustic Intelligence & Multi-Mode Studio
 * ============================================================================
 * Target: Kidsbits Multi-Purpose Coding Robot (Model KD0003 / ATmega328P)
 * Buzzer Output: Single-Timer Pin D6 (Frequency-Modulated Acoustic Chirps)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

#define EEPROM_MAGIC_ADDR  0
#define EEPROM_GEAR_ADDR   1
#define EEPROM_MUTE_ADDR   2
#define EEPROM_RUNS_ADDR   4
#define EEPROM_MAGIC_VAL   0xA5

/* ============================================================================
 * SECTION 1: Hardware Pinout Mapping
 * ============================================================================
 */
const uint8_t kPinUltrasonicTrig  = 2;   // 10µs Sonic Trigger Output
const uint8_t kPinUltrasonicEcho  = 3;   // Echo Return Input
const uint8_t kPinLineSensorLeft   = 4;  // Underside IR Sensor Left
const uint8_t kPinLineSensorRight  = 5;  // Underside IR Sensor Right
const uint8_t kPinBuzzer1          = 6;  // Piezo Buzzer 1 (Primary Audio Output)
const uint8_t kPinMotorLeftDir   = 8;    // Left Motor Phase
const uint8_t kPinMotorLeftPwm   = 9;    // Left Motor PWM (0-255)
const uint8_t kPinMotorRightPwm  = 10;   // Right Motor PWM (0-255)
const uint8_t kPinMotorRightDir  = A1;   // Right Motor Phase
const uint8_t kPinStatusLed      = LED_BUILTIN;

// Expansion Port Pins
const uint8_t kPinExpansionA2  = A2;
const uint8_t kPinExpansionA3  = A3;
const uint8_t kPinExpansionD12 = 12;
const uint8_t kPinExpansionA6  = A6;
const uint8_t kPinExpansionA7  = A7;

#define MATRIX_I2C_ADDR 0x70

/* ============================================================================
 * SECTION 2: State Machine Enumerations
 * ============================================================================
 */
enum RobotMode {
  MODE_STANDBY            = 0,
  MODE_OBSTACLE_AVOIDANCE = 1,
  MODE_LINE_TRACKING      = 2,
  MODE_BLUETOOTH_RC       = 3,
  MODE_LIGHT_SHOW         = 4,
  MODE_LIVING_PET         = 5,
  MODE_FLASHBANG_AMBUSH   = 6,
  MODE_CLIFF_DETECTION    = 7,
  MODE_AIR_SYNTHESIZER    = 8,
  MODE_APEX_SENTRY        = 9,
  MODE_COUNT              = 10
};

enum SpeedGear {
  GEAR_PRECISION = 1,  // 120 PWM (45%)
  GEAR_CRUISE    = 2,  // 180 PWM (70%)
  GEAR_TURBO     = 3   // 255 PWM (100%)
};

enum DroidEmotion {
  EMOTION_HAPPY,       // 2-4 rising trills & happy chirps
  EMOTION_CURIOUS,     // Questioning pitch glides & scanning tones
  EMOTION_RELIEVED,    // Descending sigh after braking from cliff/obstacle
  EMOTION_ALERT,       // Fast staccato startled reflex
  EMOTION_GREETING,    // Multi-syllable welcoming sentence
  EMOTION_SLEEPY,      // Soft, gentle breathing sigh
  EMOTION_RACING       // High-speed energetic pip
};

enum DialoguePhrase {
  PHRASE_ROGER_LETS_GO,     // "Okay, got it... let's roll!" (Rising dual-trill + happy chirp)
  PHRASE_AFFIRMATIVE,       // "Understood!" (Crisp double pip)
  PHRASE_BACKING_UP,        // "Watch out, backing up!" (Low warble questioning chirp)
  PHRASE_TURNING,           // "Banking left/right!" (Curved melodic frequency shift)
  PHRASE_GEAR_SHIFT,        // "Shifting gears!" (Smooth stepped harmonic climb)
  PHRASE_AMBIENT_MURMUR,    // Soft 2-syllable organic breathing murmur
  PHRASE_QUESTION,          // "What's next?" (Rising interrogative whistle)
  PHRASE_HESITATION,        // "Hmm... let me check" (Ambiguity deliberation)
  PHRASE_ECSTASY,           // "Astromech Bliss!" (Bubbling sparkling multi-octave arpeggio)
  PHRASE_R2D2_YES,          // "YES! Affirmative!" (4-tone rapid happy R2 chirp)
  PHRASE_GRUMPY,            // "Irritated complaint!" (Low raspy mechanical grunt)
  PHRASE_FATIGUED           // "Drowsy power-down sigh!" (400ms descending wheezing yawn)
};

volatile RobotMode gCurrentMode = MODE_BLUETOOTH_RC;
volatile SpeedGear gCurrentGear = GEAR_PRECISION;
volatile bool gMasterMute = false; // Default to Sound Enabled!

uint8_t gDriveSpeed = 120;
uint8_t gTurnSpeed  = 110;

// Safety Watchdog & Deadman Timer (400ms RC deadman, 1500ms autonomous keepalive)
unsigned long gLastMotionCmdTime = 0;
volatile unsigned long gLastKeepaliveTime = 0;
bool gMotorActive = false;
uint8_t gSonarGoodCount = 0;
bool gSonarFault = false;
uint8_t gLineSweepStep = 0;

// Filter State
float gFollowFilteredDist = 20.0f;
float gDistanceHistory[5] = {20.0f, 20.0f, 20.0f, 20.0f, 20.0f};
uint8_t gHistoryIdx = 0;
unsigned long gLastIdleChirpTime = 0;
bool gFollowLockAcquired = false;

// Pentatonic Scale for Air Synthesizer
const unsigned int kPentatonicScale[10] = {
  440, 494, 554, 659, 740, 880, 988, 1109, 1318, 1480
};

/* ============================================================================
 * SECTION 3: Calibrated 8x8 LED Matrix Bitmaps (PROGMEM)
 * ============================================================================
 */
// 🌟 8x8 Sparkle Burst for Ecstasy
const uint8_t PROGMEM kIconEcstasy[8] = {
  0b00011000, 0b10011001, 0b01011010, 0b00111100,
  0b00111100, 0b01011010, 0b10011001, 0b00011000
};

// 😠 8x8 Grumpy / Irritated Expression (> _ <)
const uint8_t PROGMEM kIconGrumpy[8] = {
  0b00000000, 0b10000001, 0b01000010, 0b00100100,
  0b00000000, 0b01111110, 0b00000000, 0b00000000
};

// 😴 8x8 Fatigued / Sleepy Expression (- _ -)
const uint8_t PROGMEM kIconFatigued[8] = {
  0b00000000, 0b00000000, 0b01111110, 0b00000000,
  0b00000000, 0b00111100, 0b00000000, 0b00000000
};

// ✅ 8x8 Eager Affirmative Smiling Eyes (^ _ ^)
const uint8_t PROGMEM kIconAffirmative[8] = {
  0b00000000, 0b00100100, 0b01000010, 0b00000000,
  0b10000001, 0b01111110, 0b00000000, 0b00000000
};

const uint8_t PROGMEM kIconStandby[8] = {
  0b00111100, 0b01000010, 0b10000001, 0b10111101,
  0b10100101, 0b10011001, 0b01000010, 0b00111100
};

const uint8_t PROGMEM kIconBatAvoid[8] = {
  0b00111100, 0b01000010, 0b10100101, 0b10000001,
  0b10100101, 0b10011001, 0b01000010, 0b00111100
};

const uint8_t PROGMEM kIconLineTrack[8] = {
  0b00011000, 0b00011000, 0b00111100, 0b00111100,
  0b01111110, 0b01111110, 0b11111111, 0b11111111
};

const uint8_t PROGMEM kIconBluetooth[8] = {
  0b00011000, 0b00110100, 0b01010010, 0b00111100,
  0b00111100, 0b01010010, 0b00110100, 0b00011000
};

const uint8_t PROGMEM kIconHeart[8] = {
  0b00000000, 0b01100110, 0b11111111, 0b11111111,
  0b01111110, 0b00111100, 0b00011000, 0b00000000
};

const uint8_t PROGMEM kIconFollowPuppy[8] = {
  0b01000010, 0b11000011, 0b11111111, 0b10111101,
  0b11111111, 0b01100110, 0b00111100, 0b00011000
};

const uint8_t PROGMEM kIconSentryEye[8] = {
  0b00111100, 0b01111110, 0b11011011, 0b11111111,
  0b11011011, 0b01111110, 0b00111100, 0b00000000
};

const uint8_t PROGMEM kIconSkullAlert[8] = {
  0b00111100, 0b01111110, 0b10100101, 0b11111111,
  0b01111110, 0b01011010, 0b01011010, 0b00000000
};

const uint8_t PROGMEM kIconCliffGuard[8] = {
  0b11111111, 0b10000001, 0b10111101, 0b10100101,
  0b10100101, 0b10111101, 0b10000001, 0b11111111
};

const uint8_t PROGMEM kIconCrown[8] = {
  0b10001001, 0b10101011, 0b11111111, 0b11111111,
  0b11111111, 0b01111110, 0b00111100, 0b00000000
};

// ♟️ Grandmaster Chess Knight Silhouette
const uint8_t PROGMEM kIconChessKnight[8] = {
  0b00011100, 0b00111110, 0b01101111, 0b00111110,
  0b00011110, 0b00111100, 0b01111110, 0b11111111
};

// Morse Code Visual Bitmaps
const uint8_t PROGMEM kIconMorseDot[8] = {
  0b00000000, 0b00000000, 0b00011000, 0b00111100,
  0b00111100, 0b00011000, 0b00000000, 0b00000000
};

const uint8_t PROGMEM kIconMorseDash[8] = {
  0b00000000, 0b00000000, 0b11111111, 0b11111111,
  0b11111111, 0b11111111, 0b00000000, 0b00000000
};

const uint8_t PROGMEM kIconSideWalk1[8] = {
  0b00011000, 0b00011100, 0b00001000, 0b00111100,
  0b00001000, 0b00010100, 0b00100010, 0b01000001
};

const uint8_t PROGMEM kIconSideWalk2[8] = {
  0b00011000, 0b00011100, 0b00001000, 0b00011000,
  0b00001000, 0b00001000, 0b00010100, 0b00010010
};

const uint8_t PROGMEM kIconQuietSleep[8] = {
  0b00000000, 0b00000000, 0b00100100, 0b00000000,
  0b00000000, 0b01000010, 0b00111100, 0b00000000
};

const uint8_t PROGMEM kArrowForward[8] = {
  0b00011000, 0b00111100, 0b01111110, 0b11011011,
  0b00011000, 0b00011000, 0b00011000, 0b00011000
};

const uint8_t PROGMEM kArrowBackward[8] = {
  0b00011000, 0b00011000, 0b00011000, 0b00011000,
  0b11011011, 0b01111110, 0b00111100, 0b00011000
};

const uint8_t PROGMEM kArrowLeft[8] = {
  0b00010000, 0b00110000, 0b01110000, 0b11111111,
  0b11111111, 0b01110000, 0b00110000, 0b00010000
};

const uint8_t PROGMEM kArrowRight[8] = {
  0b00001000, 0b00001100, 0b00001110, 0b11111111,
  0b11111111, 0b00001110, 0b00001100, 0b00001000
};

const uint8_t PROGMEM kIconBrake[8] = {
  0b11111111, 0b11000011, 0b10100101, 0b10011001,
  0b10011001, 0b10100101, 0b11000011, 0b11111111
};

/* ============================================================================
 * SECTION 4: Function Prototypes
 * ============================================================================
 */
void setupHardware();
void setMode(RobotMode newMode);
void setGear(SpeedGear newGear);
void checkControlInput();
void executeCurrentMode();

void matrixInit();
void displayBitmap(const uint8_t* bitmap);
void displayCustomBuffer(const uint8_t* customBuffer);
void displayOscilloscope(uint8_t noteIdx, uint8_t rhythmStep);
void clearMatrix();
void scrollTextAcrossMatrix(const char* text, uint8_t scrollSpeedMs);

// Master Audio HAL & R2-D2 Organic Synthesis Engine
void playTone(unsigned int freqHz, unsigned long durationMs);
void setBuzzerTone(unsigned int freqHz);
void stopAllAudio();
void chirpSweep(int startF, int endF, int step, int delayMs);
void chirpWarble(int f1, int f2, int reps, int durMs);
void talkAstromech(DroidEmotion emotion);
void talkDialogue(DialoguePhrase phrase);
void playDopplerRadarPing(float distanceCm);
void checkBiologicalRespiration();
void chirpAccelerate();
void chirpBrake();
void chirpVisionLock();
void chirpHappyPet();
void chirpAlert();
void chirpCurious();

void haltMotors();
void driveForward(uint8_t speed);
void driveBackward(uint8_t speed);
void pivotLeft(uint8_t speed);
void pivotRight(uint8_t speed);
float getDistanceCm();
float getFilteredDistance();
int readActivePhotocell();
bool checkPirMotionDetected();

// Master Studio Engines
void runStandbyMode();
void runObstacleMode();
void runLineTrackingMode();
void runBluetoothMode(char cmd);
void runLightShowMode();
void runLivingPetEngine();
void runFlashbangAmbushMode();
void runCliffDetectionMode();
void runSpatialAirSynthesizer();
void runApexSentryMode();

// Special Protocols & Morse Academy
void runTableSafeJoyGreeting();
void runKingShivanshProtocol();
void runSpinTheDroidRoulette();
void runKnightLPathManeuver();
void runSayingYesMood();
void runEcstasyMood();
void runGrumpyMood();
void runFatiguedMood();
void playMorseDit();
void playMorseDah();
void playMorseLetter(char letter);
void playMorsePattern(const char* pattern);

// Jukebox
void playStarWars();
void playR2D2Chirps();
void playSuperMario();
void playMissionImpossible();

/* ============================================================================
 * SECTION 5: Main Setup, Persistence & Loop
 * ============================================================================
 */
void loadEepromSettings() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC_VAL) {
    gMasterMute = (EEPROM.read(EEPROM_MUTE_ADDR) == 1);
  } else {
    // Initialize EEPROM default signature on first boot
    EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
    EEPROM.update(EEPROM_GEAR_ADDR, (uint8_t)GEAR_PRECISION);
    EEPROM.update(EEPROM_MUTE_ADDR, 0);
    uint16_t zeroRuns = 0;
    EEPROM.put(EEPROM_RUNS_ADDR, zeroRuns);
  }
  // 🛡️ Safety Boot Clamp (H5): Always boot to PRECISION on every power-up!
  gCurrentGear = GEAR_PRECISION;

  // Increment lifetime mission counter
  uint16_t missionCount = 0;
  EEPROM.get(EEPROM_RUNS_ADDR, missionCount);
  missionCount++;
  EEPROM.put(EEPROM_RUNS_ADDR, missionCount);
}

void saveEepromGear(SpeedGear g) {
  EEPROM.update(EEPROM_GEAR_ADDR, (uint8_t)g);
}

void saveEepromMute(bool m) {
  EEPROM.update(EEPROM_MUTE_ADDR, m ? 1 : 0);
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50); // Bounded 50ms stream timeout for framed commands (Prevents parser stall!)
  randomSeed(analogRead(A0) ^ analogRead(A2) ^ (uint16_t)micros()); // True hardware entropy
  setupHardware();
  loadEepromSettings();
  setGear(GEAR_PRECISION);
  setMode(MODE_BLUETOOTH_RC);
  gLastIdleChirpTime = millis();
  gLastKeepaliveTime = millis();
}

void loop() {
  checkControlInput();
  executeCurrentMode();
}

/* ============================================================================
 * SECTION 6: Hardware Setup & Mode Switching
 * ============================================================================
 */
void setupHardware() {
  pinMode(kPinMotorLeftDir, OUTPUT);
  pinMode(kPinMotorLeftPwm, OUTPUT);
  pinMode(kPinMotorRightDir, OUTPUT);
  pinMode(kPinMotorRightPwm, OUTPUT);

  pinMode(kPinUltrasonicTrig, OUTPUT);
  pinMode(kPinUltrasonicEcho, INPUT);

  pinMode(kPinLineSensorLeft, INPUT);
  pinMode(kPinLineSensorRight, INPUT);

  pinMode(kPinExpansionA2, INPUT);
  pinMode(kPinExpansionA3, INPUT);
  pinMode(kPinExpansionD12, INPUT); // Plain INPUT without internal pullup (prevents false PIR trigger)

  pinMode(kPinBuzzer1, OUTPUT);
  pinMode(kPinStatusLed, OUTPUT);

  haltMotors();
  stopAllAudio();
  matrixInit();
  clearMatrix();
}

void setGear(SpeedGear newGear) {
  gCurrentGear = newGear;
  saveEepromGear(newGear);
  switch (gCurrentGear) {
    case GEAR_PRECISION:
      gDriveSpeed = 120; gTurnSpeed = 110;
      chirpSweep(700, 1300, 100, 4); // Gentle precision chirp
      break;
    case GEAR_CRUISE:
      gDriveSpeed = 180; gTurnSpeed = 160;
      chirpSweep(900, 1800, 120, 3); // Zippy cruise chirp
      break;
    case GEAR_TURBO:
      gDriveSpeed = 255; gTurnSpeed = 255;
      chirpSweep(1000, 2600, 140, 3); // High-octane turbo sweep!
      break;
  }
}

void setMode(RobotMode newMode) {
  gCurrentMode = newMode;
  haltMotors();
  stopAllAudio();
  gMotorActive = false;
  gLastIdleChirpTime = millis();
  gLastKeepaliveTime = millis();
  gLineSweepStep = 0;

  if (newMode == MODE_LIVING_PET) {
    gFollowLockAcquired = false;
    for (uint8_t i = 0; i < 5; i++) gDistanceHistory[i] = 999.0f;
  }

  switch (gCurrentMode) {
    case MODE_STANDBY:            displayBitmap(kIconStandby); break;
    case MODE_OBSTACLE_AVOIDANCE: displayBitmap(kIconBatAvoid); talkAstromech(EMOTION_CURIOUS); break;
    case MODE_LINE_TRACKING:      displayBitmap(kIconLineTrack); talkAstromech(EMOTION_RACING); break;
    case MODE_BLUETOOTH_RC:       displayBitmap(kIconBluetooth); break;
    case MODE_LIGHT_SHOW:         displayBitmap(kIconHeart); break;
    case MODE_LIVING_PET:         displayBitmap(kIconStandby); break;
    case MODE_FLASHBANG_AMBUSH:   displayBitmap(kIconSentryEye); break;
    case MODE_CLIFF_DETECTION:    displayBitmap(kIconCliffGuard); talkAstromech(EMOTION_CURIOUS); break;
    case MODE_AIR_SYNTHESIZER:    displayBitmap(kIconStandby); break;
    case MODE_APEX_SENTRY:        displayBitmap(kIconQuietSleep); break;
  }
}

/* ============================================================================
 * SECTION 7: Universal Command Parser & Bluetooth Protocol
 * ============================================================================
 */
void checkControlInput() {
  while (Serial.available() > 0) {
    char ch = Serial.read();
    gLastIdleChirpTime = millis(); // Reset idle timer on any user touch

    // 0. FILTER ALL HM-10 BLE STATUS BANNERS (e.g. "+CONNECTED", "OK+CONN", "OK+LOST")
    if (ch == '+' || ch == 'O' || ch == 'o') {
      char bannerBuf[24];
      uint8_t bLen = 0;
      bannerBuf[bLen++] = ch;
      unsigned long bannerStart = millis();
      while (millis() - bannerStart < 35 && bLen < 23) {
        if (Serial.available()) {
          char b = Serial.read();
          if (b == '\n' || b == '\r') break;
          bannerBuf[bLen++] = b;
        }
      }
      bannerBuf[bLen] = '\0';

      // 🛑 CRITICAL LINK-LOSS INTERLOCK: If Bluetooth link died, HALT MOTORS IMMEDIATELY!
      if (strstr(bannerBuf, "LOST") != NULL || strstr(bannerBuf, "DISC") != NULL) {
        haltMotors();
        stopAllAudio();
        gCurrentMode = MODE_BLUETOOTH_RC;
        gMotorActive = false;
        displayBitmap(kIconBrake);
      }
      continue; // Swallowed banner completely — never triggers rogue motor or mood action!
    }

    // 1. EMERGENCY BRAKE
    if (ch == 'S' || ch == 's') {
      haltMotors();
      stopAllAudio();
      gCurrentMode = MODE_BLUETOOTH_RC;
      gMotorActive = false;
      gFollowLockAcquired = false;
      displayBitmap(kIconBrake);
      digitalWrite(kPinStatusLed, LOW);
      chirpBrake();
    }
    // 2. MASTER AUDIO MUTE CONTROL ('x' = Absolute Mute, 'X' = Absolute Unmute)
    else if (ch == 'x') {
      gMasterMute = true;
      saveEepromMute(true);
      stopAllAudio();
    }
    else if (ch == 'X') {
      gMasterMute = false;
      saveEepromMute(false);
      chirpSweep(800, 1600, 100, 4);
    }
    // 3. 3-Speed Gearbox (lowercase q, w, e)
    else if (ch == 'q') setGear(GEAR_PRECISION);
    else if (ch == 'w') setGear(GEAR_CRUISE);
    else if (ch == 'e') setGear(GEAR_TURBO);
    // 4. Directional Vectors (Active Deadman Watchdog Latch)
    else if (ch == 'F' || ch == 'f' || ch == 'B' || ch == 'b' || 
             ch == 'L' || ch == 'l' || ch == 'R' || ch == 'r') {
      gCurrentMode = MODE_BLUETOOTH_RC;
      gLastMotionCmdTime = millis();
      gMotorActive = true;
      runBluetoothMode(ch);
    }
    // 5. Operating Modes 0 - 9
    else if (ch >= '0' && ch <= '9') {
      setMode((RobotMode)(ch - '0'));
    }
    // 6. Table-Safe Stationary Joy Greeting ('G')
    else if (ch == 'G' || ch == 'g') {
      runTableSafeJoyGreeting();
    }
    // 7. Manual King Shivansh 360° Dance Protocol ('K')
    else if (ch == 'K' || ch == 'k') {
      runKingShivanshProtocol();
    }
    // 8. Spin-the-Droid Roulette ('U')
    else if (ch == 'U' || ch == 'u') {
      runSpinTheDroidRoulette();
    }
    // 9. Morse Code Academy Broadcaster ('T' + letter/dit/dah + '\n')
    else if (ch == 'T' || ch == 't') {
      gLastKeepaliveTime = millis();
      char tBuf[8];
      size_t n = Serial.readBytesUntil('\n', tBuf, sizeof(tBuf) - 1);
      tBuf[n] = '\0';
      if (n > 0) {
        char letter = tBuf[0];
        if (letter == '.') playMorseDit();
        else if (letter == '-') playMorseDah();
        else playMorseLetter(letter);
      }
    }
    // 10. Chiptune Jukebox Payloads ('J' + 1-4 + '\n') (Guarded Payload Isolation)
    else if (ch == 'J' || ch == 'j') {
      gLastKeepaliveTime = millis();
      haltMotors();
      gMotorActive = false;
      char jBuf[8];
      size_t n = Serial.readBytesUntil('\n', jBuf, sizeof(jBuf) - 1);
      jBuf[n] = '\0';
      if (n > 0) {
        char song = jBuf[0];
        if (song == '1') playStarWars();
        else if (song == '2') playR2D2Chirps();
        else if (song == '3') playSuperMario();
        else if (song == '4') playMissionImpossible();
      }
    }
    // 11. Live 8x8 Pixel Art Streaming ('M' + 16 hex chars + '\n')
    else if (ch == 'M' || ch == 'm') {
      gLastKeepaliveTime = millis();
      char mBuf[24];
      size_t n = Serial.readBytesUntil('\n', mBuf, sizeof(mBuf) - 1);
      mBuf[n] = '\0';
      uint8_t customBuf[8];
      bool valid = false;
      if (n == 16) {
        valid = true;
        for (int i = 0; i < 8; i++) {
          char hCh = mBuf[i * 2];
          char lCh = mBuf[i * 2 + 1];
          int h = (hCh >= '0' && hCh <= '9') ? (hCh - '0') :
                  (hCh >= 'A' && hCh <= 'F') ? (hCh - 'A' + 10) :
                  (hCh >= 'a' && hCh <= 'f') ? (hCh - 'a' + 10) : -1;
          int l = (lCh >= '0' && lCh <= '9') ? (lCh - '0') :
                  (lCh >= 'A' && lCh <= 'F') ? (lCh - 'A' + 10) :
                  (lCh >= 'a' && lCh <= 'f') ? (lCh - 'a' + 10) : -1;
          if (h < 0 || l < 0) { valid = false; break; }
          customBuf[i] = (uint8_t)((h << 4) | l);
        }
      } else if (n == 8) {
        for (int i = 0; i < 8; i++) customBuf[i] = (uint8_t)mBuf[i];
        valid = true;
      }
      if (valid) {
        displayCustomBuffer(customBuf);
        if (!gMasterMute) {
          tone(kPinBuzzer1, random(1400, 2200)); delay(12);
          noTone(kPinBuzzer1);
        }
      }
    }
    // 12. Sonic Horn (Dual R2-D2 Astromech Chirp Blast!)
    else if (ch == 'h' || ch == 'H') {
      gLastKeepaliveTime = millis();
      chirpSweep(1800, 1100, 90, 3);
      delay(20);
      chirpSweep(1300, 2400, 110, 3);
    }
    // 13. Gemini Vision AI Target Lock-On ('V')
    else if (ch == 'V' || ch == 'v') {
      gLastKeepaliveTime = millis();
      chirpVisionLock();
    }
    // 14. Real-time Variable Speed Throttle ('P' + digits) - NON-BLOCKING & DISCARD-SAFE
    else if (ch == 'P' || ch == 'p') {
      gLastKeepaliveTime = millis();
      int targetSpeed = 0;
      unsigned long pStart = millis();
      while (millis() - pStart < 25) {
        if (Serial.available()) {
          char digit = Serial.peek();
          if (digit >= '0' && digit <= '9') {
            targetSpeed = targetSpeed * 10 + (Serial.read() - '0');
          } else {
            break; // Non-digit remains in buffer, emergency stop 'S' is NEVER eaten!
          }
        }
      }
      if (targetSpeed >= 60 && targetSpeed <= 255) {
        gDriveSpeed = targetSpeed;
        gTurnSpeed  = max(95, targetSpeed - 15);
        chirpSweep(600 + targetSpeed * 2, 800 + targetSpeed * 2, 80, 2);
      }
    }
    // 15. Grandmaster Knight L-Path Maneuver ('Z')
    else if (ch == 'Z' || ch == 'z') {
      gLastKeepaliveTime = millis();
      runKnightLPathManeuver();
    }
    // 16. Dedicated Real-Time Face Clock ('@' + time string + '\n') - Fixed buffer, zero malloc
    else if (ch == '@') {
      gLastKeepaliveTime = millis();
      haltMotors();
      gCurrentMode = MODE_BLUETOOTH_RC;
      char clockBanner[24];
      size_t n = Serial.readBytesUntil('\n', clockBanner, sizeof(clockBanner) - 1);
      clockBanner[n] = '\0';
      char* p = clockBanner;
      while (*p == ' ' || *p == '\r' || *p == '\t') p++;
      int endIdx = (int)strlen(p) - 1;
      while (endIdx >= 0 && (p[endIdx] == ' ' || p[endIdx] == '\r' || p[endIdx] == '\t')) {
        p[endIdx--] = '\0';
      }
      if (strlen(p) > 0) {
        char banner[32];
        snprintf(banner, sizeof(banner), " %s ", p);
        haltMotors();
        scrollTextAcrossMatrix(banner, 45);
        haltMotors();
        displayBitmap(kIconCrown);
      }
    }
    // 17. Text Marquee Banner Streamer ('W' + string + '\n') - Fixed buffer, zero malloc
    else if (ch == 'W') {
      gLastKeepaliveTime = millis();
      haltMotors();
      char textBanner[24];
      size_t n = Serial.readBytesUntil('\n', textBanner, sizeof(textBanner) - 1);
      textBanner[n] = '\0';
      char* p = textBanner;
      while (*p == ' ' || *p == '\r' || *p == '\t') p++;
      int endIdx = (int)strlen(p) - 1;
      while (endIdx >= 0 && (p[endIdx] == ' ' || p[endIdx] == '\r' || p[endIdx] == '\t')) {
        p[endIdx--] = '\0';
      }
      if (strlen(p) > 0) {
        char banner[32];
        snprintf(banner, sizeof(banner), " %s ", p);
        scrollTextAcrossMatrix(banner, 45);
        displayBitmap(kIconCrown);
      }
    }
    // 18. Serene Static 8x8 Digital Clock ('#' + "HHMM\n") - Validated digits & clamped
    else if (ch == '#') {
      gLastKeepaliveTime = millis();
      haltMotors();
      gCurrentMode = MODE_BLUETOOTH_RC;
      char timeDigits[12];
      size_t n = Serial.readBytesUntil('\n', timeDigits, sizeof(timeDigits) - 1);
      timeDigits[n] = '\0';
      char* p = timeDigits;
      while (*p == ' ' || *p == '\r' || *p == '\t') p++;
      if (strlen(p) >= 4 && isdigit(p[0]) && isdigit(p[1]) && isdigit(p[2]) && isdigit(p[3])) {
        int hh = (p[0] - '0') * 10 + (p[1] - '0');
        int mm = (p[2] - '0') * 10 + (p[3] - '0');
        hh = constrain(hh, 0, 23);
        mm = constrain(mm, 0, 59);
        displayStaticClock((uint8_t)hh, (uint8_t)mm, true);
      }
    }
    // 19. Client Keepalive Pulse ('!' or '*')
    else if (ch == '!' || ch == '*') {
      gLastKeepaliveTime = millis();
    }
    // 20. Astromech Emotional Expressions (A=Yes, Y=Ecstasy, D=Grumpy, C=Fatigued)
    else if (ch == 'A' || ch == 'a') {
      gLastKeepaliveTime = millis();
      runSayingYesMood();
    }
    else if (ch == 'Y' || ch == 'y') {
      gLastKeepaliveTime = millis();
      runEcstasyMood();
    }
    else if (ch == 'D' || ch == 'd') {
      gLastKeepaliveTime = millis();
      runGrumpyMood();
    }
    else if (ch == 'C' || ch == 'c') {
      gLastKeepaliveTime = millis();
      runFatiguedMood();
    }
    // 21. Telemetry & EEPROM Diagnostics Query ('?')
    else if (ch == '?') {
      gLastKeepaliveTime = millis();
      uint16_t missionCount = 0;
      EEPROM.get(EEPROM_RUNS_ADDR, missionCount);
      Serial.print(F("S2-R2-D2|GEAR:"));
      Serial.print(gCurrentGear);
      Serial.print(F("|MUTE:"));
      Serial.print(gMasterMute ? 1 : 0);
      Serial.print(F("|BOOTS:"));
      Serial.println(missionCount);
    }
  }
}

void executeCurrentMode() {
  // 🛑 HOISTED UNIVERSAL DEADMAN WATCHDOG: Evaluated every iteration across all modes!
  if (gMotorActive) {
    if (gCurrentMode == MODE_BLUETOOTH_RC) {
      if (millis() - gLastMotionCmdTime > 400) {
        haltMotors();
        gMotorActive = false;
      }
    } else if (gCurrentMode != MODE_STANDBY) {
      // Autonomous modes require active client keepalive within 1500ms
      if (millis() - gLastKeepaliveTime > 1500) {
        haltMotors();
        gMotorActive = false;
        setMode(MODE_BLUETOOTH_RC);
        displayBitmap(kIconBrake);
      }
    }
  }

  switch (gCurrentMode) {
    case MODE_STANDBY:            runStandbyMode(); break;
    case MODE_OBSTACLE_AVOIDANCE: runObstacleMode(); break;
    case MODE_LINE_TRACKING:      runLineTrackingMode(); break;
    case MODE_BLUETOOTH_RC:       delay(5); break;
    case MODE_LIGHT_SHOW:         runLightShowMode(); break;
    case MODE_LIVING_PET:         runLivingPetEngine(); break;
    case MODE_FLASHBANG_AMBUSH:   runFlashbangAmbushMode(); break;
    case MODE_CLIFF_DETECTION:    runCliffDetectionMode(); break;
    case MODE_AIR_SYNTHESIZER:    runSpatialAirSynthesizer(); break;
    case MODE_APEX_SENTRY:        runApexSentryMode(); break;
  }
}

/* ============================================================================
 * SECTION 8: Mode Implementations & Contextual Astromech Sentience
 * ============================================================================
 */

void runStandbyMode() {
  haltMotors();
  displayBitmap(kIconStandby);
  digitalWrite(kPinStatusLed, (millis() / 500) % 2);
  checkBiologicalRespiration(); // Soft biological breathing murmurs every 8-14s!
  delay(40);
}

void runObstacleMode() {
  const uint8_t kAutoDriveSpeed = 175;
  const uint8_t kAutoTurnSpeed  = 160;

  // 🛑 CLIFF DETECTION FIRST (Table edge safety guard)
  uint8_t leftCliff = digitalRead(kPinLineSensorLeft);
  uint8_t rightCliff = digitalRead(kPinLineSensorRight);
  if (leftCliff == 0 || rightCliff == 0) {
    haltMotors();
    talkAstromech(EMOTION_ALERT);
    driveBackward(kAutoDriveSpeed); delay(280); haltMotors();
    pivotRight(kAutoTurnSpeed); delay(320); haltMotors();
    return;
  }

  float dist = getDistanceCm();
  if (Serial.available()) { haltMotors(); return; }

  // FAIL-SAFE: If sensor wire is disconnected, timed out, or faulted, halt immediately!
  if (gSonarFault || dist <= 0.0f) {
    haltMotors();
    displayBitmap(kIconBrake);
    if (millis() % 1200 < 60) talkAstromech(EMOTION_ALERT);
    delay(40);
    return;
  }

  // 1. Close-Range Emergency Reflex (Enhanced 18cm cushion for thin chair legs!)
  if (dist > 0.0f && dist < 18.0f) {
    haltMotors();
    talkAstromech(EMOTION_ALERT); // Startled reflex chirp!
    driveBackward(kAutoDriveSpeed);
    delay(280);
    haltMotors();
    pivotRight(kAutoTurnSpeed);
    delay(320);
    haltMotors();
  }
  // 2. Proactive Look-Ahead Radar (36cm detection cushion with Doppler Radar Echolocation)
  else if (dist > 0.0f && dist < 36.0f) {
    haltMotors();
    talkDialogue(PHRASE_HESITATION); // Inquisitive "Hmm... checking path options"
    displayBitmap(kArrowLeft);
    pivotLeft(kAutoTurnSpeed);
    delay(260);
    haltMotors();
    delay(50);
    float leftDist = getDistanceCm();
    playDopplerRadarPing(leftDist); // 🌊 Doppler acoustic distance ping
    delay(30);

    displayBitmap(kArrowRight);
    pivotRight(kAutoTurnSpeed);
    delay(520);
    haltMotors();
    delay(50);
    float rightDist = getDistanceCm();
    playDopplerRadarPing(rightDist); // 🌊 Doppler acoustic distance ping
    delay(30);

    if (leftDist > rightDist && leftDist > 30.0f) {
      displayBitmap(kArrowLeft);
      pivotLeft(kAutoTurnSpeed);
      delay(520);
      haltMotors();
    }
  }
  // 3. Clear Path Forward
  else {
    displayBitmap(kArrowForward);
    driveForward(kAutoDriveSpeed);
    delay(25);
  }
}

void runLineTrackingMode() {
  const uint8_t kAutoDriveSpeed = 175;
  const uint8_t kAutoTurnSpeed  = 160;

  uint8_t leftSensor  = digitalRead(kPinLineSensorLeft);
  uint8_t rightSensor = digitalRead(kPinLineSensorRight);

  static unsigned long lastRacingPip = 0;
  if (millis() - lastRacingPip > 1800) {
    talkAstromech(EMOTION_RACING);
    lastRacingPip = millis();
  }

  if (Serial.available()) { haltMotors(); return; }

  // 1. Smooth Differential Steering (Eliminates violent bang-bang wobble)
  if (leftSensor == 0 && rightSensor == 1) {
    gLineSweepStep = 0;
    gMotorActive = true;
    displayBitmap(kArrowLeft);
    analogWrite(kPinMotorLeftPwm, kAutoDriveSpeed / 3);
    analogWrite(kPinMotorRightPwm, kAutoTurnSpeed);
    digitalWrite(kPinMotorLeftDir, LOW);
    digitalWrite(kPinMotorRightDir, LOW);
  } else if (leftSensor == 1 && rightSensor == 0) {
    gLineSweepStep = 0;
    gMotorActive = true;
    displayBitmap(kArrowRight);
    analogWrite(kPinMotorLeftPwm, kAutoTurnSpeed);
    analogWrite(kPinMotorRightPwm, kAutoDriveSpeed / 3);
    digitalWrite(kPinMotorLeftDir, LOW);
    digitalWrite(kPinMotorRightDir, LOW);
  } else if (leftSensor == 1 && rightSensor == 1) {
    // 2. Off track (both sensors off line) -> 2-Micro-Sweep recovery
    if (gLineSweepStep == 0) {
      pivotLeft(kAutoTurnSpeed); delay(140); haltMotors(); delay(40);
      gLineSweepStep = 1;
    } else if (gLineSweepStep == 1) {
      pivotRight(kAutoTurnSpeed); delay(280); haltMotors(); delay(40);
      gLineSweepStep = 2;
    } else {
      // Finished line -> Clean finish-line victory halt
      haltMotors();
      displayBitmap(kIconCrown);
      playTone(880, 150); delay(40); playTone(1318, 250);
      gLineSweepStep = 0;
      setMode(MODE_BLUETOOTH_RC);
      return;
    }
  } else {
    // Center alignment on track
    gLineSweepStep = 0;
    displayBitmap(kArrowForward);
    driveForward(kAutoDriveSpeed);
  }
  delay(15);
}

void runBluetoothMode(char cmd) {
  static char lastCmd = 'S';
  static unsigned long lastAckTime = 0;

  // Spontaneous Astromech Intent Acknowledgment ("Okay, got it... let's go!")
  if (cmd != lastCmd && (millis() - lastAckTime > 350)) {
    lastAckTime = millis();
    if (cmd == 'F' || cmd == 'f') {
      talkDialogue(PHRASE_ROGER_LETS_GO); // "Okay, got it... let's roll!"
    } else if (cmd == 'B' || cmd == 'b') {
      talkDialogue(PHRASE_BACKING_UP); // Inquisitive "Backing up!"
    } else if (cmd == 'L' || cmd == 'l' || cmd == 'R' || cmd == 'r') {
      talkDialogue(PHRASE_TURNING); // Melodic bank
    }
  }
  lastCmd = cmd;

  switch (cmd) {
    case 'F': case 'f': displayBitmap(kArrowForward);  driveForward(gDriveSpeed); break;
    case 'B': case 'b': displayBitmap(kArrowBackward); driveBackward(gDriveSpeed); break;
    case 'L': case 'l': displayBitmap(kArrowLeft);     pivotLeft(gTurnSpeed); break;
    case 'R': case 'r': displayBitmap(kArrowRight);    pivotRight(gTurnSpeed); break;
    case 'S': case 's': default: displayBitmap(kIconBrake); haltMotors(); break;
  }
}

void runLightShowMode() {
  displayBitmap(kIconHeart);
  talkAstromech(EMOTION_HAPPY);
  delay(350);
  if (Serial.available()) return;
  displayBitmap(kIconStandby);
  delay(400);
}

/**
 * Enhanced Persona 2: The Living Pet Companion & Autonomous Human Escort (Stanford Safe Architecture)
 */
void runLivingPetEngine() {
  float dist = getFilteredDistance();
  if (Serial.available()) { haltMotors(); return; }

  // FAIL-SAFE: If sensor wire is disconnected, halt immediately!
  if (gSonarFault) {
    haltMotors();
    displayBitmap(kIconBrake);
    delay(40);
    return;
  }

  const float kSafeStopDist = 18.0f; // Any obstacle closer than 18cm triggers immediate brake/standby
  const float kSweetSpotMax = 28.0f; // Sweet spot up to 28cm
  const float kMaxLeashDist = 80.0f; // Dynamic human leg tracking up to 80cm

  static unsigned long lastLostTime = 0;
  static unsigned long lastChirpTrillTime = 0;
  static int scanDir = 1;

  // 1. Initial Handshake Lock Ritual (Wait for Pilot to stand in front before rolling!)
  if (!gFollowLockAcquired) {
    haltMotors();
    displayBitmap(kIconStandby);
    
    // Interrogative sonar pulse
    if (millis() % 700 < 40) {
      if (!gMasterMute) { tone(kPinBuzzer1, 1400); delay(20); noTone(kPinBuzzer1); }
    }

    if (dist >= 10.0f && dist <= kMaxLeashDist) {
      // ✅ TARGET ACQUIRED! Complete Knight Handshake Ceremony
      gFollowLockAcquired = true;
      haltMotors();
      displayBitmap(kIconChessKnight);
      delay(180);
      displayBitmap(kIconCrown);
      // Eager 1cm nod toward King Shivansh
      driveForward(120); delay(50); haltMotors();
      talkAstromech(EMOTION_HAPPY);
      lastLostTime = millis();
      lastChirpTrillTime = millis();
    }
    delay(40);
    return;
  }

  // 2. Petting & IMMEDIATE BRAKE (< 18cm -> Contiguous collision stop, zero gap!)
  if (dist > 0.0f && dist < kSafeStopDist) {
    haltMotors();
    if (dist < 12.0f) {
      displayBitmap(kIconHeart);
      if (millis() - lastChirpTrillTime > 2500) {
        talkAstromech(EMOTION_HAPPY);
        lastChirpTrillTime = millis();
      }
    } else {
      displayBitmap(kIconBrake);
      stopAllAudio();
    }
    lastLostTime = millis();
    delay(40);
    return;
  }

  // 3. Grandmaster Sweet Spot (18cm - 28cm -> Standby & Subtle Diagonal Flank Scan)
  if (dist >= kSafeStopDist && dist <= kSweetSpotMax) {
    haltMotors();
    displayBitmap(kIconChessKnight);
    stopAllAudio();
    lastLostTime = millis();

    // Subtle Grandmaster Diagonal Flank Scan every 2400ms
    static unsigned long lastFlankScan = 0;
    if (millis() - lastFlankScan > 2400) {
      lastFlankScan = millis();
      pivotLeft(110); delay(35); haltMotors(); delay(40);
      pivotRight(110); delay(70); haltMotors(); delay(40);
      pivotLeft(110); delay(35); haltMotors();
    }
    delay(30);
  }
  // 4. Human Walking Away (28cm - 80cm -> Smooth Proportional Follow)
  else if (dist > kSweetSpotMax && dist <= kMaxLeashDist) {
    displayBitmap(kArrowForward);
    // Smooth proportional speed curve (115 PWM gentle to 215 PWM sprint)
    uint8_t followSpeed = map(constrain((long)dist, 28, 80), 28, 80, 115, 215);
    driveForward(followSpeed);
    lastLostTime = millis();
    delay(30);
  }
  // 5. Lost Target / Out of Range (>80cm / 999cm) -> Saccadic Radar Scan
  else {
    haltMotors();
    displayBitmap(kIconStandby);

    // If lost for more than 400ms, gently search left/right
    if (millis() - lastLostTime > 400 && millis() - lastLostTime < 2500) {
      if (scanDir > 0) pivotRight(120);
      else pivotLeft(120);
      delay(90);
      haltMotors();
      delay(60);

      float checkDist = getDistanceCm();
      if (checkDist >= kSafeStopDist && checkDist <= kMaxLeashDist) {
        lastLostTime = millis(); // Re-acquired target!
        
        // 4-Second Audio & Heart Debounce (Prevents chirp spam on fast steps!)
        if (millis() - lastChirpTrillTime > 4000) {
          displayBitmap(kIconHeart);
          talkAstromech(EMOTION_HAPPY);
          lastChirpTrillTime = millis();
        }
      } else {
        scanDir = -scanDir; // Alternate search direction
      }
    } else if (millis() - lastLostTime >= 2500) {
      // 2.5s Auto-Timeout: Safe sleep disarm (reset handshake for next lock)
      gFollowLockAcquired = false;
      haltMotors();
      digitalWrite(kPinStatusLed, (millis() / 400) % 2);
      delay(60);
    }
  }
}

/**
 * Enhanced Persona 4: Spatial Air Synthesizer Theremin
 */
void runSpatialAirSynthesizer() {
  haltMotors();
  if (Serial.available()) { haltMotors(); stopAllAudio(); return; }

  int lightVal = readActivePhotocell();
  uint8_t noteIdx = map(constrain(lightVal, 60, 920), 60, 920, 0, 9);
  unsigned int targetFreq = kPentatonicScale[noteIdx];

  float rightHandDist = getDistanceCm();
  static uint8_t rhythmStep = 0;
  rhythmStep = (rhythmStep + 1) % 8;

  displayOscilloscope(noteIdx, rhythmStep);

  if (!gMasterMute) {
    if (rightHandDist < 10.0f) {
      setBuzzerTone(targetFreq); delay(35);
      setBuzzerTone(targetFreq + 120); delay(35);
    } else if (rightHandDist < 25.0f) {
      setBuzzerTone(targetFreq); delay(70);
      stopAllAudio(); delay(30);
    } else {
      setBuzzerTone(targetFreq); delay(60);
    }
  } else {
    stopAllAudio();
    delay(60);
  }
}

/**
 * Mode 9: Apex Ultrasonic & Perimeter Sentry (Presence & Approach Detection)
 */
void runApexSentryMode() {
  haltMotors();
  static float lastDist = 100.0f;
  float dist = getDistanceCm();
  bool pirTrigger = checkPirMotionDetected();

  // Approach detection: triggers when someone is within 1.2m and approaching or within 50cm
  bool approachDetected = (dist > 8.0f && dist < 120.0f && (lastDist - dist > 6.0f || dist < 50.0f));
  lastDist = dist;

  if (pirTrigger || approachDetected) {
    static bool stepToggle = false;
    stepToggle = !stepToggle;

    if (stepToggle) displayBitmap(kIconSideWalk1);
    else displayBitmap(kIconSideWalk2);

    digitalWrite(kPinStatusLed, HIGH);
    talkAstromech(EMOTION_GREETING); // Enthusiastic welcome greeting!
    delay(150);
  } else {
    displayBitmap(kIconQuietSleep);
    digitalWrite(kPinStatusLed, LOW);
    stopAllAudio();
    delay(60);
  }
}

void runFlashbangAmbushMode() {
  haltMotors();
  float dist = getDistanceCm();
  bool heat = checkPirMotionDetected();
  if (Serial.available()) { haltMotors(); stopAllAudio(); return; }

  // 🛑 Cliff check before lunging
  uint8_t leftCliff  = digitalRead(kPinLineSensorLeft);
  uint8_t rightCliff = digitalRead(kPinLineSensorRight);
  if (leftCliff == 0 || rightCliff == 0) {
    haltMotors();
    return;
  }

  if (dist > 0.0f && dist < 45.0f && heat) {
    displayBitmap(kIconSkullAlert);
    uint8_t ambushSpeed = (gCurrentGear == GEAR_PRECISION) ? 140 : 180;
    for (int i = 0; i < 4; i++) {
      if (Serial.available() || digitalRead(kPinLineSensorLeft) == 0 || digitalRead(kPinLineSensorRight) == 0) {
        haltMotors(); stopAllAudio(); return;
      }
      setBuzzerTone(2200);
      driveForward(ambushSpeed); delay(80);
      if (Serial.available() || digitalRead(kPinLineSensorLeft) == 0 || digitalRead(kPinLineSensorRight) == 0) {
        haltMotors(); stopAllAudio(); return;
      }
      setBuzzerTone(800);
      driveBackward(ambushSpeed); delay(80);
    }
    stopAllAudio();
    haltMotors();
    delay(500);
  } else {
    displayBitmap(kIconSentryEye);
    digitalWrite(kPinStatusLed, (millis() / 800) % 2);
    delay(80);
  }
  haltMotors();
}

void runCliffDetectionMode() {
  uint8_t leftCliff  = digitalRead(kPinLineSensorLeft);
  uint8_t rightCliff = digitalRead(kPinLineSensorRight);

  if (leftCliff == 0 || rightCliff == 0) {
    haltMotors();
    talkAstromech(EMOTION_RELIEVED); // Relieved "Phew, that was close!" sigh
    displayBitmap(kIconSkullAlert);
    driveBackward(160);
    delay(280);
    haltMotors();
    pivotRight(180);
    delay(350);
    haltMotors();
  } else {
    displayBitmap(kIconCliffGuard);
    driveForward(140);
    delay(20);
  }
}

/* ============================================================================
 * SECTION 9: Special Protocols & Morse Academy
 * ============================================================================
 */

void runTableSafeJoyGreeting() {
  haltMotors();
  displayBitmap(kIconCrown);
  digitalWrite(kPinStatusLed, HIGH);

  // 1. Check underside cliff sensors for safety before micro-nod
  uint8_t leftCliff  = digitalRead(kPinLineSensorLeft);
  uint8_t rightCliff = digitalRead(kPinLineSensorRight);

  static uint8_t gGreetingStyle = 0;
  uint8_t currentStyle = gGreetingStyle % 3;
  gGreetingStyle++;

  switch (currentStyle) {
    // 🌟 VARIATION 1: Imperial Star Wars Throne Room Fanfare
    case 0: {
      displayBitmap(kIconCrown);
      if (!gMasterMute) {
        int notes[] = { 440, 440, 440, 349, 523, 440, 349, 523, 440 };
        int dur[]   = { 260, 260, 260, 180, 100, 260, 180, 100, 450 };
        for (int i = 0; i < 9; i++) {
          if (Serial.available()) { haltMotors(); return; }
          playTone(notes[i], dur[i]);
          if (i == 3) displayBitmap(kIconHeart);
          if (i == 6) displayBitmap(kIconCrown);
          delay(30);
        }
      }
      if (leftCliff != 0 && rightCliff != 0) {
        driveForward(140); delay(45); haltMotors(); delay(40);
        driveBackward(140); delay(45); haltMotors(); delay(40);
      }
      scrollTextAcrossMatrix(" KING SHIVANSH ", 45);
      break;
    }

    // ⚡ VARIATION 2: Grandmaster Knight's Royal Fanfare
    case 1: {
      displayBitmap(kIconChessKnight);
      if (!gMasterMute) {
        int notes[] = { 523, 659, 784, 1046, 1318, 1046, 1318 };
        int dur[]   = { 120, 120, 120,  220,  150,  120,  400 };
        for (int i = 0; i < 7; i++) {
          if (Serial.available()) { haltMotors(); return; }
          playTone(notes[i], dur[i]);
          if (i == 3) displayBitmap(kIconCrown);
          delay(25);
        }
      }
      if (leftCliff != 0 && rightCliff != 0) {
        // Playful double micro-bow (100% linear, zero yaw)
        driveForward(140); delay(35); haltMotors(); delay(30);
        driveBackward(140); delay(35); haltMotors(); delay(30);
        driveForward(140); delay(35); haltMotors(); delay(30);
        driveBackward(140); delay(35); haltMotors(); delay(40);
      }
      scrollTextAcrossMatrix(" GRANDMASTER SHIVANSH ", 45);
      break;
    }

    // 💖 VARIATION 3: Melodic Companion Astromech Singing Trill
    case 2: {
      displayBitmap(kIconHeart);
      if (!gMasterMute) {
        chirpSweep(900, 2200, 120, 2); delay(30);
        chirpWarble(1600, 2400, 3, 25); delay(25);
        chirpSweep(2200, 1400, 100, 2); delay(30);
        tone(kPinBuzzer1, 1975); delay(180); noTone(kPinBuzzer1); delay(40);
        tone(kPinBuzzer1, 2637); delay(350); noTone(kPinBuzzer1);
      }
      if (leftCliff != 0 && rightCliff != 0) {
        driveForward(140); delay(50); haltMotors(); delay(40);
        driveBackward(140); delay(50); haltMotors(); delay(40);
      }
      scrollTextAcrossMatrix(" MY KING SHIVANSH ", 45);
      break;
    }
  }

  talkAstromech(EMOTION_HAPPY);
  displayBitmap(kIconCrown);
  digitalWrite(kPinStatusLed, LOW);
  haltMotors();
}

void runKingShivanshProtocol() {
  haltMotors();
  displayBitmap(kIconCrown);
  scrollTextAcrossMatrix(" KING SHIVANSH ", 45);
  displayBitmap(kIconCrown);
}

void runSpinTheDroidRoulette() {
  haltMotors();
  for (int pwm = 255; pwm >= 110; pwm -= 25) {
    if (Serial.available()) { haltMotors(); return; }
    pivotRight(pwm);
    displayBitmap(kIconCrown);
    playTone(600 + pwm * 3, 30);
    for (unsigned long start = millis(); millis() - start < 100; delay(10)) {
      if (Serial.available()) { haltMotors(); return; }
    }
  }
  haltMotors();
  talkAstromech(EMOTION_HAPPY);
  displayBitmap(kIconHeart);
}

void runKnightLPathManeuver() {
  haltMotors();
  displayBitmap(kIconChessKnight);
  digitalWrite(kPinStatusLed, HIGH);

  // Tournament Chess Clock Double-Tap Sound
  if (!gMasterMute) {
    tone(kPinBuzzer1, 1800); delay(25);
    noTone(kPinBuzzer1); delay(35);
    tone(kPinBuzzer1, 1200); delay(35);
    noTone(kPinBuzzer1);
  }

  if (Serial.available()) { haltMotors(); return; }

  // 1. Two paces forward (Cliff-Guarded & Obstacle-Guarded)
  if (digitalRead(kPinLineSensorLeft) != 0 && digitalRead(kPinLineSensorRight) != 0 && getDistanceCm() > 15.0f) {
    driveForward(175);
    for (unsigned long start = millis(); millis() - start < 420; delay(10)) {
      if (Serial.available() || digitalRead(kPinLineSensorLeft) == 0 || digitalRead(kPinLineSensorRight) == 0) {
        haltMotors(); return;
      }
    }
    haltMotors();
  }
  delay(60);
  if (Serial.available()) { haltMotors(); return; }

  // 2. Crisp 90-degree pivot
  pivotRight(185);
  for (unsigned long start = millis(); millis() - start < 270; delay(10)) {
    if (Serial.available()) { haltMotors(); return; }
  }
  haltMotors();
  delay(60);
  if (Serial.available()) { haltMotors(); return; }

  // 3. One pace forward (Completing the L - Cliff-Guarded & Obstacle-Guarded)
  if (digitalRead(kPinLineSensorLeft) != 0 && digitalRead(kPinLineSensorRight) != 0 && getDistanceCm() > 15.0f) {
    driveForward(175);
    for (unsigned long start = millis(); millis() - start < 240; delay(10)) {
      if (Serial.available() || digitalRead(kPinLineSensorLeft) == 0 || digitalRead(kPinLineSensorRight) == 0) {
        haltMotors(); return;
      }
    }
    haltMotors();
  }
  delay(40);

  // 4. Tactical fanfare & Crown
  talkAstromech(EMOTION_HAPPY);
  displayBitmap(kIconCrown);
  digitalWrite(kPinStatusLed, LOW);
  haltMotors();
}

// ----------------------------------------------------------------------------
// MORSE CODE ACADEMY AUDIO-VISUAL ENGINE
// ----------------------------------------------------------------------------
void playMorseDit() {
  displayBitmap(kIconMorseDot);
  digitalWrite(kPinStatusLed, HIGH);
  chirpSweep(850, 950, 50, 4);
  digitalWrite(kPinStatusLed, LOW);
  clearMatrix();
  delay(80);
}

void playMorseDah() {
  displayBitmap(kIconMorseDash);
  digitalWrite(kPinStatusLed, HIGH);
  chirpSweep(850, 900, 20, 15);
  digitalWrite(kPinStatusLed, LOW);
  clearMatrix();
  delay(80);
}

void playMorsePattern(const char* pattern) {
  haltMotors();
  for (int i = 0; pattern[i] != '\0'; i++) {
    if (Serial.available()) return;
    if (pattern[i] == '.') playMorseDit();
    else if (pattern[i] == '-') playMorseDah();
  }
  delay(200);
}

void playMorseLetter(char letter) {
  letter = toupper(letter);
  switch (letter) {
    case 'A': playMorsePattern(".-"); break;
    case 'B': playMorsePattern("-..."); break;
    case 'C': playMorsePattern("-.-."); break;
    case 'D': playMorsePattern("-.."); break;
    case 'E': playMorsePattern("."); break;
    case 'F': playMorsePattern("..-."); break;
    case 'G': playMorsePattern("--."); break;
    case 'H': playMorsePattern("...."); break;
    case 'I': playMorsePattern(".."); break;
    case 'J': playMorsePattern(".---"); break;
    case 'K': playMorsePattern("-.-"); break;
    case 'L': playMorsePattern(".-.."); break;
    case 'M': playMorsePattern("--"); break;
    case 'N': playMorsePattern("-."); break;
    case 'O': playMorsePattern("---"); break;
    case 'P': playMorsePattern(".--."); break;
    case 'Q': playMorsePattern("--.-"); break;
    case 'R': playMorsePattern(".-."); break;
    case 'S': playMorsePattern("..."); break;
    case 'T': playMorsePattern("-"); break;
    case 'U': playMorsePattern("..-"); break;
    case 'V': playMorsePattern("...-"); break;
    case 'W': playMorsePattern(".--"); break;
    case 'X': playMorsePattern("-..-"); break;
    case 'Y': playMorsePattern("-.--"); break;
    case 'Z': playMorsePattern("--.."); break;
    case '1': playMorsePattern(".----"); break;
    case '2': playMorsePattern("..---"); break;
    case '3': playMorsePattern("...--"); break;
    case '4': playMorsePattern("....-"); break;
    case '5': playMorsePattern("....."); break;
    case '6': playMorsePattern("-...."); break;
    case '7': playMorsePattern("--..."); break;
    case '8': playMorsePattern("---.."); break;
    case '9': playMorsePattern("----."); break;
    case '0': playMorsePattern("-----"); break;
    default: delay(150); break;
  }
}

/* ============================================================================
 * SECTION 10: 8-Bit Chiptune Jukebox
 * ============================================================================
 */

void playStarWars() {
  displayBitmap(kIconSkullAlert);
  int notes[] = { 440, 440, 440, 349, 523, 440, 349, 523, 440 };
  int durations[] = { 400, 400, 400, 280, 140, 400, 280, 140, 600 };
  for (int i = 0; i < 9; i++) {
    if (Serial.available()) break;
    playTone(notes[i], durations[i]);
    delay(50);
  }
}

void playR2D2Chirps() {
  displayBitmap(kIconHeart);
  for (int i = 0; i < 8; i++) {
    if (Serial.available()) break;
    int freq = random(800, 2600);
    playTone(freq, random(40, 90));
    delay(random(20, 60));
  }
}

void playSuperMario() {
  displayBitmap(kIconHeart);
  int notes[] = { 660, 660, 660, 510, 660, 770, 380 };
  int durations[] = { 100, 100, 100, 100, 100, 100, 200 };
  for (int i = 0; i < 7; i++) {
    if (Serial.available()) break;
    playTone(notes[i], durations[i]);
    delay(60);
  }
}

void playMissionImpossible() {
  displayBitmap(kIconSentryEye);
  int notes[] = { 587, 587, 698, 784, 587, 587, 523, 554 };
  int durations[] = { 150, 150, 150, 150, 150, 150, 150, 150 };
  for (int i = 0; i < 8; i++) {
    if (Serial.available()) break;
    playTone(notes[i], durations[i]);
    delay(50);
  }
}

/* ============================================================================
 * SECTION 11: Master Audio HAL Drivers & R2-D2 Acoustic Synthesis Engine
 * ============================================================================
 */

void setBuzzerTone(unsigned int freqHz) {
  if (gMasterMute) {
    noTone(kPinBuzzer1);
    return;
  }
  tone(kPinBuzzer1, freqHz);
}

void playTone(unsigned int freqHz, unsigned long durationMs) {
  if (gMasterMute) {
    delay(durationMs);
    return;
  }
  tone(kPinBuzzer1, freqHz);
  delay(durationMs);
  noTone(kPinBuzzer1);
}

void stopAllAudio() {
  noTone(kPinBuzzer1);
}

void chirpSweep(int startF, int endF, int step, int delayMs) {
  if (gMasterMute) return;
  if (startF < endF) {
    for (int f = startF; f <= endF; f += step) {
      tone(kPinBuzzer1, f);
      delay(delayMs);
    }
  } else {
    for (int f = startF; f >= endF; f -= step) {
      tone(kPinBuzzer1, f);
      delay(delayMs);
    }
  }
  noTone(kPinBuzzer1);
}

void chirpWarble(int f1, int f2, int reps, int durMs) {
  if (gMasterMute) return;
  for (int i = 0; i < reps; i++) {
    tone(kPinBuzzer1, f1); delay(durMs);
    tone(kPinBuzzer1, f2); delay(durMs);
  }
  noTone(kPinBuzzer1);
}

void talkAstromech(DroidEmotion emotion) {
  if (gMasterMute) return;
  switch (emotion) {
    case EMOTION_HAPPY: {
      int numChirps = random(2, 5);
      for (int i = 0; i < numChirps; i++) {
        int startF = random(900, 1800);
        int endF = startF + random(300, 800);
        chirpSweep(startF, endF, 120, 3);
        delay(random(15, 35));
      }
      break;
    }
    case EMOTION_CURIOUS: {
      int f1 = random(1100, 1900);
      chirpSweep(f1, f1 + 400, 80, 4);
      delay(20);
      tone(kPinBuzzer1, random(1800, 2400)); delay(40);
      noTone(kPinBuzzer1);
      break;
    }
    case EMOTION_RELIEVED: {
      chirpSweep(1800, 600, 100, 3);
      delay(30);
      chirpSweep(700, 1100, 80, 4);
      break;
    }
    case EMOTION_ALERT: {
      tone(kPinBuzzer1, 2400); delay(30);
      noTone(kPinBuzzer1); delay(15);
      tone(kPinBuzzer1, 2800); delay(40);
      noTone(kPinBuzzer1);
      break;
    }
    case EMOTION_GREETING: {
      chirpSweep(900, 2200, 140, 3);
      delay(30);
      chirpWarble(1700, 2500, 3, 20);
      delay(20);
      chirpSweep(2100, 1300, 100, 3);
      break;
    }
    case EMOTION_SLEEPY: {
      chirpSweep(1200, 600, 50, 6);
      break;
    }
    case EMOTION_RACING: {
      tone(kPinBuzzer1, random(1600, 2600)); delay(25);
      noTone(kPinBuzzer1);
      break;
    }
  }
}

void talkDialogue(DialoguePhrase phrase) {
  if (gMasterMute) return;
  switch (phrase) {
    case PHRASE_ROGER_LETS_GO: {
      // Snappy, eager "Okay, I got it... let's go!"
      chirpSweep(1100, 1900, 120, 2);
      delay(15);
      tone(kPinBuzzer1, 2200); delay(30);
      noTone(kPinBuzzer1); delay(10);
      chirpSweep(1600, 2400, 140, 2);
      break;
    }
    case PHRASE_AFFIRMATIVE: {
      // Crisp "Understood!"
      tone(kPinBuzzer1, 1750); delay(20);
      noTone(kPinBuzzer1); delay(15);
      tone(kPinBuzzer1, 2350); delay(35);
      noTone(kPinBuzzer1);
      break;
    }
    case PHRASE_BACKING_UP: {
      // Inquisitive "Backing up!"
      chirpSweep(1800, 1200, 90, 2);
      delay(15);
      tone(kPinBuzzer1, 950); delay(40);
      noTone(kPinBuzzer1);
      break;
    }
    case PHRASE_TURNING: {
      // Melodic bank
      tone(kPinBuzzer1, random(1400, 1900)); delay(18);
      noTone(kPinBuzzer1);
      break;
    }
    case PHRASE_GEAR_SHIFT: {
      // Harmonious transmission chirp
      chirpSweep(800, 2200, 140, 2);
      break;
    }
    case PHRASE_AMBIENT_MURMUR: {
      // Soft 2-syllable biological breathing murmur
      int f1 = random(1100, 1500);
      tone(kPinBuzzer1, f1); delay(18);
      tone(kPinBuzzer1, f1 + random(100, 300)); delay(22);
      noTone(kPinBuzzer1);
      break;
    }
    case PHRASE_QUESTION: {
      chirpSweep(1200, 2500, 80, 2);
      break;
    }
    case PHRASE_HESITATION: {
      // Inquisitive "Hmm... checking path options"
      chirpSweep(1350, 1800, 90, 2);
      delay(25);
      chirpSweep(1750, 1500, 70, 2);
      break;
    }
    case PHRASE_ECSTASY: {
      // Sparkling bubbling multi-octave arpeggio (Astromech bliss!)
      int notes[] = { 880, 1175, 1397, 1760, 2093, 2637, 3136, 3520 };
      for (int i = 0; i < 8; i++) {
        tone(kPinBuzzer1, notes[i]); delay(28);
      }
      chirpWarble(2400, 3200, 3, 15);
      noTone(kPinBuzzer1);
      break;
    }
    case PHRASE_R2D2_YES: {
      // 4-tone rapid ascending cheerful R2-D2 affirmation
      chirpSweep(1200, 2400, 70, 2);
      delay(10);
      tone(kPinBuzzer1, 2794); delay(35);
      noTone(kPinBuzzer1); delay(10);
      chirpWarble(1800, 2600, 2, 18);
      noTone(kPinBuzzer1);
      break;
    }
    case PHRASE_GRUMPY: {
      // Low raspy mechanical grunt / complaint
      tone(kPinBuzzer1, 550); delay(50);
      tone(kPinBuzzer1, 380); delay(65);
      tone(kPinBuzzer1, 280); delay(90);
      noTone(kPinBuzzer1); delay(15);
      tone(kPinBuzzer1, 420); delay(40);
      tone(kPinBuzzer1, 220); delay(120);
      noTone(kPinBuzzer1);
      break;
    }
    case PHRASE_FATIGUED: {
      // Long 400ms descending tired power-down sigh
      chirpSweep(1500, 380, 220, 3);
      delay(20);
      tone(kPinBuzzer1, 260); delay(140);
      tone(kPinBuzzer1, 140); delay(240);
      noTone(kPinBuzzer1);
      break;
    }
  }
}

void runSayingYesMood() {
  haltMotors();
  displayBitmap(kIconAffirmative);
  talkDialogue(PHRASE_R2D2_YES);
  uint8_t leftCliff  = digitalRead(kPinLineSensorLeft);
  uint8_t rightCliff = digitalRead(kPinLineSensorRight);
  if (leftCliff != 0 && rightCliff != 0) {
    driveForward(140); delay(35); haltMotors(); delay(25);
    driveBackward(140); delay(35); haltMotors();
  }
}

void runEcstasyMood() {
  haltMotors();
  displayBitmap(kIconEcstasy);
  talkDialogue(PHRASE_ECSTASY);
  displayBitmap(kIconHeart);
  delay(120);
  displayBitmap(kIconCrown);
}

void runGrumpyMood() {
  haltMotors();
  displayBitmap(kIconGrumpy);
  talkDialogue(PHRASE_GRUMPY);
  pivotLeft(130); delay(15);
  pivotRight(130); delay(30);
  pivotLeft(130); delay(15);
  haltMotors();
}

void runFatiguedMood() {
  haltMotors();
  displayBitmap(kIconFatigued);
  talkDialogue(PHRASE_FATIGUED);
  uint8_t leftCliff  = digitalRead(kPinLineSensorLeft);
  uint8_t rightCliff = digitalRead(kPinLineSensorRight);
  if (leftCliff != 0 && rightCliff != 0) {
    driveBackward(130); delay(45); haltMotors();
  }
  displayBitmap(kIconQuietSleep);
}

void playDopplerRadarPing(float distanceCm) {
  if (gMasterMute || distanceCm <= 0.0f || distanceCm > 120.0f) return;
  int freq = map(constrain((long)distanceCm, 18, 80), 18, 80, 2200, 450);
  tone(kPinBuzzer1, freq);
  delay(12);
  noTone(kPinBuzzer1);
}

void checkBiologicalRespiration() {
  static unsigned long lastRespirationTime = 0;
  static unsigned long nextInterval = 9000;
  if (millis() - lastRespirationTime > nextInterval) {
    lastRespirationTime = millis();
    nextInterval = random(8000, 14000); // Uniform random interval (8s to 14s)
    talkDialogue(PHRASE_AMBIENT_MURMUR);
  }
}

void chirpAccelerate() {
  chirpSweep(900, 2200, 140, 3);
}

void chirpBrake() {
  chirpSweep(1600, 500, 120, 3);
}

void chirpVisionLock() {
  if (gMasterMute) return;
  chirpSweep(1200, 2400, 200, 4);
  delay(20);
  chirpSweep(2400, 1800, 150, 3);
  delay(20);
  chirpSweep(2000, 2600, 150, 4);
}

void chirpHappyPet() {
  talkAstromech(EMOTION_HAPPY);
}

void chirpAlert() {
  talkAstromech(EMOTION_ALERT);
}

void chirpCurious() {
  talkAstromech(EMOTION_CURIOUS);
}

void matrixInit() {
  Wire.begin();
  Wire.setWireTimeout(3000, true); // 🛑 3ms I2C timeout with automatic bus recovery on line hang!
  Wire.beginTransmission(MATRIX_I2C_ADDR); Wire.write(0x21); Wire.endTransmission();
  Wire.beginTransmission(MATRIX_I2C_ADDR); Wire.write(0xEF); Wire.endTransmission();
  Wire.beginTransmission(MATRIX_I2C_ADDR); Wire.write(0x81); Wire.endTransmission();
}

void displayBitmap(const uint8_t* bitmap) {
  Wire.beginTransmission(MATRIX_I2C_ADDR);
  Wire.write(0x00);
  for (uint8_t row = 0; row < 8; row++) {
    uint8_t data = pgm_read_byte(&bitmap[7 - row]); // Calibrated Upright
    uint8_t mapped = 0;
    for (int b = 0; b < 8; b++) {
      if (data & (1 << b)) mapped |= (1 << (7 - b));
    }
    Wire.write(mapped);
    Wire.write(0x00);
  }
  Wire.endTransmission();
}

void displayCustomBuffer(const uint8_t* customBuffer) {
  Wire.beginTransmission(MATRIX_I2C_ADDR);
  Wire.write(0x00);
  for (uint8_t row = 0; row < 8; row++) {
    uint8_t data = customBuffer[7 - row]; // Calibrated Upright
    uint8_t mapped = 0;
    for (int b = 0; b < 8; b++) {
      if (data & (1 << b)) mapped |= (1 << (7 - b));
    }
    Wire.write(mapped);
    Wire.write(0x00);
  }
  Wire.endTransmission();
}

void displayOscilloscope(uint8_t noteIdx, uint8_t rhythmStep) {
  Wire.beginTransmission(MATRIX_I2C_ADDR);
  Wire.write(0x00);
  uint8_t height = map(noteIdx, 0, 9, 1, 8);
  for (uint8_t row = 0; row < 8; row++) {
    if (row < height) {
      uint8_t wavePattern = (rhythmStep % 2 == 0) ? 0b11001100 : 0b00110011;
      Wire.write(wavePattern);
    } else {
      Wire.write(0x00);
    }
    Wire.write(0x00);
  }
  Wire.endTransmission();
}

// ----------------------------------------------------------------------------
// 5x7 ALPHANUMERIC FONT TABLE & SMOOTH RUNNING TEXT MARQUEE SCROLLER
// ----------------------------------------------------------------------------
const uint8_t PROGMEM kFont5x7[39][5] = {
  { 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0: ' ' Space
  { 0x7E, 0x11, 0x11, 0x11, 0x7E }, // 1: A
  { 0x7F, 0x49, 0x49, 0x49, 0x36 }, // 2: B
  { 0x3E, 0x41, 0x41, 0x41, 0x22 }, // 3: C
  { 0x7F, 0x41, 0x41, 0x22, 0x1C }, // 4: D
  { 0x7F, 0x49, 0x49, 0x49, 0x41 }, // 5: E
  { 0x7F, 0x09, 0x09, 0x09, 0x01 }, // 6: F
  { 0x3E, 0x41, 0x49, 0x49, 0x7A }, // 7: G
  { 0x7F, 0x08, 0x08, 0x08, 0x7F }, // 8: H
  { 0x00, 0x41, 0x7F, 0x41, 0x00 }, // 9: I
  { 0x20, 0x40, 0x41, 0x3F, 0x01 }, // 10: J
  { 0x7F, 0x08, 0x14, 0x22, 0x41 }, // 11: K
  { 0x7F, 0x40, 0x40, 0x40, 0x40 }, // 12: L
  { 0x7F, 0x02, 0x0C, 0x02, 0x7F }, // 13: M
  { 0x7F, 0x04, 0x08, 0x10, 0x7F }, // 14: N
  { 0x3E, 0x41, 0x41, 0x41, 0x3E }, // 15: O
  { 0x7F, 0x09, 0x09, 0x09, 0x06 }, // 16: P
  { 0x3E, 0x41, 0x51, 0x21, 0x5E }, // 17: Q
  { 0x7F, 0x09, 0x19, 0x29, 0x46 }, // 18: R
  { 0x46, 0x49, 0x49, 0x49, 0x31 }, // 19: S
  { 0x01, 0x01, 0x7F, 0x01, 0x01 }, // 20: T
  { 0x3F, 0x40, 0x40, 0x40, 0x3F }, // 21: U
  { 0x1F, 0x20, 0x40, 0x20, 0x1F }, // 22: V
  { 0x7F, 0x20, 0x18, 0x20, 0x7F }, // 23: W
  { 0x63, 0x14, 0x08, 0x14, 0x63 }, // 24: X
  { 0x07, 0x08, 0x70, 0x08, 0x07 }, // 25: Y
  { 0x61, 0x51, 0x49, 0x45, 0x43 }, // 26: Z
  { 0x3E, 0x51, 0x49, 0x45, 0x3E }, // 27: '0'
  { 0x00, 0x42, 0x7F, 0x40, 0x00 }, // 28: '1'
  { 0x42, 0x61, 0x51, 0x49, 0x46 }, // 29: '2'
  { 0x21, 0x41, 0x45, 0x4B, 0x31 }, // 30: '3'
  { 0x18, 0x14, 0x12, 0x7F, 0x10 }, // 31: '4'
  { 0x27, 0x45, 0x45, 0x45, 0x39 }, // 32: '5'
  { 0x3C, 0x4A, 0x49, 0x49, 0x30 }, // 33: '6'
  { 0x01, 0x71, 0x09, 0x05, 0x03 }, // 34: '7'
  { 0x36, 0x49, 0x49, 0x49, 0x36 }, // 35: '8'
  { 0x06, 0x49, 0x49, 0x29, 0x1E }, // 36: '9'
  { 0x00, 0x36, 0x36, 0x00, 0x00 }, // 37: ':' Colon
  { 0x08, 0x08, 0x08, 0x08, 0x08 }  // 38: '-' Dash
};

uint8_t getCharCol(char c, uint8_t colIdx) {
  if (colIdx >= 5) return 0x00; // Inter-letter spacing column
  c = toupper(c);
  if (c >= 'A' && c <= 'Z') {
    return pgm_read_byte(&kFont5x7[c - 'A' + 1][colIdx]);
  }
  if (c >= '0' && c <= '9') {
    return pgm_read_byte(&kFont5x7[c - '0' + 27][colIdx]);
  }
  if (c == ':') {
    return pgm_read_byte(&kFont5x7[37][colIdx]);
  }
  if (c == '-') {
    return pgm_read_byte(&kFont5x7[38][colIdx]);
  }
  return 0x00;
}

// ----------------------------------------------------------------------------
// 3x3 COMPACT NUMERIC FONT & SERENE STATIC 8x8 DIGITAL CLOCK ENGINE
// ----------------------------------------------------------------------------
const uint8_t PROGMEM kFont3x3[10][3] = {
  { 0b111, 0b101, 0b111 }, // 0
  { 0b010, 0b010, 0b010 }, // 1
  { 0b110, 0b010, 0b011 }, // 2
  { 0b110, 0b011, 0b110 }, // 3
  { 0b101, 0b111, 0b001 }, // 4
  { 0b111, 0b110, 0b011 }, // 5
  { 0b111, 0b110, 0b111 }, // 6
  { 0b111, 0b001, 0b001 }, // 7
  { 0b111, 0b010, 0b111 }, // 8
  { 0b111, 0b111, 0b001 }  // 9
};

void displayStaticClock(uint8_t hours, uint8_t minutes, bool showColon) {
  hours = constrain(hours, 0, 23);
  minutes = constrain(minutes, 0, 59);
  uint8_t hDisplay = (hours % 12 == 0) ? 12 : (hours % 12);
  uint8_t h1 = constrain(hDisplay / 10, 0, 9);
  uint8_t h2 = constrain(hDisplay % 10, 0, 9);
  uint8_t m1 = constrain(minutes / 10, 0, 9);
  uint8_t m2 = constrain(minutes % 10, 0, 9);

  uint8_t frame[8] = {0};

  // Rows 0, 1, 2: Hours (d1 in cols 7..5, gap in col 4, d2 in cols 3..1)
  for (uint8_t r = 0; r < 3; r++) {
    uint8_t d1_row = pgm_read_byte(&kFont3x3[h1][r]);
    uint8_t d2_row = pgm_read_byte(&kFont3x3[h2][r]);
    frame[r] = (d1_row << 5) | (d2_row << 1);
  }

  // Row 3: Colon dots
  if (showColon) {
    frame[3] = 0b00100100;
  }

  // Rows 4, 5, 6: Minutes
  for (uint8_t r = 0; r < 3; r++) {
    uint8_t d3_row = pgm_read_byte(&kFont3x3[m1][r]);
    uint8_t d4_row = pgm_read_byte(&kFont3x3[m2][r]);
    frame[r + 4] = (d3_row << 5) | (d4_row << 1);
  }

  // Row 7: PM indicator dot in col 0
  if (hours >= 12) {
    frame[7] = 0b00000001;
  }

  displayCustomBuffer(frame);
}

void scrollTextAcrossMatrix(const char* text, uint8_t scrollSpeedMs) {
  uint16_t len = strlen(text);
  uint16_t totalCols = len * 6 + 8;

  for (uint16_t pos = 0; pos < totalCols; pos++) {
    if (Serial.available()) { haltMotors(); return; } // Non-blocking interrupt

    uint8_t frameRows[8] = {0};

    for (uint8_t screenCol = 0; screenCol < 8; screenCol++) {
      int16_t globalCol = (int16_t)pos + screenCol - 8;
      if (globalCol >= 0 && globalCol < (int16_t)(len * 6)) {
        uint16_t charIdx = globalCol / 6;
        uint8_t colInChar = globalCol % 6;
        uint8_t colData = getCharCol(text[charIdx], colInChar);

        for (uint8_t row = 0; row < 8; row++) {
          if (colData & (1 << row)) {
            frameRows[row] |= (1 << (7 - screenCol));
          }
        }
      }
    }

    displayCustomBuffer(frameRows);
    delay(scrollSpeedMs);
  }
  clearMatrix();
}

void clearMatrix() {
  Wire.beginTransmission(MATRIX_I2C_ADDR);
  Wire.write(0x00);
  for (uint8_t i = 0; i < 16; i++) Wire.write(0x00);
  Wire.endTransmission();
}

void haltMotors() {
  analogWrite(kPinMotorLeftPwm, 0);
  analogWrite(kPinMotorRightPwm, 0);
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, LOW);
  gMotorActive = false;
}

void driveForward(uint8_t speed) {
  analogWrite(kPinMotorLeftPwm, speed);
  analogWrite(kPinMotorRightPwm, speed);
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, LOW);
  if (speed > 0) gMotorActive = true;
}

void driveBackward(uint8_t speed) {
  analogWrite(kPinMotorLeftPwm, speed);
  analogWrite(kPinMotorRightPwm, speed);
  digitalWrite(kPinMotorLeftDir, HIGH);
  digitalWrite(kPinMotorRightDir, HIGH);
  if (speed > 0) gMotorActive = true;
}

void pivotLeft(uint8_t speed) {
  analogWrite(kPinMotorLeftPwm, speed);
  analogWrite(kPinMotorRightPwm, speed);
  digitalWrite(kPinMotorLeftDir, HIGH);
  digitalWrite(kPinMotorRightDir, LOW);
  if (speed > 0) gMotorActive = true;
}

void pivotRight(uint8_t speed) {
  analogWrite(kPinMotorLeftPwm, speed);
  analogWrite(kPinMotorRightPwm, speed);
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, HIGH);
  if (speed > 0) gMotorActive = true;
}

float getDistanceCm() {
  digitalWrite(kPinUltrasonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(kPinUltrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(kPinUltrasonicTrig, LOW);

  long duration = pulseIn(kPinUltrasonicEcho, HIGH, 25000); // 25ms max timeout (~4.2m)
  if (duration == 0) {
    gSonarFault = true; // 🛑 1 single timeout immediately latches fault!
    gSonarGoodCount = 0;
    return 0.0f; // STOP: return 0.0f, not 999.0f!
  }
  gSonarGoodCount++;
  if (gSonarGoodCount >= 5) {
    gSonarFault = false; // Requires 5 consecutive valid readings to clear!
  }
  return duration * 0.034f / 2.0f;
}

float getFilteredDistance() {
  float raw = getDistanceCm();
  if (raw > 0.0f && raw < 400.0f) {
    gDistanceHistory[gHistoryIdx] = raw;
    gHistoryIdx = (gHistoryIdx + 1) % 5;
  }

  // 6-Line Median-of-5 Filter (Rejects acoustic dropouts and spikes completely)
  float sorted[5];
  for (uint8_t i = 0; i < 5; i++) sorted[i] = gDistanceHistory[i];
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = i + 1; j < 5; j++) {
      if (sorted[i] > sorted[j]) {
        float tmp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = tmp;
      }
    }
  }
  gFollowFilteredDist = sorted[2];
  return gFollowFilteredDist;
}

int readActivePhotocell() {
  int rawA2 = analogRead(kPinExpansionA2);
  int rawA3 = analogRead(kPinExpansionA3);
  int rawA6 = analogRead(kPinExpansionA6);
  int rawA7 = analogRead(kPinExpansionA7);
  return max(max(rawA2, rawA3), max(rawA6, rawA7));
}

bool checkPirMotionDetected() {
  static uint8_t lastPirState = LOW;
  uint8_t current = digitalRead(kPinExpansionD12);
  bool triggered = (current == HIGH && lastPirState == LOW);
  lastPirState = current;
  return triggered;
}
