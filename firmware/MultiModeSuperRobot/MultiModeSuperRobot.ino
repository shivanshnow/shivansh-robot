/**
 * ============================================================================
 * MECHA OS 4.3 • WALL-E Autonomous Robotics Platform
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
const uint8_t kPinLdrFollow    = A6; // Photocell (LDR): the kit's light port is A6/A7 (signal on A6). A0 is the ONBOARD SOUND SENSOR — never the light sensor.
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
  MODE_LIVING_PET         = 5,  // Mode 5: Follow Me
  MODE_FLASHBANG_AMBUSH   = 6,
  MODE_CLIFF_DETECTION    = 7,
  MODE_AIR_SYNTHESIZER    = 8,
  MODE_APEX_SENTRY        = 9,
  MODE_FOCUS_CLOCK        = 10, // Sitting-clock countdown; never drives motors
  MODE_COUNT              = 11
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

// Pivot calibration derived from runKnightLPathManeuver:
// 270ms at PWM 185 produces a 90-degree pivot (~0.333 deg/ms, or 3 ms per degree).
const uint8_t kPivotCalibratedPwm = 185;
const uint16_t kPivotMsPerDegree = 3; // 270ms / 90 deg = 3 ms/deg (~0.333 deg/ms)

// Safety Watchdog & Deadman Timer (400ms RC deadman, 1500ms autonomous keepalive)
unsigned long gLastMotionCmdTime = 0;
unsigned long gLastEchoMicros   = 0; // raw sonar flight time of the last ping (0 = no echo) — Echo Lab
volatile unsigned long gLastKeepaliveTime = 0;
bool gMotorActive = false;
uint8_t gSonarGoodCount = 0;
bool gSonarFault = false;
uint8_t gLineSweepStep = 0;

// Cliff Sensor Health & Plausibility State
bool gCliffFault = false;
uint8_t gCliffRecoveryAttempts = 0;

// FLOOR / DESK (dad's call, 5 Sep 2026). The edge sensors are IR reflectance: on a dark
// floor they read "no floor" forever and would block all play. So the edge veto is ON only
// in Desk mode (chosen from the cockpit, or declared by the desk tools over USB) or in Desk
// Companion, whose whole job is the table edge. Boot = Floor mode. The deadman, the brake
// and the sonar collision stop are NOT affected by this switch — they stay on always.
bool gDeskMode = false;
inline bool edgeGuardActive() { return gDeskMode || gCurrentMode == MODE_CLIFF_DETECTION; }
inline bool edgeSeen(uint8_t l, uint8_t r) { return edgeGuardActive() && (l == 0 || r == 0 || gCliffFault); }
unsigned long gCliffForwardDriveStart = 0;
uint8_t gLastLeftCliffState = 1;
uint8_t gLastRightCliffState = 1;
unsigned long gCliffLastStateChangeTime = 0;

// Filter State (Fixed-point integer mm/cm to eliminate IEEE-754 software float library)
uint16_t gFollowFilteredDist = 20;
uint16_t gLatestRawDist = 0; // Unfiltered nearest sonar return for immediate safety stop
uint16_t gDistanceHistory[5] = {20, 20, 20, 20, 20};
uint8_t gHistoryIdx = 0;
uint8_t gSonarZeroRun = 0;          // F6: consecutive no-echo pings
const uint8_t kSonarZeroLost = 6;   // F6: this many in a row == target LOST
unsigned long gLastIdleChirpTime = 0;
bool gFollowLockAcquired = false;
bool gPrevFollowLocked = false; // F9: previous pass's lock state (heartbeat reset edge)

// Focus Clock (mode 10): a real sitting clock that lives in the robot, not the phone.
uint8_t gFocusMinutes = 25;
unsigned long gFocusStartMs = 0;
bool gFocusDone = false;

// Phase 2: Three-Sensor Fusion State & Bearing Memory
enum FollowBearing {
  BEARING_CENTER = 0,
  BEARING_LEFT   = -1,
  BEARING_RIGHT  = 1
};

struct BearingMemory {
  FollowBearing bearing;
  uint8_t confidence;
  unsigned long timestamp;
};

BearingMemory gLastBearing = {BEARING_CENTER, 0, 0};
bool gLdrHealthy = false;
bool gPirHealthy = false;

// Pentatonic Scale for Air Synthesizer (PROGMEM saves 20 bytes of static RAM)
const uint16_t PROGMEM kPentatonicScale[10] = {
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

// B2: the resting half of the heartbeat — same heart, one ring smaller.
const uint8_t PROGMEM kIconHeartSmall[8] = {
  0b00000000, 0b00000000, 0b00100100, 0b01111110,
  0b00111100, 0b00011000, 0b00000000, 0b00000000
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

// The face is on the FRONT, so a person looking at it sees the robot's left on their
// right. These arrows are drawn so that the viewer sees the arrow point the way the
// robot actually moves in the room (dad's call, 5 Sep 2026 — they used to look reversed).
const uint8_t PROGMEM kArrowLeft[8] = {
  0b00001000, 0b00001100, 0b00001110, 0b11111111,
  0b11111111, 0b00001110, 0b00001100, 0b00001000
};

const uint8_t PROGMEM kArrowRight[8] = {
  0b00010000, 0b00110000, 0b01110000, 0b11111111,
  0b11111111, 0b01110000, 0b00110000, 0b00010000
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
bool safeMotorDelay(unsigned long ms, bool checkCliff = true, bool checkSonar = false);
size_t readFramedPayload(char* buffer, size_t maxLen, unsigned long timeoutMs = 60);

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
void playDopplerRadarPing(uint16_t distanceCm);
void checkBiologicalRespiration();
void chirpAccelerate();
void chirpBrake();
void chirpVisionLock();
void chirpHappyPet();
void chirpAlert();
void chirpCurious();
void chirpRandomR2();      // Never-the-same R2 sentence (3-6 glides, total <= 250ms)
void chirpModeChange(uint8_t mode);
void serviceChirpNB();     // Advances the non-blocking chirp one step per loop pass

void haltMotors();
void driveForward(uint8_t speed);
void driveBackward(uint8_t speed);
void pivotLeft(uint8_t speed);
void pivotRight(uint8_t speed);
void arcDrive(uint8_t leftPwm, uint8_t rightPwm);
uint16_t getDistanceCm();
uint16_t getFilteredDistance();
int readActivePhotocell();
const uint8_t* tickFollowHeartbeat(uint16_t dist); // B2: Follow Me living heartbeat
void resetFollowHeartbeat();
int sampleLdrWithHealth();
bool checkPirMotionDetected();

// Master Studio Engines
void runStandbyMode();
void runObstacleMode();
void runLineTrackingMode();
void runBluetoothMode(char cmd);
void runLightShowMode();
void runLivingPetEngine(); // Mode 5: Follow Me
void runFlashbangAmbushMode();
void runCliffDetectionMode();
void runSpatialAirSynthesizer();
void runApexSentryMode();
void runFocusClockMode();
void displayStaticClock(uint8_t hours, uint8_t minutes, bool showColon);

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
uint16_t getStackFreeBytes();

/* ============================================================================
 * SECTION 5: Main Setup, Persistence & Loop
 * ============================================================================
 */

// 🛡️ Stack Watermarking: Paints unused SRAM with 0xC5 at boot to measure peak stack depth
extern uint8_t _end;
extern uint8_t __stack;

void paintStack(void) __attribute__((naked, section(".init1"), used));
void paintStack(void) {
  __asm volatile (
    "  ldi r30, lo8(_end)\n  ldi r31, hi8(_end)\n"
    "  ldi r24, 0xC5\n      ldi r25, hi8(__stack)\n"
    "0:cpi r30, lo8(__stack)\n  cpc r31, r25\n  brsh 1f\n"
    "  st  Z+, r24\n        rjmp 0b\n1:\n" ::);
}

uint16_t getStackFreeBytes() {
  const uint8_t *p = &_end;
  uint16_t freeCount = 0;
  while (p <= &__stack && *p == 0xC5) {
    freeCount++;
    p++;
  }
  return freeCount;
}

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
  // "Good morning" — the droid says hello on every power-up. Motors are stopped.
  displayBitmap(kIconHeart);
  chirpSweep(700, 1600, 110, 3);
  chirpRandomR2();
  displayBitmap(kIconBluetooth);
}

extern bool gProbeServicedThisPass;

void loop() {
  gProbeServicedThisPass = false; // F4: one '?' or '=' probe per pass, at most
  serviceChirpNB(); // advance any chirp that is playing while the wheels turn
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

  pinMode(kPinLdrFollow, INPUT);
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
  gCliffRecoveryAttempts = 0;
  gCliffForwardDriveStart = 0;
  gCliffFault = false;
  resetFollowHeartbeat(); // B2: a mode change stops the pulse and clears D13

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
    case MODE_FOCUS_CLOCK:
      // The countdown starts the instant the mode is engaged and owns the face
      // until another mode takes over. No motors, ever.
      gFocusStartMs = millis();
      gFocusDone = false;
      displayBitmap(kIconStandby);
      break;
  }

  // R2-D2 mode-change voice: motors are already halted above, so a short
  // blocking chirp here can never delay a stop.
  chirpModeChange((uint8_t)newMode);
}

/* ============================================================================
 * SECTION 7: Universal Command Parser & Bluetooth Protocol
 * ============================================================================
 */

/**
 * Safe Framed Payload Reader:
 * Reads characters until '\n' into buffer (null-terminated).
 *
 * F1 — ABSOLUTE DEADLINE: the timeout is measured from the FIRST call, never reset
 * per byte, so a sender dripping one byte per 100 ms can no longer hold the loop
 * (and both watchdogs) hostage. On deadline expiry OR buffer overflow the reader
 * gives up immediately and arms gDiscardToNewline: every byte after that is
 * consumed and thrown away by the parser until a '\n' arrives, so a late tail can
 * never be re-read as opcodes on a later loop pass.
 * A complete frame still returns normally with its length.
 */
bool gDiscardToNewline = false;

size_t readFramedPayload(char* buffer, size_t maxLen, unsigned long timeoutMs) {
  if (maxLen > 0) buffer[0] = '\0';
  size_t count = 0;
  unsigned long start = millis(); // ABSOLUTE deadline — never reset per byte
  while (millis() - start < timeoutMs) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n') {
        if (count < maxLen) buffer[count] = '\0';
        else if (maxLen > 0) buffer[maxLen - 1] = '\0';
        return count;
      }
      if (c == '\r') continue; // Ignore CR
      if (count < maxLen - 1) {
        buffer[count++] = c;
      } else {
        // Oversized frame: stop parsing NOW and drop the rest of the line.
        if (maxLen > 0) buffer[0] = '\0';
        gDiscardToNewline = true;
        return 0;
      }
    }
  }
  // Deadline hit before '\n': reject payload and drop whatever tail follows.
  if (maxLen > 0) buffer[0] = '\0';
  gDiscardToNewline = true;
  return 0;
}
// F4: at most ONE pulseIn-heavy probe ('?' or '=') is serviced per loop pass, and the
// parser reads at most kParserBytesPerPass bytes per pass, so a flooded link can never
// starve executeCurrentMode() or either watchdog. Reset at the top of loop().
bool gProbeServicedThisPass = false;
const uint8_t kParserBytesPerPass = 32;

void checkControlInput() {
  uint8_t bytesThisPass = 0;
  while (Serial.available() > 0 && bytesThisPass < kParserBytesPerPass) {
    bytesThisPass++;

    // F1: a rejected frame's tail is swallowed here — never parsed as opcodes.
    if (gDiscardToNewline) {
      char d = (char)Serial.read();
      // An 'S' still brakes even while a rejected frame is being flushed.
      if (d == 'S' || d == 's') {
        haltMotors(); stopAllAudio();
        gCurrentMode = MODE_BLUETOOTH_RC;
        gMotorActive = false;
        gFollowLockAcquired = false;
        displayBitmap(kIconBrake);
      }
      if (d == '\n') gDiscardToNewline = false;
      continue;
    }

    char ch = Serial.read();
    gLastIdleChirpTime = millis(); // Reset idle timer on any user touch

    // F5: 🛑 EMERGENCY BRAKE IS THE VERY FIRST THING AFTER Serial.read().
    // It used to sit behind the HM-10 banner filter, so an 'S' arriving mid-banner
    // could be swallowed. Nothing is allowed in front of the stop any more.
    if (ch == 'S' || ch == 's') {
      haltMotors();
      stopAllAudio();
      gCurrentMode = MODE_BLUETOOTH_RC;
      gMotorActive = false;
      gFollowLockAcquired = false;
      displayBitmap(kIconBrake);
      digitalWrite(kPinStatusLed, LOW);
      chirpBrake();
      continue;
    }

    // 0. FILTER ALL HM-10 BLE STATUS BANNERS (e.g. "+CONNECTED", "OK+CONN", "OK+LOST")
    if (ch == '+' || ch == 'O' || ch == 'o') {
      char bannerBuf[24];
      uint8_t bLen = 0;
      bannerBuf[bLen++] = ch;
      bool stopInBanner = false;
      unsigned long bannerStart = millis();
      while (millis() - bannerStart < 35) {
        if (Serial.available()) {
          bannerStart = millis();
          char b = Serial.read();
          // F5: an 'S' inside the collected text stops the drain AND the wheels.
          if (b == 'S' || b == 's') { stopInBanner = true; break; }
          if (b == '\n' || b == '\r') break;
          if (bLen < sizeof(bannerBuf) - 1) bannerBuf[bLen++] = b;
        }
      }
      bannerBuf[bLen] = '\0';

      // 🛑 CRITICAL LINK-LOSS INTERLOCK: If Bluetooth link died, HALT MOTORS IMMEDIATELY!
      if (stopInBanner || strstr(bannerBuf, "LOST") != NULL || strstr(bannerBuf, "DISC") != NULL) {
        haltMotors();
        stopAllAudio();
        gCurrentMode = MODE_BLUETOOTH_RC;
        gMotorActive = false;
        if (stopInBanner) gFollowLockAcquired = false;
        displayBitmap(kIconBrake);
      }
      continue; // Swallowed banner completely — never triggers rogue motor or mood action!
    }

    // 2. MASTER AUDIO MUTE CONTROL ('x' = Absolute Mute, 'X' = Absolute Unmute)
    if (ch == 'x') {
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
      char tBuf[8];
      size_t n = readFramedPayload(tBuf, sizeof(tBuf));
      if (n > 0) {
        char letter = tBuf[0];
        if (letter == '.') playMorseDit();
        else if (letter == '-') playMorseDah();
        else playMorseLetter(letter);
      }
    }
    // 10. Chiptune Jukebox Payloads ('J' + 1-4 + '\n') (Guarded Payload Isolation)
    else if (ch == 'J' || ch == 'j') {
      haltMotors();
      gMotorActive = false;
      char jBuf[8];
      size_t n = readFramedPayload(jBuf, sizeof(jBuf));
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
      char mBuf[24];
      size_t n = readFramedPayload(mBuf, sizeof(mBuf));
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
      chirpSweep(1800, 1100, 90, 3);
      delay(20);
      chirpSweep(1300, 2400, 110, 3);
    }
    // 13. Gemini Vision AI Target Lock-On ('V')
    else if (ch == 'V' || ch == 'v') {
      chirpVisionLock();
    }
    // 14. Real-time Variable Speed Throttle ('P' + digits) - NON-BLOCKING & DISCARD-SAFE
    // This is the cockpit slider's opcode and NOTHING else: it never changes mode.
    // The Focus Clock has its own framed opcode '^' below.
    else if (ch == 'P' || ch == 'p') {
      int targetSpeed = 0;
      unsigned long pStart = millis();
      while (millis() - pStart < 25) {
        if (Serial.available()) {
          char digit = Serial.peek();
          if (digit >= '0' && digit <= '9') {
            targetSpeed = targetSpeed * 10 + (Serial.read() - '0');
          } else {
            // F2: the throttle frame's tail is CONSUMED, never left for the parser.
            // "P120F\n" used to set the speed and then execute the 'F'. A newline or
            // CR simply ends the frame; ANY other byte means the frame is malformed,
            // so the rest of that line is dropped and can never become a motion command.
            Serial.read();
            if (digit != '\n' && digit != '\r') {
              gDiscardToNewline = true;
              // An 'S' is still never eaten: it brakes even inside a bad frame.
              if (digit == 'S' || digit == 's') {
                haltMotors(); stopAllAudio();
                gCurrentMode = MODE_BLUETOOTH_RC;
                gMotorActive = false;
                gFollowLockAcquired = false;
                displayBitmap(kIconBrake);
              }
            }
            break;
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
      runKnightLPathManeuver();
    }
    // 16. Dedicated Real-Time Face Clock ('@' + time string + '\n') - Fixed buffer, zero malloc
    else if (ch == '@') {
      haltMotors();
      gCurrentMode = MODE_BLUETOOTH_RC;
      char clockBanner[24];
      size_t n = readFramedPayload(clockBanner, sizeof(clockBanner));
      char* p = clockBanner;
      while (*p == ' ' || *p == '\r' || *p == '\t') p++;
      int endIdx = (int)strlen(p) - 1;
      while (endIdx >= 0 && (p[endIdx] == ' ' || p[endIdx] == '\r' || p[endIdx] == '\t')) {
        p[endIdx--] = '\0';
      }
      if (strlen(p) > 0) {
        char banner[32];
        uint8_t pLen = strlen(p);
        if (pLen > sizeof(banner) - 3) pLen = sizeof(banner) - 3;
        banner[0] = ' ';
        memcpy(banner + 1, p, pLen);
        banner[pLen + 1] = ' ';
        banner[pLen + 2] = '\0';
        haltMotors();
        scrollTextAcrossMatrix(banner, 45);
        haltMotors();
        displayBitmap(kIconCrown);
      }
    }
    // 17. Text Marquee Banner Streamer ('W' + string + '\n') - Fixed buffer, zero malloc
    else if (ch == 'W') {
      haltMotors();
      char textBanner[24];
      size_t n = readFramedPayload(textBanner, sizeof(textBanner));
      char* p = textBanner;
      while (*p == ' ' || *p == '\r' || *p == '\t') p++;
      int endIdx = (int)strlen(p) - 1;
      while (endIdx >= 0 && (p[endIdx] == ' ' || p[endIdx] == '\r' || p[endIdx] == '\t')) {
        p[endIdx--] = '\0';
      }
      if (strlen(p) > 0) {
        char banner[32];
        uint8_t pLen = strlen(p);
        if (pLen > sizeof(banner) - 3) pLen = sizeof(banner) - 3;
        banner[0] = ' ';
        memcpy(banner + 1, p, pLen);
        banner[pLen + 1] = ' ';
        banner[pLen + 2] = '\0';
        scrollTextAcrossMatrix(banner, 45);
        displayBitmap(kIconCrown);
      }
    }
    // 18. Serene Static 8x8 Digital Clock ('#' + "HHMM\n") - Validated digits & clamped
    else if (ch == '#') {
      haltMotors();
      gCurrentMode = MODE_BLUETOOTH_RC;
      char timeDigits[12];
      size_t n = readFramedPayload(timeDigits, sizeof(timeDigits));
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
      runSayingYesMood();
    }
    else if (ch == 'Y' || ch == 'y') {
      runEcstasyMood();
    }
    else if (ch == 'D' || ch == 'd') {
      runGrumpyMood();
    }
    else if (ch == 'C' || ch == 'c') {
      runFatiguedMood();
    }
    // 21. Telemetry & EEPROM Diagnostics Query ('?')
    // F3: '?' is a QUERY, not a keepalive — it must not keep an unattended
    // autonomous mission alive. Only '!' / '*' and mode entry refresh the deadman.
    // F4: at most one pulseIn-heavy probe per loop pass.
    else if (ch == '?') {
      if (gProbeServicedThisPass) continue;
      gProbeServicedThisPass = true;
      uint16_t missionCount = 0;
      EEPROM.get(EEPROM_RUNS_ADDR, missionCount);
      Serial.print(F("WALL-E|GEAR:"));
      Serial.print(gCurrentGear);
      Serial.print(F("|MUTE:"));
      Serial.print(gMasterMute ? 1 : 0);
      Serial.print(F("|BOOTS:"));
      Serial.print(missionCount);
      Serial.print(F("|LIGHT:"));
      Serial.print(analogRead(kPinLdrFollow));
      // Why won't it move? Everything the safety supervisor looks at, in one line.
      Serial.print(F("|FLOOR:"));
      Serial.print(digitalRead(kPinLineSensorLeft));   // 1 = floor seen, 0 = edge/dark
      Serial.print(digitalRead(kPinLineSensorRight));
      Serial.print(F("|SONAR:"));
      Serial.print(getDistanceCm());                  // 0 = blind
      Serial.print(F("|FAULT:"));
      Serial.print(gCliffFault ? 'C' : '-');
      Serial.print(gSonarFault ? 'S' : '-');
      Serial.print(F("|GUARD:"));
      Serial.print(gDeskMode ? 'D' : 'F');            // D = desk (edge veto on), F = floor
      Serial.print(F("|MODE:"));
      Serial.print((uint8_t)gCurrentMode);
      Serial.print(F("|FREE:"));
      Serial.println(getStackFreeBytes());
    }
    // 24. Floor / Desk switch ('%' + 'D' or 'F' + '\n'). Framed and drained like every
    // payload. 'D' arms the edge veto for the table; 'F' (the boot default) frees the
    // robot to play on any floor. Desk Companion ignores this and always guards.
    else if (ch == '%') {
      char modeBuf[4];
      size_t n = readFramedPayload(modeBuf, sizeof(modeBuf));
      if (n > 0) {
        if (modeBuf[0] == 'D' || modeBuf[0] == 'd') gDeskMode = true;
        else if (modeBuf[0] == 'F' || modeBuf[0] == 'f') gDeskMode = false;
      }
    }
    // 25. Focus Clock ('^' + minutes + '\n', 1-99). Its own opcode, deliberately
    // NOT the throttle 'P': a slow slider value must never be able to start a
    // clock in the middle of a drive. Framed and drained to '\n' exactly like
    // '~' and '%', so the digits can never leak back into the parser.
    else if (ch == '^') {
      char fBuf[6];
      size_t n = readFramedPayload(fBuf, sizeof(fBuf));
      if (n > 0) {
        char* p = fBuf;
        while (*p == ' ' || *p == '\t') p++;
        if (isdigit(*p)) {
          uint16_t mins = 0;
          uint8_t digits = 0;
          while (isdigit(*p) && digits < 2) { mins = mins * 10 + (*p - '0'); p++; digits++; }
          if (mins >= 1 && mins <= 99) {
            gFocusMinutes = (uint8_t)mins;
            setMode(MODE_FOCUS_CLOCK);
          }
        }
      }
    }
    // 23. Echo Lab probe ('='): one ping, raw flight time in microseconds + the cm the
    // droid computes from it. Shivansh measures the wall with a tape and works out the
    // speed of sound himself: speed = 2 x distance / time. No motion, no mode change.
    else if (ch == '=') {
      if (gProbeServicedThisPass) continue; // F4: one probe per loop pass
      gProbeServicedThisPass = true;
      uint16_t cm = getDistanceCm(); // same read as every mode, same fault latches
      Serial.print(F("ECHO:"));
      Serial.print(gLastEchoMicros);
      Serial.print(F("us|CM:"));
      Serial.println(cm);
    }
    // 22. Phone Heading Hint ('~' + signed degrees + '\n') — DATA, NOT A COMMAND.
    // When the pilot turns his phone during Follow Me, the cockpit whispers
    // which way he turned. This ONLY biases the bearing memory, so a droid
    // that has lost him searches the right side first. It never touches a
    // motor, never changes mode, and is ignored outside MODE_LIVING_PET.
    // The payload is always drained to '\n' first, so its digits can never
    // leak back into the parser as mode or motion commands.
    // Deliberately does NOT refresh gLastKeepaliveTime: a passive hint must
    // not be able to keep an unattended mission alive. The '!' keepalive at
    // 600ms already covers mode 5.
    else if (ch == '~') {
      char hintBuf[8];
      size_t n = readFramedPayload(hintBuf, sizeof(hintBuf));
      if (n > 0 && gCurrentMode == MODE_LIVING_PET) {
        char* p = hintBuf;
        while (*p == ' ' || *p == '\t') p++;
        int8_t sign = 1;
        if (*p == '-') { sign = -1; p++; }
        else if (*p == '+') p++;
        if (isdigit(*p)) {
          int16_t deg = 0;
          uint8_t digits = 0;
          while (isdigit(*p) && digits < 3) { deg = deg * 10 + (*p - '0'); p++; digits++; }
          deg = constrain(deg, 0, 90) * sign;
          if (deg <= -15 || deg >= 15) {
            gLastBearing.bearing = (deg < 0) ? BEARING_LEFT : BEARING_RIGHT;
            gLastBearing.confidence = 45; // moderate: a hint, not a sighting
            gLastBearing.timestamp = millis();
          }
        }
      }
    }
  }
}

void executeCurrentMode() {
  // 🛑 1. AUTONOMOUS DEADMAN: Evaluated on MODE ALONE, ungated by gMotorActive!
  // MODE_FOCUS_CLOCK joins STANDBY and RC in the exemption: it never drives a
  // motor, so it must survive the phone locking and the keepalive going quiet.
  if (gCurrentMode != MODE_STANDBY && gCurrentMode != MODE_BLUETOOTH_RC &&
      gCurrentMode != MODE_FOCUS_CLOCK) {
    if (millis() - gLastKeepaliveTime > 1500) {
      haltMotors();
      gMotorActive = false;
      setMode(MODE_BLUETOOTH_RC);
      displayBitmap(kIconBrake);
      return;
    }
  }

  // 🛑 2. RC DEADMAN: Evaluated in Bluetooth RC mode when motors are active
  if (gCurrentMode == MODE_BLUETOOTH_RC && gMotorActive) {
    if (millis() - gLastMotionCmdTime > 400) {
      haltMotors();
      gMotorActive = false;
    }
  }

  // 🛑 3. FIRMWARE INDEPENDENT SAFETY SUPERVISOR: Live cliff and sonar veto
  if (gCurrentMode == MODE_BLUETOOTH_RC && gMotorActive) {
    uint8_t leftCliff  = digitalRead(kPinLineSensorLeft);
    uint8_t rightCliff = digitalRead(kPinLineSensorRight);
    uint16_t dist = getDistanceCm();
    if (edgeSeen(leftCliff, rightCliff) || (dist > 0 && dist < 15)) {
      haltMotors();
      gMotorActive = false;
      displayBitmap(kIconBrake);
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
    case MODE_FOCUS_CLOCK:        runFocusClockMode(); break;
  }
}

/**
 * Polled non-blocking motor delay helper:
 * Polls serial, keepalive expiry, cliff sensors, and sonar every 8-10ms.
 * Returns false immediately if any stop condition or safety hazard triggers,
 * cutting motor power and aborting the maneuver with <10ms latency.
 */
bool safeMotorDelay(unsigned long ms, bool checkCliff, bool checkSonar) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    // 1. Immediate Serial Abort ('S' or any incoming command)
    if (Serial.available()) {
      haltMotors();
      gMotorActive = false;
      return false;
    }

    // 2. Autonomous Keepalive Expiry Check (same exemption set as executeCurrentMode)
    if (gCurrentMode != MODE_STANDBY && gCurrentMode != MODE_BLUETOOTH_RC &&
        gCurrentMode != MODE_FOCUS_CLOCK) {
      if (millis() - gLastKeepaliveTime > 1500) {
        haltMotors();
        gMotorActive = false;
        setMode(MODE_BLUETOOTH_RC);
        displayBitmap(kIconBrake);
        return false;
      }
    }

    // 3. Cliff Sensor Live Veto
    if (checkCliff) {
      uint8_t leftCliff  = digitalRead(kPinLineSensorLeft);
      uint8_t rightCliff = digitalRead(kPinLineSensorRight);
      if (edgeSeen(leftCliff, rightCliff)) {
        haltMotors();
        gMotorActive = false;
        return false;
      }
    }

    // 4. Ultrasonic Sonar Obstacle Live Veto
    if (checkSonar) {
      uint16_t dist = getDistanceCm();
      if (gSonarFault || (dist > 0 && dist < 18)) {
        haltMotors();
        gMotorActive = false;
        return false;
      }
    }

    delay(8);
  }
  return true;
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
  if (edgeSeen(leftCliff, rightCliff)) {
    haltMotors();
    talkAstromech(EMOTION_ALERT);
    driveBackward(kAutoDriveSpeed);
    if (!safeMotorDelay(280, false, false)) { haltMotors(); return; }
    haltMotors();
    pivotRight(kAutoTurnSpeed);
    if (!safeMotorDelay(320, false, false)) { haltMotors(); return; }
    haltMotors();
    return;
  }

  uint16_t dist = getDistanceCm();
  if (Serial.available()) { haltMotors(); return; }

  // FAIL-SAFE: a PROVEN electrical fault (stuck-HIGH echo pin) still halts.
  if (gSonarFault) {
    haltMotors();
    displayBitmap(kIconBrake);
    if (millis() % 1200 < 60) talkAstromech(EMOTION_ALERT);
    safeMotorDelay(40, true, false);
    return;
  }

  // 0. NOTHING AHEAD WITHIN RANGE (no echo). This is the middle of a big room,
  // not a broken sensor: creep forward slowly and re-ping often. safeMotorDelay's
  // sonar veto still fires the instant a real return under 18cm appears.
  // F7: ...but the creep gets a TIME BUDGET. A real room always returns an echo
  // within a few seconds; six seconds of unbroken silence means the sonar is gone,
  // not that the room is big. Stop, say so, and fall back to Standby.
  static unsigned long creepStart = 0;
  if (dist != 0) {
    creepStart = 0; // a real ping resets the budget
  } else {
    if (creepStart == 0) creepStart = millis();
    else if (millis() - creepStart > 6000UL) {
      creepStart = 0;
      haltMotors();
      chirpAlert(); // respects gMasterMute internally
      displayBitmap(kIconStandby);
      setMode(MODE_STANDBY);
      return;
    }
    displayBitmap(kArrowForward);
    driveForward(140); // deliberately slower than the 175 cruise
    safeMotorDelay(20, true, true);
    return;
  }

  // 1. Close-Range Emergency Reflex (Enhanced 18cm cushion for thin chair legs!)
  if (dist > 0 && dist < 18) {
    haltMotors();
    talkAstromech(EMOTION_ALERT); // Startled reflex chirp!
    chirpRandomR2();              // "turning away!" — wheels are stopped here
    driveBackward(kAutoDriveSpeed);
    if (!safeMotorDelay(280, false, false)) { haltMotors(); return; }
    haltMotors();
    pivotRight(kAutoTurnSpeed);
    if (!safeMotorDelay(320, true, false)) { haltMotors(); return; }
    haltMotors();
  }
  // 2. Proactive Look-Ahead Radar (36cm detection cushion with Doppler Radar Echolocation)
  else if (dist > 0 && dist < 36) {
    haltMotors();
    talkDialogue(PHRASE_HESITATION); // Inquisitive "Hmm... checking path options"
    chirpRandomR2();                 // "turning away!" — wheels are stopped here
    displayBitmap(kArrowLeft);
    pivotLeft(kAutoTurnSpeed);
    if (!safeMotorDelay(260, true, false)) { haltMotors(); return; }
    haltMotors();
    if (!safeMotorDelay(50, true, false)) return;
    uint16_t leftDist = getDistanceCm();
    playDopplerRadarPing(leftDist); // 🌊 Doppler acoustic distance ping
    if (!safeMotorDelay(30, true, false)) return;

    displayBitmap(kArrowRight);
    pivotRight(kAutoTurnSpeed);
    if (!safeMotorDelay(520, true, false)) { haltMotors(); return; }
    haltMotors();
    if (!safeMotorDelay(50, true, false)) return;
    uint16_t rightDist = getDistanceCm();
    playDopplerRadarPing(rightDist); // 🌊 Doppler acoustic distance ping
    if (!safeMotorDelay(30, true, false)) return;

    if (leftDist > rightDist && leftDist > 30) {
      displayBitmap(kArrowLeft);
      pivotLeft(kAutoTurnSpeed);
      if (!safeMotorDelay(520, true, false)) { haltMotors(); return; }
      haltMotors();
    }
  }
  // 3. Clear Path Forward
  else {
    displayBitmap(kArrowForward);
    driveForward(kAutoDriveSpeed);
    safeMotorDelay(25, true, true);
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
    case 'F': case 'f': {
      uint8_t leftCliff  = digitalRead(kPinLineSensorLeft);
      uint8_t rightCliff = digitalRead(kPinLineSensorRight);
      uint16_t dist = getDistanceCm();
      if (edgeSeen(leftCliff, rightCliff) || (dist > 0 && dist < 15)) {
        haltMotors();
        gMotorActive = false;
        displayBitmap(kIconBrake);
      } else {
        displayBitmap(kArrowForward);
        driveForward(gDriveSpeed);
      }
      break;
    }
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
 * Mode 5: Follow Me (Three-Sensor Fusion: LDR on A0, Sonar on D2/D3, PIR on D12)
 * Full Graceful Degradation Matrix & Continuous Bacterial Chemotaxis Arc-Drive
 */
void runLivingPetEngine() {
  uint16_t dist = getFilteredDistance();
  uint16_t rawDist = gLatestRawDist;
  if (Serial.available()) { haltMotors(); return; }

  const uint16_t kSafeStopDist = 25; // Unfiltered nearest obstacle stop (immediate brake/standby within 25cm)
  const uint16_t kSweetSpotMax = 32; // Sweet spot up to 32cm
  const uint16_t kMaxLeashDist = 80; // Dynamic human leg tracking up to 80cm

  static unsigned long lastLostTime = 0;
  static unsigned long lastChirpTrillTime = 0;
  static int scanDir = 1;
  static uint16_t lastValidDist = 0;
  static int lastLdrVal = 0;
  static int8_t arcDirection = 1; // +1 = right arc bias, -1 = left arc bias
  static int16_t lastFollowErr = 0;

  // 1. Sample Sensors & Evaluate Health (B4)
  int currentLdr = sampleLdrWithHealth();
  int ldrDelta = currentLdr - lastLdrVal;
  lastLdrVal = currentLdr;

  bool pirMotion = checkPirMotionDetected();
  bool sonarHealthy = !gSonarFault;
  bool ldrHealthy   = gLdrHealthy;
  bool pirHealthy   = gPirHealthy;

  // PIR gate: if PIR is healthy, gate tracking on motion seen within 4 seconds;
  // if PIR is down/absent, assume human is present (B3 degradation matrix)
  bool humanConfirmed = true;
  if (pirHealthy) {
    static unsigned long lastMotionSeen = 0;
    if (pirMotion) lastMotionSeen = millis();
    humanConfirmed = (millis() - lastMotionSeen < 4000UL);
  } else {
    humanConfirmed = true;
  }

  // 2. DEGRADATION MATRIX: ALL THREE DOWN -> Halt, chirp, standby (Never guess)
  if (!ldrHealthy && !sonarHealthy && !pirHealthy) {
    haltMotors();
    chirpAlert();
    gFollowLockAcquired = false;
    displayBitmap(kIconStandby);
    safeMotorDelay(100, false, false);
    return;
  }

  // F6: the sonar has answered "nothing" kSonarZeroLost times in a row. That is not a
  // reading, it is silence — drop the lock, stop, and let the handshake ritual below
  // re-acquire. The sonar is NOT marked faulted here: only a stuck-HIGH echo pin does
  // that, and the single-sensor degradation branch (§4) still keys on gSonarFault alone.
  if (sonarHealthy && gFollowLockAcquired && gSonarZeroRun >= kSonarZeroLost) {
    haltMotors();
    gFollowLockAcquired = false;
    displayBitmap(kIconStandby);
  }

  // 3. Initial Handshake Lock Ritual (Wait for Pilot to stand in front before rolling!)
  if (!gFollowLockAcquired) {
    // F9: reset the pulse only on the lock -> unlocked TRANSITION. It used to run on
    // every unlocked pass, which cleared D13 one pass after the lost-timeout branch
    // below lit it, so the "where did you go?" blink was never visible.
    if (gPrevFollowLocked) resetFollowHeartbeat(); // B2: no lock, no pulse — D13 LOW
    gPrevFollowLocked = false;
    haltMotors();
    displayBitmap(kIconStandby);

    // Interrogative acoustic pulse
    if (millis() % 700 < 40) {
      if (!gMasterMute) { tone(kPinBuzzer1, 1400); delay(20); noTone(kPinBuzzer1); }
    }

    bool lockTriggered = false;
    if (sonarHealthy && dist >= 10 && dist <= kMaxLeashDist) {
      lockTriggered = true;
    } else if (!sonarHealthy && ldrHealthy && currentLdr > 250) {
      lockTriggered = true; // Sonar down: lock on optical beacon
    }

    if (lockTriggered) {
      // Explicit cliff check before handshake nod
      if (edgeSeen(digitalRead(kPinLineSensorLeft), digitalRead(kPinLineSensorRight))) {
        haltMotors();
        gFollowLockAcquired = false;
        displayBitmap(kIconCliffGuard);
        return;
      }

      // ✅ TARGET ACQUIRED! Complete Follow Me Handshake Ceremony
      gFollowLockAcquired = true;
      haltMotors();
      displayBitmap(kIconFollowPuppy);
      safeMotorDelay(180, false, false);
      displayBitmap(kIconCrown);
      // Eager 1cm nod toward pilot (cliff guarded + polled delay)
      driveForward(120);
      if (!safeMotorDelay(50, true, false)) {
        haltMotors();
        gFollowLockAcquired = false;
        return;
      }
      haltMotors();
      chirpVisionLock();   // "I see you" — lock acquired
      chirpRandomR2();
      lastLostTime = millis();
      lastChirpTrillTime = millis();
      lastValidDist = dist;
    }
    safeMotorDelay(40, false, false);
    return;
  }

  // B2: Lock is held — the droid has a pulse. D13 beats for the whole of Follow
  // Me; the matrix heart below is used only by the two resting branches (§5
  // close-range and §6 sweet spot), so arrows, brake and standby are untouched.
  gPrevFollowLocked = true; // F9: mark the lock so the next unlock is a real transition
  const uint8_t* heartFrame = tickFollowHeartbeat(sonarHealthy ? dist : 0);

  // 4. DEGRADATION MATRIX: SONAR DOWN -> LDR bearing + PIR gate, NO range: cap speed, shorten legs, stop often
  if (!sonarHealthy && ldrHealthy) {
    if (!humanConfirmed) {
      haltMotors();
      displayBitmap(kIconStandby);
      safeMotorDelay(40, false, false);
      return;
    }
    // Explicit cliff check before moving
    if (edgeSeen(digitalRead(kPinLineSensorLeft), digitalRead(kPinLineSensorRight))) {
      haltMotors(); gFollowLockAcquired = false; displayBitmap(kIconCliffGuard); return;
    }

    // LDR gradient run-and-tumble: compare change over time
    if (ldrDelta > 4) {
      // Brightness rising -> keep current arc direction
    } else if (ldrDelta < -4) {
      // Brightness falling -> invert arc direction
      arcDirection = -arcDirection;
    }
    FollowBearing ldrB = (arcDirection > 0) ? BEARING_RIGHT : BEARING_LEFT;

    uint8_t crawlSpeed = 115;
    uint8_t lPwm = crawlSpeed;
    uint8_t rPwm = crawlSpeed;
    if (ldrB == BEARING_LEFT) { lPwm = 90; rPwm = 135; }
    else if (ldrB == BEARING_RIGHT) { lPwm = 135; rPwm = 90; }

    displayBitmap(kArrowForward);
    arcDrive(lPwm, rPwm);
    if (!safeMotorDelay(25, true, false)) { haltMotors(); gFollowLockAcquired = false; return; }
    haltMotors(); // Short leg, stop often!
    safeMotorDelay(35, false, false);
    lastLostTime = millis();
    return;
  }

  // 5. IMMEDIATE UNFILTERED COLLISION STOP (< 25cm -> Immediate brake, zero median delay!)
  if (sonarHealthy && rawDist > 0 && rawDist < kSafeStopDist) {
    haltMotors();
    if (rawDist < 12) {
      displayBitmap(heartFrame); // B2: beating heart, fastest at his feet
      if (millis() - lastChirpTrillTime > 2500) {
        talkAstromech(EMOTION_HAPPY);
        lastChirpTrillTime = millis();
      }
    } else {
      displayBitmap(kIconBrake);
      stopAllAudio();
    }
    lastLostTime = millis();
    lastValidDist = rawDist;
    safeMotorDelay(40, false, false);
    return;
  }

  // 6. Follow Me Sweet Spot (25cm - 32cm -> Standby & Subtle Flank Scan)
  if (sonarHealthy && dist >= kSafeStopDist && dist <= kSweetSpotMax) {
    haltMotors();
    displayBitmap(heartFrame); // B2: resting in the sweet spot, breathing
    stopAllAudio();
    lastLostTime = millis();
    lastValidDist = dist;

    // Subtle Follow Me Flank Scan every 2400ms (calibrated 15° at PWM 185)
    static unsigned long lastFlankScan = 0;
    if (millis() - lastFlankScan > 2400) {
      lastFlankScan = millis();
      const uint16_t flankMs = 15 * kPivotMsPerDegree; // 15° = 45ms at PWM 185
      pivotLeft(kPivotCalibratedPwm);
      if (!safeMotorDelay(flankMs, true, false)) { haltMotors(); gFollowLockAcquired = false; return; }
      haltMotors();
      safeMotorDelay(40, false, false);
      pivotRight(kPivotCalibratedPwm);
      if (!safeMotorDelay(flankMs * 2, true, false)) { haltMotors(); gFollowLockAcquired = false; return; }
      haltMotors();
      safeMotorDelay(40, false, false);
      pivotLeft(kPivotCalibratedPwm);
      if (!safeMotorDelay(flankMs, true, false)) { haltMotors(); gFollowLockAcquired = false; return; }
      haltMotors();
    }
    safeMotorDelay(30, false, false);
  }
  // 7. Human Walking Away (32cm - 80cm -> Three-Sensor Fusion Arc-Drive Follow)
  else if (sonarHealthy && dist > kSweetSpotMax && dist <= kMaxLeashDist) {
    // Explicit cliff check immediately before forward leg
    if (edgeSeen(digitalRead(kPinLineSensorLeft), digitalRead(kPinLineSensorRight))) {
      haltMotors();
      gFollowLockAcquired = false;
      displayBitmap(kIconCliffGuard);
      return;
    }

    if (!humanConfirmed) {
      haltMotors();
      displayBitmap(kIconStandby);
      safeMotorDelay(40, false, false);
      return;
    }

    // Determine Bearing & Confidence from Sensors (B1, B2, B3)
    FollowBearing ldrBearing = BEARING_CENTER;
    uint8_t ldrConfidence = 0;

    if (ldrHealthy) {
      // B1: Continuous run-and-tumble gradient comparison (chemotaxis)
      const int kGradThreshold = 4;
      if (ldrDelta > kGradThreshold) {
        // Brightness rising -> curving toward torch beacon
        ldrBearing = (arcDirection > 0) ? BEARING_RIGHT : BEARING_LEFT;
        ldrConfidence = constrain(ldrDelta * 8, 25, 85);
      } else if (ldrDelta < -kGradThreshold) {
        // Brightness falling -> curving away, invert arc
        arcDirection = -arcDirection;
        ldrBearing = (arcDirection > 0) ? BEARING_RIGHT : BEARING_LEFT;
        ldrConfidence = constrain((-ldrDelta) * 8, 25, 85);
      } else {
        ldrBearing = BEARING_CENTER;
        ldrConfidence = 20;
      }
    }

    // B3: DEGRADATION MATRIX — LDR Down -> Fall back to Sonar Conical Scan-and-Compare
    FollowBearing sonarBearing = BEARING_CENTER;
    uint8_t sonarConfidence = 0;

    if (!ldrHealthy && sonarHealthy) {
      static unsigned long lastConicalScan = 0;
      if (millis() - lastConicalScan > 800UL) {
        lastConicalScan = millis();
        const uint16_t kConicalMs = 20 * kPivotMsPerDegree; // 20° = 60ms at PWM 185
        
        uint16_t dCenter = getDistanceCm();
        
        pivotLeft(kPivotCalibratedPwm);
        if (!safeMotorDelay(kConicalMs, true, false)) { haltMotors(); gFollowLockAcquired = false; return; }
        haltMotors();
        safeMotorDelay(20, false, false);
        uint16_t dLeft = getDistanceCm();
        
        pivotRight(kPivotCalibratedPwm);
        if (!safeMotorDelay(kConicalMs, true, false)) { haltMotors(); gFollowLockAcquired = false; return; }
        haltMotors();
        safeMotorDelay(20, false, false);
        
        pivotRight(kPivotCalibratedPwm);
        if (!safeMotorDelay(kConicalMs, true, false)) { haltMotors(); gFollowLockAcquired = false; return; }
        haltMotors();
        safeMotorDelay(20, false, false);
        uint16_t dRight = getDistanceCm();
        
        pivotLeft(kPivotCalibratedPwm);
        if (!safeMotorDelay(kConicalMs, true, false)) { haltMotors(); gFollowLockAcquired = false; return; }
        haltMotors();
        safeMotorDelay(20, false, false);

        // Steer to shortest return
        uint16_t minD = 999;
        FollowBearing bestB = BEARING_CENTER;
        if (dCenter > 0 && dCenter < minD) { minD = dCenter; bestB = BEARING_CENTER; }
        if (dLeft > 0 && dLeft < minD)     { minD = dLeft; bestB = BEARING_LEFT; }
        if (dRight > 0 && dRight < minD)   { minD = dRight; bestB = BEARING_RIGHT; }

        if (minD <= kMaxLeashDist) {
          sonarBearing = bestB;
          sonarConfidence = 75;
        }
      } else {
        sonarBearing = gLastBearing.bearing;
        sonarConfidence = (gLastBearing.confidence > 20) ? gLastBearing.confidence - 10 : 0;
      }
    } else {
      sonarBearing = BEARING_CENTER;
      sonarConfidence = 40;
    }

    // B2: Confidence-Weighted Fusion Vote
    FollowBearing fusedBearing = BEARING_CENTER;
    int16_t totalVote = 0;
    int16_t totalWeight = 0;

    if (ldrHealthy && ldrConfidence > 0) {
      totalVote += (int16_t)ldrBearing * ldrConfidence;
      totalWeight += ldrConfidence;
    }
    if (sonarConfidence > 0) {
      totalVote += (int16_t)sonarBearing * sonarConfidence;
      totalWeight += sonarConfidence;
    }

    if (totalWeight > 0) {
      if (totalVote > 15) fusedBearing = BEARING_RIGHT;
      else if (totalVote < -15) fusedBearing = BEARING_LEFT;
      else fusedBearing = BEARING_CENTER;

      // Update Bearing Memory (B5)
      gLastBearing.bearing = fusedBearing;
      gLastBearing.confidence = constrain(totalWeight / 2, 10, 90);
      gLastBearing.timestamp = millis();
    } else if (millis() - gLastBearing.timestamp < 3000UL) {
      fusedBearing = gLastBearing.bearing; // Use memory within 3 seconds
    }

    displayBitmap(kArrowForward);

    // B5: PD Controller on distance error: target distance 28cm (pure integer math)
    const int16_t kTargetDist = 28;
    int16_t err = (int16_t)dist - kTargetDist;
    int16_t dErr = err - lastFollowErr;
    lastFollowErr = err;

    // Integer PD: Kp = 1.4, Kd = 0.6 -> (err * 14 + dErr * 6) / 10
    int16_t pdSpeed = 115 + (err * 14 + dErr * 6) / 10;
    uint8_t baseSpeed = constrain(pdSpeed, 115, 170); // Capped at 170 PWM (0.2-0.25 m/s)

    // Arc-Drive: bias wheel PWM while moving based on fused bearing
    uint8_t leftPwm = baseSpeed;
    uint8_t rightPwm = baseSpeed;
    const uint8_t kBias = 24;

    if (fusedBearing == BEARING_LEFT) {
      leftPwm = (baseSpeed > kBias + 60) ? baseSpeed - kBias : 60;
      rightPwm = (baseSpeed + kBias <= 170) ? baseSpeed + kBias : 170;
    } else if (fusedBearing == BEARING_RIGHT) {
      leftPwm = (baseSpeed + kBias <= 170) ? baseSpeed + kBias : 170;
      rightPwm = (baseSpeed > kBias + 60) ? baseSpeed - kBias : 60;
    }

    arcDrive(leftPwm, rightPwm);
    if (!safeMotorDelay(35, true, false)) {
      haltMotors();
      gFollowLockAcquired = false;
      return;
    }
    lastLostTime = millis();
    lastValidDist = dist;
  }
  // 8. Lost Target / Out of Range (>80cm / No Echo) -> Saccadic Radar Scan with Bearing Memory Bias
  else {
    haltMotors();
    displayBitmap(kIconStandby);

    // Safety rule: Do NOT pivot after a close-range loss: child may be standing beside wheels
    bool wasCloseRange = (lastValidDist > 0 && lastValidDist < 35);

    // If lost for > 400ms, gently search left/right ONLY if not lost at close range
    if (!wasCloseRange && millis() - lastLostTime > 400 && millis() - lastLostTime < 2500) {
      // B5: Use lastBearing memory to bias search direction instead of coin-flipping!
      if (gLastBearing.bearing == BEARING_LEFT && (millis() - gLastBearing.timestamp < 3500UL)) {
        scanDir = -1; // Bias search to LEFT
      } else if (gLastBearing.bearing == BEARING_RIGHT && (millis() - gLastBearing.timestamp < 3500UL)) {
        scanDir = 1;  // Bias search to RIGHT
      }

      const uint16_t searchMs = 25 * kPivotMsPerDegree; // 25° = 75ms at PWM 185
      if (scanDir > 0) pivotRight(kPivotCalibratedPwm);
      else pivotLeft(kPivotCalibratedPwm);
      if (!safeMotorDelay(searchMs, true, false)) {
        haltMotors();
        gFollowLockAcquired = false;
        return;
      }
      haltMotors();
      safeMotorDelay(60, false, false);

      uint16_t checkDist = getDistanceCm();
      if (checkDist >= kSafeStopDist && checkDist <= kMaxLeashDist) {
        lastLostTime = millis(); // Re-acquired target!
        lastValidDist = checkDist;

        // Audio & Heart Debounce
        if (millis() - lastChirpTrillTime > 4000) {
          displayBitmap(kIconHeart);
          talkAstromech(EMOTION_HAPPY);
          lastChirpTrillTime = millis();
        }
      } else {
        scanDir = -scanDir; // Alternate search direction if not found
      }
    } else if (millis() - lastLostTime >= 2500) {
      // 2.5s Auto-Timeout: Safe sleep disarm (reset handshake for next lock)
      bool wasLocked = gFollowLockAcquired;
      gFollowLockAcquired = false;
      haltMotors();
      if (wasLocked) chirpCurious(); // "where did you go?" — target lost, once
      digitalWrite(kPinStatusLed, (millis() / 400) % 2);
      safeMotorDelay(60, false, false);
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
  unsigned int targetFreq = pgm_read_word(&kPentatonicScale[noteIdx]);

  uint16_t rightHandDist = getDistanceCm();
  static uint8_t rhythmStep = 0;
  rhythmStep = (rhythmStep + 1) % 8;

  displayOscilloscope(noteIdx, rhythmStep);

  if (!gMasterMute) {
    if (rightHandDist < 10) {
      setBuzzerTone(targetFreq); delay(35);
      setBuzzerTone(targetFreq + 120); delay(35);
    } else if (rightHandDist < 25) {
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
  static uint16_t lastDist = 100;
  uint16_t dist = getDistanceCm();
  bool pirTrigger = checkPirMotionDetected();

  // Approach detection: triggers when someone is within 1.2m and approaching or within 50cm
  bool approachDetected = (dist > 8 && dist < 120 && ((int16_t)lastDist - (int16_t)dist > 6 || dist < 50));
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

/**
 * Mode 10: FOCUS CLOCK — a sitting clock that lives in the robot.
 * The 28 pixels of the outer ring are the minutes. They go out one by one as the
 * time is spent, and the centre dot blinks once a second so it reads as a clock.
 * At zero it plays a short finish tune and holds a "done" face until the pilot
 * chooses another mode. This mode NEVER drives a motor, which is why it is exempt
 * from the 1500 ms autonomous keepalive: it must survive the phone locking.
 */
const uint8_t PROGMEM kFocusRing[28] = {
  // packed (row << 3) | col, clockwise from the top-left corner
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x0F, 0x17, 0x1F, 0x27, 0x2F, 0x37, 0x3F,
  0x3E, 0x3D, 0x3C, 0x3B, 0x3A, 0x39, 0x38,
  0x30, 0x28, 0x20, 0x18, 0x10, 0x08
};

void runFocusClockMode() {
  // Belt and braces: this mode never commands a motor, but if anything else left
  // one running, the deadman path here kills it before anything else happens.
  if (gMotorActive) haltMotors();

  unsigned long elapsed = millis() - gFocusStartMs;
  unsigned long total = (unsigned long)gFocusMinutes * 60000UL;

  if (!gFocusDone && elapsed >= total) {
    gFocusDone = true;
    displayBitmap(kIconCrown);
    // Motors are stopped in this mode, so a blocking finish tune is safe.
    playTone(1046, 120); playTone(1318, 120);
    playTone(1568, 120); playTone(2093, 260);
    stopAllAudio();
  }

  if (gFocusDone) {
    displayBitmap(kIconAffirmative);
    digitalWrite(kPinStatusLed, (millis() / 500) % 2);
    delay(40);
    return;
  }

  uint8_t lit = (uint8_t)(((total - elapsed) * 28UL) / total);
  uint8_t frame[8] = {0};
  for (uint8_t i = 0; i < lit; i++) {
    uint8_t rc = pgm_read_byte(&kFocusRing[i]);
    frame[rc >> 3] |= (uint8_t)(1 << (7 - (rc & 0x07)));
  }
  // Centre dot: a one-second heartbeat so it reads as a clock, not a picture.
  if ((elapsed / 500) % 2 == 0) {
    frame[3] |= 0b00011000;
    frame[4] |= 0b00011000;
  }
  displayCustomBuffer(frame);
  digitalWrite(kPinStatusLed, ((elapsed / 500) % 2) ? HIGH : LOW);
  delay(40);
}

void runFlashbangAmbushMode() {
  haltMotors();
  uint16_t dist = getDistanceCm();
  bool heat = checkPirMotionDetected();
  if (Serial.available()) { haltMotors(); stopAllAudio(); return; }

  // 🛑 Cliff check before lunging
  uint8_t leftCliff  = digitalRead(kPinLineSensorLeft);
  uint8_t rightCliff = digitalRead(kPinLineSensorRight);
  if (edgeGuardActive() && (leftCliff == 0 || rightCliff == 0)) {
    haltMotors();
    return;
  }

  if (dist > 0 && dist < 45 && heat) {
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

  // 1. Cliff detected or existing fault: Stuck-State Defense
  if (leftCliff == 0 || rightCliff == 0 || gCliffFault) {
    haltMotors();
    gCliffRecoveryAttempts++;

    // Plausibility / Stuck-Low Protection:
    // If cliff reading persists across >3 consecutive recovery maneuvers,
    // declare a stuck-low sensor fault and latch into an emergency halt.
    if (gCliffRecoveryAttempts > 3 || gCliffFault) {
      gCliffFault = true;
      haltMotors();
      displayBitmap(kIconSkullAlert);
      if (!gMasterMute) chirpSweep(350, 150, 30, 3);
      safeMotorDelay(60, false, false);
      return;
    }

    talkAstromech(EMOTION_RELIEVED);
    displayBitmap(kIconSkullAlert);
    driveBackward(160);
    if (!safeMotorDelay(280, false, false)) { haltMotors(); return; }
    haltMotors();
    pivotRight(180);
    if (!safeMotorDelay(350, false, false)) { haltMotors(); return; }
    haltMotors();

    // Verify recovery to safe floor
    leftCliff  = digitalRead(kPinLineSensorLeft);
    rightCliff = digitalRead(kPinLineSensorRight);
    if (leftCliff == 1 && rightCliff == 1) {
      gCliffRecoveryAttempts = 0; // Successfully backed onto safe floor
      gCliffForwardDriveStart = 0; // Reset drive timer
    }
    return;
  }

  // 2. Clear Floor: Reset recovery counter
  gCliffRecoveryAttempts = 0;

  // 3. Sensor Plausibility Check (Stuck-High / Disconnected Wire Protection):
  // When continuously driving forward in Cliff Mode, if both sensors remain completely
  // unchanged for > 10 seconds without any edge or variation, latch fault and halt.
  if (gCliffForwardDriveStart == 0) {
    gCliffForwardDriveStart = millis();
    gCliffLastStateChangeTime = millis();
    gLastLeftCliffState = leftCliff;
    gLastRightCliffState = rightCliff;
  } else {
    if (leftCliff != gLastLeftCliffState || rightCliff != gLastRightCliffState) {
      gCliffLastStateChangeTime = millis();
      gLastLeftCliffState = leftCliff;
      gLastRightCliffState = rightCliff;
    } else if (millis() - gCliffLastStateChangeTime > 10000) {
      // Invariant: Unchanged continuous drive timeout -> fault -> latched halt!
      gCliffFault = true;
      haltMotors();
      displayBitmap(kIconSkullAlert);
      return;
    }
  }

  displayBitmap(kIconCliffGuard);
  driveForward(140);
  safeMotorDelay(20, true, true);
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
        static const uint16_t PROGMEM notes[] = { 440, 440, 440, 349, 523, 440, 349, 523, 440 };
        static const uint16_t PROGMEM dur[]   = { 260, 260, 260, 180, 100, 260, 180, 100, 450 };
        for (int i = 0; i < 9; i++) {
          if (Serial.available()) { haltMotors(); return; }
          playTone(pgm_read_word(&notes[i]), pgm_read_word(&dur[i]));
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
        static const uint16_t PROGMEM notes[] = { 523, 659, 784, 1046, 1318, 1046, 1318 };
        static const uint16_t PROGMEM dur[]   = { 120, 120, 120,  220,  150,  120,  400 };
        for (int i = 0; i < 7; i++) {
          if (Serial.available()) { haltMotors(); return; }
          playTone(pgm_read_word(&notes[i]), pgm_read_word(&dur[i]));
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
  static const uint16_t PROGMEM notes[] = { 440, 440, 440, 349, 523, 440, 349, 523, 440 };
  static const uint16_t PROGMEM durations[] = { 400, 400, 400, 280, 140, 400, 280, 140, 600 };
  for (int i = 0; i < 9; i++) {
    if (Serial.available()) break;
    playTone(pgm_read_word(&notes[i]), pgm_read_word(&durations[i]));
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
  static const uint16_t PROGMEM notes[] = { 660, 660, 660, 510, 660, 770, 380 };
  static const uint16_t PROGMEM durations[] = { 100, 100, 100, 100, 100, 100, 200 };
  for (int i = 0; i < 7; i++) {
    if (Serial.available()) break;
    playTone(pgm_read_word(&notes[i]), pgm_read_word(&durations[i]));
    delay(60);
  }
}

void playMissionImpossible() {
  displayBitmap(kIconSentryEye);
  static const uint16_t PROGMEM notes[] = { 587, 587, 698, 784, 587, 587, 523, 554 };
  static const uint16_t PROGMEM durations[] = { 150, 150, 150, 150, 150, 150, 150, 150 };
  for (int i = 0; i < 8; i++) {
    if (Serial.available()) break;
    playTone(pgm_read_word(&notes[i]), pgm_read_word(&durations[i]));
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

/* ---------------------------------------------------------------------------
 * R2-D2 CHIRP ENGINE
 * A chirp must never delay a stop. So: when the wheels are STOPPED a chirp may
 * block (it is only tens of milliseconds); when the wheels are TURNING the same
 * chirp is handed to a tiny millis() state machine that advances exactly one
 * tone step per loop pass. Any stopAllAudio() — and every halt path calls it —
 * cancels a chirp in flight instantly.
 * ---------------------------------------------------------------------------
 */
#define NB_CHIRP_MAX 6
static uint16_t gNbF0[NB_CHIRP_MAX];
static uint16_t gNbF1[NB_CHIRP_MAX];
static uint8_t  gNbDur[NB_CHIRP_MAX];
static uint8_t  gNbCount = 0;
static uint8_t  gNbIdx = 0;
static unsigned long gNbStepStart = 0;
static bool gNbActive = false;

void stopAllAudio() {
  gNbActive = false;
  noTone(kPinBuzzer1);
}

void serviceChirpNB() {
  if (!gNbActive) return;
  if (gMasterMute) { gNbActive = false; noTone(kPinBuzzer1); return; }
  unsigned long dt = millis() - gNbStepStart;
  uint8_t d = gNbDur[gNbIdx];
  if (dt >= d) {
    gNbIdx++;
    if (gNbIdx >= gNbCount) { gNbActive = false; noTone(kPinBuzzer1); return; }
    gNbStepStart = millis();
    tone(kPinBuzzer1, gNbF0[gNbIdx]);
  } else if (dt >= (unsigned long)(d >> 1)) {
    tone(kPinBuzzer1, gNbF1[gNbIdx]); // second half of the glide
  }
}

// A random R2 sentence: 3-6 short glides, 30-80ms each, capped at 250ms total,
// so no two ever sound the same.
void chirpRandomR2() {
  if (gMasterMute) return;
  uint8_t want = random(3, 7);
  uint16_t budget = 0;
  gNbCount = 0;
  for (uint8_t i = 0; i < want; i++) {
    uint8_t dur = (uint8_t)random(30, 81);
    if (budget + dur > 250) break;
    gNbF0[gNbCount] = (uint16_t)random(600, 2601);
    gNbF1[gNbCount] = (uint16_t)random(600, 2601);
    gNbDur[gNbCount] = dur;
    budget += dur;
    gNbCount++;
  }
  if (gNbCount == 0) return;

  if (gMotorActive) {
    // Wheels turning: hand it to the non-blocking machine and return at once.
    gNbIdx = 0;
    gNbStepStart = millis();
    gNbActive = true;
    tone(kPinBuzzer1, gNbF0[0]);
    return;
  }

  gNbActive = false;
  for (uint8_t i = 0; i < gNbCount; i++) {
    uint8_t d = gNbDur[i];
    tone(kPinBuzzer1, gNbF0[i]); delay(d >> 1);
    tone(kPinBuzzer1, gNbF1[i]); delay(d - (d >> 1));
  }
  noTone(kPinBuzzer1);
}

// One short, distinct two-note signature per mode (~70ms). Only ever called
// from setMode(), which has already halted the motors.
void chirpModeChange(uint8_t mode) {
  if (gMasterMute) return;
  unsigned int base = 700 + (unsigned int)mode * 130;
  tone(kPinBuzzer1, base); delay(35);
  tone(kPinBuzzer1, base + 400); delay(35);
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
      static const uint16_t PROGMEM notes[] = { 880, 1175, 1397, 1760, 2093, 2637, 3136, 3520 };
      for (int i = 0; i < 8; i++) {
        tone(kPinBuzzer1, pgm_read_word(&notes[i])); delay(28);
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

void playDopplerRadarPing(uint16_t distanceCm) {
  if (gMasterMute || distanceCm == 0 || distanceCm > 120) return;
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

void arcDrive(uint8_t leftPwm, uint8_t rightPwm) {
  analogWrite(kPinMotorLeftPwm, leftPwm);
  analogWrite(kPinMotorRightPwm, rightPwm);
  digitalWrite(kPinMotorLeftDir, LOW);
  digitalWrite(kPinMotorRightDir, LOW);
  if (leftPwm > 0 || rightPwm > 0) gMotorActive = true;
}

uint16_t getDistanceCm() {
  // F8: clear the Echo Lab reading FIRST, so the stuck-high early return below can
  // never hand back the flight time of a previous, unrelated ping.
  gLastEchoMicros = 0;
  // Stuck-high echo before trigger indicates hardware electrical fault
  if (digitalRead(kPinUltrasonicEcho) == HIGH) {
    gSonarFault = true;
    gSonarGoodCount = 0;
    return 0;
  }

  digitalWrite(kPinUltrasonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(kPinUltrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(kPinUltrasonicTrig, LOW);

  unsigned long duration = pulseIn(kPinUltrasonicEcho, HIGH, 25000); // 25ms max timeout (~4.2m)
  gLastEchoMicros = duration;
  if (duration == 0) {
    // A RUN OF TIMEOUTS IS NOT A FAULT. Facing an open room, "no echo inside
    // 4.2 m" is the correct, healthy answer — it used to latch gSonarFault after
    // 8 of them, which froze Scout in the middle of a big floor. The only
    // electrical fault we can actually prove is the stuck-HIGH echo pin checked
    // above. Return value is unchanged: 0 still means "cannot see".
    gSonarGoodCount = 0;
    return 0;
  }
  gSonarGoodCount++;
  if (gSonarGoodCount >= 5) {
    gSonarFault = false; // Requires 5 consecutive valid readings to clear!
  }
  // Integer arithmetic: (duration + 29) / 58 cm
  return (uint16_t)((duration + 29UL) / 58UL);
}

// F6: a silent sonar must not look like a steady reading. The median below excludes
// zeros, so a sonar returning 0 (unplugged, echo floating low) used to leave the last
// good median in place and Follow Me kept driving on it. Count the consecutive zeros:
// after kSonarZeroLost of them the filter reports 0 — "I cannot see" — and Follow Me
// drops its lock and runs the normal lost/search behaviour.
uint16_t getFilteredDistance() {
  uint16_t raw = getDistanceCm();
  gLatestRawDist = raw;
  if (raw == 0) {
    if (gSonarZeroRun < 255) gSonarZeroRun++;
  } else {
    gSonarZeroRun = 0;
  }
  if (gSonarZeroRun >= kSonarZeroLost) {
    gFollowFilteredDist = 0;
    return 0; // stale median deliberately NOT returned
  }
  if (raw > 0 && raw < 400) {
    gDistanceHistory[gHistoryIdx] = raw;
    gHistoryIdx = (gHistoryIdx + 1) % 5;
  }

  // 6-Line Median-of-5 Filter (Rejects acoustic dropouts and spikes completely)
  uint16_t sorted[5];
  for (uint8_t i = 0; i < 5; i++) sorted[i] = gDistanceHistory[i];
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = i + 1; j < 5; j++) {
      if (sorted[i] > sorted[j]) {
        uint16_t tmp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = tmp;
      }
    }
  }
  gFollowFilteredDist = sorted[2];
  return gFollowFilteredDist;
}

// Reads the ONE photocell that is actually wired: the LDR on A0 (kPinLdrFollow).
// The old version took max() over A2/A3/A6/A7 — all unconnected expansion pins,
// which float and return noise, so the theremin (mode 8) tracked nothing real.
// Polarity assumption: higher ADC == brighter, matching Follow Me's convention
// (currentLdr > 250 == "optical beacon", ldrDelta > 4 == "brightness rising").
// If a future LDR module is wired inverted, flip here and in sampleLdrWithHealth().
int readActivePhotocell() {
  return analogRead(kPinLdrFollow);
}

// ---------------------------------------------------------------------------
// B2: Follow Me living heartbeat.
// While Follow Me holds lock, D13 pulses like a pulse: a short bright systole
// then a rest, and the rest gets shorter the closer the pilot stands. Returns
// the heart bitmap for this instant (big on the beat, small between beats) so
// the two RESTING branches can show it; the moving branches ignore it and keep
// their arrows. No delay(), no float, no heap — pure millis() state machine,
// so nothing in the motion or safety path is touched.
// Period: systole 90ms; diastole 320ms at 18cm rising to ~1126ms at 80cm
// (integer curve 320 + (d-18)*13, matching a map() without the long math).
// ---------------------------------------------------------------------------
static bool gHeartSystole = false;
static unsigned long gHeartLastBeat = 0;

const uint8_t* tickFollowHeartbeat(uint16_t dist) {
  uint16_t d = (dist == 0) ? 80 : constrain(dist, 18, 80);
  uint16_t phaseMs = gHeartSystole ? 90 : (uint16_t)(320 + (d - 18) * 13);
  if (millis() - gHeartLastBeat >= phaseMs) {
    gHeartLastBeat = millis();
    gHeartSystole = !gHeartSystole;
    // Safe: inside Follow Me, D13 is otherwise driven only by the lost-timeout
    // branch, which runs only after gFollowLockAcquired has been cleared.
    digitalWrite(kPinStatusLed, gHeartSystole ? HIGH : LOW);
  }
  return gHeartSystole ? kIconHeart : kIconHeartSmall;
}

void resetFollowHeartbeat() {
  gHeartSystole = false;
  gHeartLastBeat = millis();
  digitalWrite(kPinStatusLed, LOW);
}

int sampleLdrWithHealth() {
  int raw = analogRead(kPinLdrFollow);
  static int minAdc = 1023;
  static int maxAdc = 0;
  static unsigned long lastLdrWindow = 0;

  if (raw < minAdc) minAdc = raw;
  if (raw > maxAdc) maxAdc = raw;

  if (millis() - lastLdrWindow >= 3000UL) {
    int variance = maxAdc - minAdc;
    // Active LDR responds to torch / ambient movement with variance >= 12
    gLdrHealthy = (variance >= 12);
    minAdc = 1023;
    maxAdc = 0;
    lastLdrWindow = millis();
  }
  return raw;
}

bool checkPirMotionDetected() {
  static uint8_t lastPirState = 255;
  static unsigned long lastEdgeTime = 0;

  uint8_t current = digitalRead(kPinExpansionD12);
  if (lastPirState == 255) {
    lastPirState = current;
    lastEdgeTime = millis();
    gPirHealthy = false;
    return false;
  }

  bool triggered = false;
  if (current != lastPirState) {
    if (current == HIGH && lastPirState == LOW) {
      triggered = true;
    }
    lastPirState = current;
    lastEdgeTime = millis();
    gPirHealthy = true;
  } else {
    // If no pin level transitions occur for > 12 seconds, sensor is absent/unplugged
    if (millis() - lastEdgeTime > 12000UL) {
      gPirHealthy = false;
    }
  }

  return (gPirHealthy && triggered);
}
