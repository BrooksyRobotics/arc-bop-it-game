/*
 * ============================================================
 *  HEAD BOP-IT v2  —  Arduino + GY-521 + MAX7219 Matrix
 * ============================================================
 *
 *  WIRING (unchanged)
 *  GY-521:  VCC->3.3V  GND->GND  SDA->A4  SCL->A5  AD0->GND
 *  MAX7219: VCC->5V    GND->GND  DIN->11  CS->10   CLK->13
 *  Buzzer:  (+)->Pin 8  (-)->GND
 *
 *  AXES (corrected for sensor orientation)
 *  gz < 0  = Twist Left   (yaw left)
 *  gz > 0  = Twist Right  (yaw right)
 *  gy > 0  = Bob Down     (nod forward)
 *  gy < 0  = Bob Up       (nod back)
 *  gx > 0  = Roll Right   (lean right)
 *  gx < 0  = Roll Left    (lean left)
 *
 *  LEVELS
 *  Level 1 — 3 moves: Twist Left, Bob Down, Roll Right
 *             Full-width arrows (existing style)
 *  Level 2 — All 6 moves: corner + center arrows
 *             Unlocks after scoring LEVEL_UP_SCORE in Level 1
 *
 *  Level 2 display layout (after 90 deg CW rotation):
 *    top-left  TL = Twist Left    top-center  TC = Bob Up
 *    top-right TR = Twist Right   bot-center  BC = Bob Down
 *    bot-right BR = Roll Right    bot-left    BL = Roll Left
 *
 *  DEPENDENCIES  (Library Manager)
 *  LedControl  by Eberhard Fahle
 *  MPU6050     by Electronic Cats
 * ============================================================
 */

#include <Wire.h>
#include <MPU6050.h>
#include <LedControl.h>
#include <EEPROM.h>
const uint16_t EEPROM_ADDR = 0;  // uses 2 bytes (uint16_t)

// ── Pins ─────────────────────────────────────────────────────
const uint8_t DIN_PIN    = 11;
const uint8_t CS_PIN     = 10;
const uint8_t CLK_PIN    = 13;
const uint8_t BUZZER_PIN =  8;

// ── Move IDs ─────────────────────────────────────────────────
const uint8_t MOVE_NONE        = 0;
const uint8_t MOVE_TWIST_LEFT  = 1;   // gz < 0
const uint8_t MOVE_BOB_DOWN    = 2;   // gy > 0
const uint8_t MOVE_ROLL_RIGHT  = 3;   // gx > 0
const uint8_t MOVE_TWIST_RIGHT = 4;   // gz > 0
const uint8_t MOVE_BOB_UP      = 5;   // gy < 0
const uint8_t MOVE_ROLL_LEFT   = 6;   // gx < 0

// ── Gyro tuning ──────────────────────────────────────────────
const int16_t GYRO_THRESH     = 8000; // ~61 deg/s
const uint8_t CONFIRM_SAMPLES =    4; // consecutive matching samples required
const uint8_t SAMPLE_DELAY_MS =   15; // ms between samples

// ── Timing ───────────────────────────────────────────────────
const uint16_t SHOW_CUE_MS   = 300;
const uint16_t BASE_LIMIT_MS = 3000;
const uint16_t MIN_LIMIT_MS  =  700;
const uint16_t SHRINK_MS     =   80;
const uint8_t  LEVEL_UP_SCORE =  10; // correct moves to unlock Level 2

// ── Brightness ───────────────────────────────────────────────
const uint8_t MATRIX_BRIGHT = 8;
const uint8_t MATRIX_DIM    = 2;

// ── Tones (Hz) ───────────────────────────────────────────────
const uint16_t TONE_TWIST_LEFT  = 880;   // A5
const uint16_t TONE_BOB_DOWN    = 660;   // E5
const uint16_t TONE_ROLL_RIGHT  = 523;   // C5
const uint16_t TONE_TWIST_RIGHT = 784;   // G5
const uint16_t TONE_BOB_UP      = 587;   // D5
const uint16_t TONE_ROLL_LEFT   = 440;   // A4
const uint16_t TONE_WIN         = 1047;  // C6
const uint16_t TONE_LOSE_1      = 200;
const uint16_t TONE_LOSE_2      = 150;
const uint16_t TONE_LOSE_3      = 100;

// ─────────────────────────────────────────────────────────────
LedControl lc = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);
MPU6050    mpu;
uint16_t   timeLimit;
uint32_t   score;
uint8_t    currentLevel;

// ═══════════════════════════════════════════════════════════════
//  BITMAPS  (rotated 90 deg CW at display time via showPattern)
//  Each byte = one raw row. MSB = leftmost pixel.
// ═══════════════════════════════════════════════════════════════

// ── Level 1: full-width arrows (unchanged) ────────────────────

const byte PAT_L1_TWIST_LEFT[8] PROGMEM = {   // raw <- -> displays as up-arrow
  0b00010000,
  0b00110000,
  0b01110000,
  0b11111110,
  0b01110000,
  0b00110000,
  0b00010000,
  0b00000000
};

const byte PAT_L1_BOB_DOWN[8] PROGMEM = {     // raw down-arrow -> displays as left-arrow
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b01111110,
  0b00111100,
  0b00011000
};

const byte PAT_L1_ROLL_RIGHT[8] PROGMEM = {   // raw -> -> displays as down-arrow
  0b00001000,
  0b00001100,
  0b00001110,
  0b01111111,
  0b00001110,
  0b00001100,
  0b00001000,
  0b00000000
};

// ── Level 2: corner + center arrows ──────────────────────────

const byte PAT_L2_TWIST_LEFT[8] PROGMEM = {
  0b11111000,   
  0b11100000,   
  0b11000000,
  0b10000000,
  0b10000000,
  0b00000000,
  0b00000000,
  0b00000000
};

const byte PAT_L2_TWIST_RIGHT[8] PROGMEM = {
  0b00011111,   
  0b00000111,   
  0b00000011,
  0b00000001,
  0b00000001,
  0b00000000,
  0b00000000,
  0b00000000
};

const byte PAT_L2_BOB_UP[8] PROGMEM = {
  0b00011000, 
  0b00111100,   
  0b11111111,
  0b00011000,
  0b00011000,
  0b00000000,
  0b00000000,
  0b00000000
};

const byte PAT_L2_BOB_DOWN[8] PROGMEM = {
  0b00000000,
  0b00000000,
  0b00000000,
  0b00011000,
  0b00011000,
  0b11111111,
  0b00111100,
  0b00011000
};

const byte PAT_L2_ROLL_RIGHT[8] PROGMEM = {
  0b00000000,   
  0b00000000,   
  0b00000000,
  0b00000001,
  0b00000001,
  0b00000011,
  0b00000111,
  0b00011111
};

const byte PAT_L2_ROLL_LEFT[8] PROGMEM = {
  0b00000000,   
  0b00000000,   
  0b00000000,
  0b10000000,
  0b10000000,
  0b11000000,
  0b11100000,
  0b11111000
};

// ── Shared patterns ───────────────────────────────────────────

const byte PAT_WIN[8] PROGMEM = {      // checkmark
  0b00000000,
  0b00000001,
  0b00000011,
  0b10000110,
  0b11001100,
  0b01111000,
  0b00110000,
  0b00000000
};

const byte PAT_LOSE[8] PROGMEM = {     // bold X
  0b11000011,
  0b01100110,
  0b00111100,
  0b00011000,
  0b00011000,
  0b00111100,
  0b01100110,
  0b11000011
};

const byte PAT_READY[8] PROGMEM = {    // smiley face
  0b00111100,
  0b01000010,
  0b10100101,
  0b10000001,
  0b10100101,
  0b10011001,
  0b01000010,
  0b00111100
};

const byte PAT_LEVELUP[8] PROGMEM = {  // checkerboard flash
  0b10101010,
  0b01010101,
  0b10101010,
  0b01010101,
  0b10101010,
  0b01010101,
  0b10101010,
  0b01010101
};

// Column-major digit font. bit 0 = top row, bit 6 = bottom row.
const byte FONT_5x7[10][5] PROGMEM = {
  {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 0
  {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
  {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
  {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
  {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
  {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
  {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
  {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
  {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
  {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
};
// ─────────────────────────────────────────────────────────────
//  Display helpers
// ─────────────────────────────────────────────────────────────
uint16_t loadHighScore() {
  uint16_t hs;
  EEPROM.get(EEPROM_ADDR, hs);
  // Fresh chip returns 0xFFFF — treat as zero
  if (hs == 0xFFFF) hs = 0;
  return hs;
}

void saveHighScore(uint16_t hs) {
  EEPROM.put(EEPROM_ADDR, hs);
}

void rotate90CW(const byte* src, byte* dst) {
  for (uint8_t r = 0; r < 8; r++) {
    dst[r] = 0;
    for (uint8_t c = 0; c < 8; c++) {
      uint8_t bit = (pgm_read_byte(&src[7 - c]) >> (7 - r)) & 1;
      dst[r] |= (bit << (7 - c));
    }
  }
}

void showPattern(const byte* pattern) {
  byte rotated[8];
  rotate90CW(pattern, rotated);
  for (uint8_t row = 0; row < 8; row++) {
    lc.setRow(0, row, rotated[row]);
  }
}

void clearMatrix() { lc.clearDisplay(0); }

const byte* patternForMove(uint8_t mv) {
  if (currentLevel == 1) {
    if (mv == MOVE_TWIST_LEFT)  return PAT_L1_TWIST_LEFT;
    if (mv == MOVE_BOB_DOWN)    return PAT_L1_BOB_DOWN;
    if (mv == MOVE_ROLL_RIGHT)  return PAT_L1_ROLL_RIGHT;
  } else {
    if (mv == MOVE_TWIST_LEFT)  return PAT_L2_TWIST_LEFT;
    if (mv == MOVE_TWIST_RIGHT) return PAT_L2_TWIST_RIGHT;
    if (mv == MOVE_BOB_DOWN)    return PAT_L2_BOB_DOWN;
    if (mv == MOVE_BOB_UP)      return PAT_L2_BOB_UP;
    if (mv == MOVE_ROLL_RIGHT)  return PAT_L2_ROLL_RIGHT;
    if (mv == MOVE_ROLL_LEFT)   return PAT_L2_ROLL_LEFT;
  }
  return PAT_READY;
}

uint16_t toneForMove(uint8_t mv) {
  switch (mv) {
    case MOVE_TWIST_LEFT:  return TONE_TWIST_LEFT;
    case MOVE_BOB_DOWN:    return TONE_BOB_DOWN;
    case MOVE_ROLL_RIGHT:  return TONE_ROLL_RIGHT;
    case MOVE_TWIST_RIGHT: return TONE_TWIST_RIGHT;
    case MOVE_BOB_UP:      return TONE_BOB_UP;
    case MOVE_ROLL_LEFT:   return TONE_ROLL_LEFT;
    default:               return 440;
  }
}

void scrollScore(uint16_t s) {
  char buf[8];
  uint8_t len = (uint8_t)sprintf(buf, "%u", s);
  // 8 blank lead + (5 pixels + 1 gap) per digit + 8 blank trail
  uint16_t totalCols = 8 + (uint16_t)len * 6 + 8;

  lc.setIntensity(0, MATRIX_BRIGHT);

  for (uint16_t offset = 0; offset <= totalCols - 8; offset++) {
    for (uint8_t dc = 0; dc < 8; dc++) {
      // display column dc → physical row (7-dc) due to 90 CW mount
      int16_t textCol = (int16_t)(offset + dc) - 8;
      byte colByte = 0;

      if (textCol >= 0 && textCol < (int16_t)(len * 6)) {
        uint8_t charIdx = textCol / 6;
        uint8_t charCol = textCol % 6;
        if (charCol < 5)
          colByte = pgm_read_byte(&FONT_5x7[buf[charIdx] - '0'][charCol]);
      }

      lc.setRow(0, dc, colByte);
    }
    delay(80); // increase to slow the scroll
  }
  clearMatrix();
}

// ─────────────────────────────────────────────────────────────
//  Game sequences
// ─────────────────────────────────────────────────────────────

void startupSequence() {
  // Preview all moves for the current level so the player can
  // memorise which arrow means which head movement.
  uint8_t moves[] = { MOVE_TWIST_LEFT, MOVE_BOB_DOWN,  MOVE_ROLL_RIGHT,
                      MOVE_TWIST_RIGHT, MOVE_BOB_UP,   MOVE_ROLL_LEFT };
  uint8_t count   = (currentLevel == 1) ? 3 : 6;

  for (uint8_t i = 0; i < count; i++) {
    lc.setIntensity(0, MATRIX_BRIGHT);
    showPattern(patternForMove(moves[i]));
    tone(BUZZER_PIN, toneForMove(moves[i]), 220);
    delay(380);
    clearMatrix();
    delay(120);
  }

  // Ready smiley + rising arpeggio
  lc.setIntensity(0, MATRIX_BRIGHT);
  showPattern(PAT_READY);
  tone(BUZZER_PIN, 523, 100); delay(120);
  tone(BUZZER_PIN, 660, 100); delay(120);
  tone(BUZZER_PIN, 784, 100); delay(120);
  tone(BUZZER_PIN, TONE_WIN, 300);
  delay(600);
  noTone(BUZZER_PIN);
  clearMatrix();
  delay(400);
}

void levelUpSequence() {
  // Rising fanfare
  uint16_t notes[] = { 523, 659, 784 };
  for (uint8_t i = 0; i < 3; i++) {
    tone(BUZZER_PIN, notes[i], 120);
    delay(140);
  }
  tone(BUZZER_PIN, 1047, 500);
  delay(600);
  noTone(BUZZER_PIN);

  // Checkerboard flash x5
  for (uint8_t i = 0; i < 5; i++) {
    lc.setIntensity(0, MATRIX_BRIGHT);
    showPattern(PAT_LEVELUP);
    delay(160);
    clearMatrix();
    delay(120);
  }

  Serial.println(F("*** LEVEL 2 UNLOCKED — all 6 moves active! ***"));
  delay(500);
}

void loseSequence() {
  noTone(BUZZER_PIN);
  lc.setIntensity(0, MATRIX_BRIGHT);
  showPattern(PAT_LOSE);

  tone(BUZZER_PIN, TONE_LOSE_1, 180); delay(200);
  tone(BUZZER_PIN, TONE_LOSE_2, 180); delay(200);
  tone(BUZZER_PIN, TONE_LOSE_3, 350); delay(400);
  noTone(BUZZER_PIN);

  for (uint8_t i = 0; i < 5; i++) {
    lc.setIntensity(0, MATRIX_BRIGHT);
    showPattern(PAT_LOSE);
    delay(160);
    clearMatrix();
    delay(120);
  }
  uint16_t hiScore = loadHighScore();
  if (score > hiScore) {
    hiScore = (uint16_t)score;
    saveHighScore(hiScore);
    tone(BUZZER_PIN, TONE_WIN, 300); delay(400); noTone(BUZZER_PIN);
  }
  Serial.print(F("Score: ")); Serial.print(score);
  Serial.print(F("  Best: ")); Serial.println(hiScore);

  scrollScore(score);    // scroll current score
  scrollScore(hiScore);  // then scroll the best
  clearMatrix();
  delay(1000);
}

void winPing(uint8_t mv) {
  lc.setIntensity(0, MATRIX_BRIGHT);
  showPattern(PAT_WIN);
  tone(BUZZER_PIN, TONE_WIN, 80);
  delay(150);
  noTone(BUZZER_PIN);
  lc.setIntensity(0, MATRIX_DIM);
  showPattern(patternForMove(mv));
  delay(80);
  clearMatrix();
}

// ─────────────────────────────────────────────────────────────
//  Movement detection
// ─────────────────────────────────────────────────────────────

uint8_t readMovement() {
  uint8_t candidate = MOVE_NONE;
  uint8_t streak    = 0;

  for (uint8_t i = 0; i < 20; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    int16_t agx    = abs(gx);
    int16_t agy    = abs(gy);
    int16_t agz    = abs(gz);
    int16_t maxVal = max(agx, max(agy, agz));

    uint8_t detected = MOVE_NONE;

    if (maxVal >= GYRO_THRESH) {
      // Winning axis must be at least 2x stronger than both others
      if      (agz >= agx && agz >= agy && agz > agx * 2 && agz > agy * 2)
        detected = (gz > 0) ? MOVE_TWIST_LEFT  : MOVE_TWIST_RIGHT;
      else if (agy >= agx && agy >= agz && agy > agx * 2 && agy > agz * 2)
        detected = (gy > 0) ? MOVE_BOB_DOWN    : MOVE_BOB_UP;
      else if (agx >= agy && agx >= agz && agx > agy * 2 && agx > agz * 2)
        detected = (gx > 0) ? MOVE_ROLL_RIGHT  : MOVE_ROLL_LEFT;
    }

    // Level 1: ignore the three new moves entirely
    if (currentLevel == 1 &&
        detected != MOVE_TWIST_LEFT &&
        detected != MOVE_BOB_DOWN   &&
        detected != MOVE_ROLL_RIGHT)
      detected = MOVE_NONE;

    if (detected != MOVE_NONE && detected == candidate) {
      if (++streak >= CONFIRM_SAMPLES) return candidate;
    } else {
      candidate = detected;
      streak    = (detected != MOVE_NONE) ? 1 : 0;
    }

    delay(SAMPLE_DELAY_MS);
  }
  return MOVE_NONE;
}

// ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000);

  Wire.begin();
  lc.shutdown(0, false);
  lc.setIntensity(0, MATRIX_BRIGHT);
  lc.clearDisplay(0);

  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  // testConnection() skipped — unreliable on clone chips
  // I2C scanner already confirmed address 0x68

  Serial.println(F("Head Bop-It v2 ready — Level 1"));
  randomSeed(analogRead(A0));

  currentLevel = 1;
  startupSequence();
  //saveHighScore(0);  // Uncomment to reset high score
}

// ─────────────────────────────────────────────────────────────

void loop() {
  score     = 0;
  timeLimit = BASE_LIMIT_MS;
  clearMatrix();
  noTone(BUZZER_PIN);
  delay(500);

  Serial.print(F("--- NEW GAME  Level: "));
  Serial.print(currentLevel);
  Serial.println(F(" ---"));

  while (true) {
    // Choose a random move for the current level
    uint8_t currentMove = (currentLevel == 1) ? random(1, 4) : random(1, 7);

    // Bright cue: show pattern + play tone
    lc.setIntensity(0, MATRIX_BRIGHT);
    showPattern(patternForMove(currentMove));
    tone(BUZZER_PIN, toneForMove(currentMove), SHOW_CUE_MS / 2);
    delay(SHOW_CUE_MS);
    noTone(BUZZER_PIN);

    // Dim = "your turn now"
    lc.setIntensity(0, MATRIX_DIM);

    const char* name;
    switch (currentMove) {
      case MOVE_TWIST_LEFT:  name = "TWIST LEFT";  break;
      case MOVE_BOB_DOWN:    name = "BOB DOWN";    break;
      case MOVE_ROLL_RIGHT:  name = "ROLL RIGHT";  break;
      case MOVE_TWIST_RIGHT: name = "TWIST RIGHT"; break;
      case MOVE_BOB_UP:      name = "BOB UP";      break;
      case MOVE_ROLL_LEFT:   name = "ROLL LEFT";   break;
      default:               name = "???";
    }
    Serial.println(name);

    // Wait for a matching move within the time window
    uint32_t deadline = millis() + timeLimit;
    uint8_t  detected  = MOVE_NONE;
    delay(80);  // let residual motion settle

    while (millis() < deadline) {
      detected = readMovement();
      if (detected != MOVE_NONE) break;
      delay(10);
    }

    if (detected == currentMove) {
      // ── CORRECT ─────────────────────────────────────────────
      score++;
      winPing(currentMove);

      if (timeLimit > MIN_LIMIT_MS) {
        timeLimit -= SHRINK_MS;
        if (timeLimit < MIN_LIMIT_MS) timeLimit = MIN_LIMIT_MS;
      }

      Serial.print(F("  Score: "));  Serial.print(score);
      Serial.print(F("  Window: ")); Serial.print(timeLimit);
      Serial.println(F("ms"));

      // Level up?
      if (currentLevel == 1 && score >= LEVEL_UP_SCORE) {
        currentLevel = 2;
        timeLimit    = BASE_LIMIT_MS;   // reset window for new level
        levelUpSequence();
        startupSequence();              // preview the 6 new patterns
      }

      delay(200);

    } else {
      // ── WRONG / TIMEOUT ─────────────────────────────────────
      loseSequence();
      currentLevel = 1;   // back to the beginning
      break;
    }
  }
}
