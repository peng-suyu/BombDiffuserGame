/*
  wirecut.ino
  ----------------
  Wire-Cutting Bomb Diffuser Game
  - Detects "cut" wires using INPUT_PULLUP (wire connected to 5V when intact).
  - Countdown timer on I2C 16x2 LCD.
  - Buzzer for sounds (defused / wrong cut / timer up).
  - Reset/Start button to arm and restart the game.
  - Optional LEDs for status indicators.

  Hardware / Parts (suggested)
  - Arduino Uno / Nano (any AVR-compatible board)
  - I2C 16x2 LCD module (LiquidCrystal_I2C library) OR compatible OLED (you'd need to swap library)
  - Piezo buzzer (passive works; active buzzers also OK but you cannot play tones on active buzzer)
  - Jumper wires / headers for "cuttable" wires (or short wires you will physically cut)
  - Pushbutton (momentary) for reset/start
  - Optional: LEDs (green/red) with 220Ω resistors
  - Breadboard / enclosure / battery pack

  Wiring (recommended)
  - LCD I2C:
      VCC -> 5V
      GND -> GND
      SDA -> A4  (Uno)
      SCL -> A5  (Uno)
  - Buzzer:
      + -> BUZZER_PIN (digital)
      - -> GND
  - Wires to cut (for each i):
      5V -----> WIRE_i -----> WIRE_PIN[i]
    (Because we use INPUT_PULLUP, when wire is connected pin reads HIGH. When unplugged/cut, pin reads LOW.)
    If you prefer, you can place a 1k resistor in series for safety.
  - Button:
      One side -> BUTTON_PIN
      Other side -> GND
    (button uses INPUT_PULLUP; pressed = LOW)
  - Optional LEDs:
      LED_PIN -> (220Ω) -> LED -> GND

  Libraries:
  - LiquidCrystal_I2C (install from Library Manager)
    Example include below:
      #include <LiquidCrystal_I2C.h>

  Usage:
  - Upload sketch.
  - Press the start/reset button to arm the device and start countdown.
  - Cut/unplug wires. Cutting the correct wire defuses, cutting a wrong wire triggers boom.
  - After game end, press button to reset and play again.

  Tuning variables are in CONFIG section below.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h> // install via Library Manager if you don't have it

// ================= CONFIG =================

// Choose how many wires you want (2..6 recommended)
const uint8_t NUM_WIRES = 3;

// Digital pins for wire inputs (must match NUM_WIRES)
// Ensure these are digital-capable pins on your board.
const uint8_t WIRE_PINS[6] = {2, 3, 4, 5, 6, 7};  // use first NUM_WIRES entries

// Buzzer pin
const uint8_t BUZZER_PIN = 8;

// Button pin (start/reset). Using INPUT_PULLUP: pressed -> LOW
const uint8_t BUTTON_PIN = 9;

// Optional LEDs for status (set to 255 to disable)
const uint8_t LED_GREEN = 10; // lights on defuse
const uint8_t LED_RED   = 11; // lights on boom

// Default countdown time in seconds
const uint16_t DEFAULT_TIME_SECONDS = 30;

// Debounce time (ms) for detecting wire cut to avoid false triggers
const unsigned long WIRE_DEBOUNCE_MS = 50;

// Minimum time between audible tones (avoid overlapping)
const unsigned long TONE_MIN_GAP_MS = 50;

// I2C LCD address and size (adjust if different)
LiquidCrystal_I2C lcd(0x27, 16, 2); // common addr: 0x27 or 0x3F

// ================= END CONFIG =============

// Game states
enum GameState { IDLE, ARMED, COUNTDOWN, DEFUSED, BOOM };
GameState gameState = IDLE;

// Runtime variables
uint16_t timeLeft = DEFAULT_TIME_SECONDS;
unsigned long lastSecondMillis = 0;

// Randomly chosen correct wire index (0..NUM_WIRES-1)
int correctWireIndex = -1;

// Track last stable readings for wires (for debounce)
bool wireLastStable[6];
unsigned long wireLastChangeMillis[6];

// Track if a wire has already been cut (so we don't re-handle)
bool wireAlreadyCut[6];

// For managing tones (to avoid spamming tone calls)
unsigned long lastToneMillis = 0;

// For serial logging (helpful for debugging)
const bool SERIAL_DEBUG = true;

// ---------- Helper function declarations ----------
void armGame();
void resetGame();
void pickCorrectWire();
void handleWireInput();
void handleCountdown();
void playBoomSequence();
void playDefuseSequence();
void displayStatus();
void startTone(unsigned int frequency, unsigned long duration);
bool anyWireCut();
void showWiresOnLCD();
// --------------------------------------------------

void setup() {
  // Initialize serial for debugging
  if (SERIAL_DEBUG) {
    Serial.begin(115200);
    delay(10);
    Serial.println(F("\n--- WireCut Game Starting ---"));
  }

  // Init pins
  for (uint8_t i = 0; i < NUM_WIRES; ++i) {
    pinMode(WIRE_PINS[i], INPUT_PULLUP); // wires: intact => HIGH, cut => LOW
    wireLastStable[i] = true; // assume intact at start
    wireLastChangeMillis[i] = 0;
    wireAlreadyCut[i] = false;
  }

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP); // pressed => LOW

  if (LED_GREEN != 255) { pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, LOW); }
  if (LED_RED   != 255) { pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, LOW); }

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WireCut Ready");
  lcd.setCursor(0, 1);
  lcd.print("Press button...");

  // Seed random using floating analog pin
  randomSeed(analogRead(A0));

  // Ensure initial game state
  resetGame();
}

void loop() {
  // Read button: if pressed (LOW)
  if (digitalRead(BUTTON_PIN) == LOW) {
    // Simple debounce for button (small delay)
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      // If currently IDLE or finished, start/arm game
      if (gameState == IDLE || gameState == DEFUSED || gameState == BOOM) {
        armGame();
        // wait until button released to avoid immediate double reset
        while (digitalRead(BUTTON_PIN) == LOW) { delay(10); }
      } else {
        // If game running and button pressed, treat as reset
        resetGame();
        while (digitalRead(BUTTON_PIN) == LOW) { delay(10); }
      }
    }
  }

  // Main game behavior
  switch (gameState) {
    case IDLE:
      // nothing special; wait for button
      break;
    case ARMED:
      // start countdown immediately on arming
      gameState = COUNTDOWN;
      lastSecondMillis = millis();
      if (SERIAL_DEBUG) Serial.println("Countdown started.");
      break;
    case COUNTDOWN:
      handleWireInput();
      handleCountdown();
      break;
    case DEFUSED:
      // nothing — wait for reset button
      break;
    case BOOM:
      // nothing — wait for reset button
      break;
  }

  // small loop delay to reduce CPU usage
  delay(5);
}

// Arm the game (choose correct wire, reset timers, etc.)
void armGame() {
  timeLeft = DEFAULT_TIME_SECONDS;
  pickCorrectWire();
  for (uint8_t i = 0; i < NUM_WIRES; ++i) {
    wireAlreadyCut[i] = false;
  }
  if (LED_GREEN != 255) digitalWrite(LED_GREEN, LOW);
  if (LED_RED   != 255) digitalWrite(LED_RED, LOW);

  gameState = ARMED;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Armed! Timer:");
  lcd.setCursor(0, 1);
  lcd.print(timeLeft);
  lcd.print(" s");

  if (SERIAL_DEBUG) {
    Serial.print("Armed. Correct wire index = ");
    Serial.println(correctWireIndex);
  }
}

// Reset completely to IDLE
void resetGame() {
  gameState = IDLE;
  timeLeft = DEFAULT_TIME_SECONDS;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WireCut Ready");
  lcd.setCursor(0, 1);
  lcd.print("Press button...");
  if (LED_GREEN != 255) digitalWrite(LED_GREEN, LOW);
  if (LED_RED   != 255) digitalWrite(LED_RED, LOW);
  if (SERIAL_DEBUG) Serial.println("Reset to IDLE.");
}

// Choose a random wire index as the "correct" wire to defuse
void pickCorrectWire() {
  correctWireIndex = random(0, NUM_WIRES);
}

// Non-blocking check of wires with debounce
void handleWireInput() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_WIRES; ++i) {
    bool raw = (digitalRead(WIRE_PINS[i]) == HIGH); // true if intact
    // if raw differs from last stable, update timestamp
    if (raw != wireLastStable[i]) {
      // if last change was long ago, treat as stable change
      if (now - wireLastChangeMillis[i] >= WIRE_DEBOUNCE_MS) {
        // stable transition
        wireLastStable[i] = raw;
        wireLastChangeMillis[i] = now;
        // detect falling edge: intact (HIGH) -> cut (LOW)
        if (!raw && !wireAlreadyCut[i]) {
          // wire has been cut/unplugged
          wireAlreadyCut[i] = true;
          // handle cut
          if (SERIAL_DEBUG) {
            Serial.print("Wire ");
            Serial.print(i);
            Serial.println(" cut!");
          }
          // If cut wire is the correct one
          if ((int)i == correctWireIndex) {
            // defuse the bomb
            gameState = DEFUSED;
            displayStatus();
            playDefuseSequence();
            if (LED_GREEN != 255) digitalWrite(LED_GREEN, HIGH);
            if (SERIAL_DEBUG) Serial.println("DEFUSED!");
          } else {
            // wrong wire -> boom
            gameState = BOOM;
            displayStatus();
            playBoomSequence();
            if (LED_RED != 255) digitalWrite(LED_RED, HIGH);
            if (SERIAL_DEBUG) {
              Serial.print("BOOM! Wrong wire: ");
              Serial.println(i);
            }
          }
        }
      } else {
        // change detected, but within debounce window; update last change time
        wireLastChangeMillis[i] = now;
      }
    } else {
      // no change; refresh last change time for stability
      wireLastChangeMillis[i] = now;
    }
  }
}

// Handle the countdown timer and time-up condition
void handleCountdown() {
  unsigned long now = millis();
  // update each second
  if (now - lastSecondMillis >= 1000) {
    lastSecondMillis = now;
    if (timeLeft > 0) {
      timeLeft--;
      // update display
      lcd.setCursor(0, 0);
      lcd.print("T: ");
      lcd.print(timeLeft);
      lcd.print("s    "); // padding to clear previous text
      // optional: show which wires are still intact on second line
      showWiresOnLCD();
      if (SERIAL_DEBUG) {
        Serial.print("Time left: ");
        Serial.println(timeLeft);
      }
    }
    if (timeLeft == 0 && gameState == COUNTDOWN) {
      // time's up -> boom
      gameState = BOOM;
      displayStatus();
      playBoomSequence();
      if (LED_RED != 255) digitalWrite(LED_RED, HIGH);
      if (SERIAL_DEBUG) Serial.println("Time up -> BOOM!");
    }
  }
}

// Display full status on LCD based on game state
void displayStatus() {
  lcd.clear();
  switch (gameState) {
    case IDLE:
      lcd.setCursor(0, 0);
      lcd.print("WireCut Ready");
      lcd.setCursor(0, 1);
      lcd.print("Press button...");
      break;
    case ARMED:
      lcd.setCursor(0, 0);
      lcd.print("Armed!");
      lcd.setCursor(0, 1);
      lcd.print(timeLeft);
      lcd.print(" s");
      break;
    case COUNTDOWN:
      lcd.setCursor(0, 0);
      lcd.print("T: ");
      lcd.print(timeLeft);
      lcd.print("s    ");
      showWiresOnLCD();
      break;
    case DEFUSED:
      lcd.setCursor(0, 0);
      lcd.print("DEFUSED!");
      lcd.setCursor(0, 1);
      lcd.print("Nice job :)");
      break;
    case BOOM:
      lcd.setCursor(0, 0);
      lcd.print("BOOM!");
      lcd.setCursor(0, 1);
      lcd.print("Game Over");
      break;
  }
}

// Play a short defuse melody (non-blocking tone calls)
void playDefuseSequence() {
  // simple celebratory tones
  startTone(880, 150); delay(180);
  startTone(1320, 120); delay(140);
  startTone(1760, 200);
}

// Play an escalating boom sequence
void playBoomSequence() {
  // quick descending/low thrash to sound "boom"
  startTone(200, 300); delay(320);
  startTone(150, 400); delay(420);
  startTone(100, 600); delay(620);
}

// Centralized tone starter to enforce minimum gap
void startTone(unsigned int frequency, unsigned long duration) {
  unsigned long now = millis();
  if (now - lastToneMillis < TONE_MIN_GAP_MS) {
    // not enough gap; skip
    return;
  }
  lastToneMillis = now;
  // Use Arduino tone() which is non-blocking for duration
  tone(BUZZER_PIN, frequency, duration);
}

// Utility: show wires' intact/cut state on LCD second line
void showWiresOnLCD() {
  // format: W1:1 W2:0 ... or simple pattern: [X][ ][X]
  lcd.setCursor(0, 1);
  for (uint8_t i = 0; i < NUM_WIRES; ++i) {
    bool intact = (digitalRead(WIRE_PINS[i]) == HIGH);
    lcd.print("W");
    lcd.print(i + 1);
    lcd.print(":");
    lcd.print(intact ? "1" : "0");
    if (i < NUM_WIRES - 1) lcd.print(" ");
  }
  // Fill remaining with spaces to clear leftover characters
  for (uint8_t s = 0; s < (16 - (4 * NUM_WIRES)); ++s) lcd.print(" ");
}

// Returns true if any wire was cut (for debug or alternate logic)
bool anyWireCut() {
  for (uint8_t i = 0; i < NUM_WIRES; ++i) if (wireAlreadyCut[i]) return true;
  return false;
}
