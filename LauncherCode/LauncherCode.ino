/*
  ESP32 + ELRS TX — Button Controller with State Machine
  =======================================================
  INAV / Mode 2 / Arm switch on CH5 (AUX1)

  Physical Buttons:
    CH5  (GPIO 4)  — State machine driver
    CH9  (GPIO 18) — Context-dependent action
    BIND (GPIO 19) — Hold 3 seconds → enter/exit bind mode

  State machine:
    IDLE   → CH5 press → ARMED:  CH3(throttle)=1000, CH5=2000
    ARMED  → CH5 press → FLIGHT: CH6=2000
    ARMED  → CH9 press → IDLE:   CH5=1000 (disarm)
    FLIGHT → CH9 press → CH9=2000 (one-shot, no toggle back)

  UART2: TX2=GPIO17 → Ranger Nano signal pin
         RX2=GPIO16 ← (optional)
  Baud : 420000

  INAV channel map (AETR, Mode 2):
    CH1=Roll  CH2=Pitch  CH3=Throttle  CH4=Yaw
    CH5=ARM(AUX1)  CH6=AUX2  CH9=AUX5
*/

#include <Arduino.h>

// ── Pin definitions ──────────────────────────────────────────────
#define BTN_CH5      4
#define BTN_CH9     18
#define BTN_BIND    19
#define LED_BUILTIN  2

// ── CRSF constants ───────────────────────────────────────────────
#define CRSF_SYNC_BYTE     0xC8
#define CRSF_FRAMETYPE_RC  0x16
#define CRSF_FRAME_SIZE    26
#define CRSF_NUM_CHANNELS  16
#define CRSF_BAUDRATE      420000

#define CRSF_1000   172
#define CRSF_1500   992
#define CRSF_2000   1811

// ── State machine ────────────────────────────────────────────────
enum DroneState : uint8_t {
  STATE_IDLE   = 0,
  STATE_ARMED  = 1,
  STATE_FLIGHT = 2
};
DroneState droneState = STATE_IDLE;

// ── Channel array ────────────────────────────────────────────────
uint16_t channels[CRSF_NUM_CHANNELS];

// ── Bind mode ────────────────────────────────────────────────────
bool bindModeActive             = false;
bool bindLedState               = false;
unsigned long lastBindLedToggle = 0;
#define BIND_LED_BLINK_MS 200

// ── Button debounce — index 0=CH5, 1=CH9 ─────────────────────────
unsigned long lastDebounce[2]  = {0, 0};
bool lastBtnState[2]           = {HIGH, HIGH};
bool btnAlreadyFired[2]        = {false, false};
#define DEBOUNCE_MS 80

// ── Bind hold ────────────────────────────────────────────────────
unsigned long bindBtnPressStart    = 0;
bool bindBtnHolding                = false;
bool bindBtnAlreadyTriggered       = false;
#define BIND_HOLD_MS 3000

// ── CRSF CRC8 ────────────────────────────────────────────────────
uint8_t crsf_crc8(const uint8_t *buf, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
  }
  return crc;
}

// ── Send CRSF frame ──────────────────────────────────────────────
void sendCRSF() {
  uint8_t frame[CRSF_FRAME_SIZE];
  frame[0] = CRSF_SYNC_BYTE;
  frame[1] = 24;
  frame[2] = CRSF_FRAMETYPE_RC;

  uint8_t *p = &frame[3];
  memset(p, 0, 22);

  for (uint8_t ch = 0; ch < CRSF_NUM_CHANNELS; ch++) {
    uint32_t bitOffset = (uint32_t)ch * 11;
    uint8_t  byteIdx   = bitOffset / 8;
    uint8_t  bitShift  = bitOffset % 8;
    uint32_t bits      = (uint32_t)(channels[ch] & 0x7FF) << bitShift;
    p[byteIdx]     |= (uint8_t)(bits        & 0xFF);
    p[byteIdx + 1] |= (uint8_t)((bits >> 8) & 0xFF);
    if (bitShift > 5)
      p[byteIdx + 2] |= (uint8_t)((bits >> 16) & 0xFF);
  }

  frame[25] = crsf_crc8(&frame[2], 23);
  Serial2.write(frame, CRSF_FRAME_SIZE);

  static uint32_t cnt = 0;
  if (++cnt % 250 == 0) {
    static const char* stateNames[] = {"IDLE", "ARMED", "FLIGHT"};
    Serial.printf("[CRSF] %lu frames | State=%-6s CH3=%4d CH5=%4d CH6=%4d CH9=%4d\n",
                  cnt, stateNames[droneState],
                  channels[2], channels[4], channels[5], channels[8]);
  }
}

// ── Edge-detect button press (active LOW, debounced) ─────────────
bool buttonPressed(uint8_t idx, uint8_t pin) {
  bool reading = digitalRead(pin);
  if (reading != lastBtnState[idx]) {
    lastDebounce[idx] = millis();
    lastBtnState[idx] = reading;
  }
  if ((millis() - lastDebounce[idx]) > DEBOUNCE_MS && reading == LOW) {
    if (!btnAlreadyFired[idx]) { btnAlreadyFired[idx] = true; return true; }
  }
  if (reading == HIGH) btnAlreadyFired[idx] = false;
  return false;
}

// ── Bind button 3-second hold ─────────────────────────────────────
bool bindButtonHeld() {
  bool reading = digitalRead(BTN_BIND);
  if (reading == LOW) {
    if (!bindBtnHolding) {
      bindBtnHolding          = true;
      bindBtnPressStart       = millis();
      bindBtnAlreadyTriggered = false;
    }
    if (!bindBtnAlreadyTriggered &&
        (millis() - bindBtnPressStart) >= BIND_HOLD_MS) {
      bindBtnAlreadyTriggered = true;
      return true;
    }
  } else {
    bindBtnHolding = false;
  }
  return false;
}

// ── State transition helpers ──────────────────────────────────────
void goIdle() {
  droneState  = STATE_IDLE;
  channels[2] = 0;   // CH3  throttle = 0
  channels[4] = CRSF_1000;   // CH5  ARM switch OFF
  channels[5] = CRSF_1000;   // CH6  AUX2 low
  channels[8] = CRSF_1000;   // CH9  AUX5 low
  Serial.println("[STATE] IDLE — disarmed, all aux low");
}

void goArmed() {
  droneState  = STATE_ARMED;
  channels[2] = 0;   // CH3  throttle = 0  (INAV requires this before arm)
  channels[4] = CRSF_2000;   // CH5  ARM switch ON
  channels[5] = CRSF_1000;   // CH6  stays low
  channels[8] = CRSF_1000;   // CH9  stays low
  Serial.println("[STATE] ARMED — CH5=2000, throttle=0");
}

void goFlight() {
  droneState  = STATE_FLIGHT;
  channels[5] = CRSF_2000;   // CH6  AUX2 = 2000 (flight mode active)
  // CH5 stays 2000 (keep armed), CH3 stays 1000 (pilot controls throttle)
  Serial.println("[STATE] FLIGHT — CH6=2000");
}

// ── Bind mode ────────────────────────────────────────────────────
void enterBindMode() {
  bindModeActive = true;
  for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++) channels[i] = CRSF_1500;
  channels[2] = 0;   // throttle safe during bind
  channels[4] = CRSF_1000;
  channels[5] = CRSF_1000;
  channels[8] = CRSF_1000;
  Serial.println("[BIND] ACTIVE — waiting for RX...");
}

void exitBindMode() {
  bindModeActive = false;
  goIdle();
  Serial.println("[BIND] EXITED — returned to IDLE");
}

void handleBindLed() {
  if (!bindModeActive) { digitalWrite(LED_BUILTIN, LOW); return; }
  unsigned long now = millis();
  if (now - lastBindLedToggle >= BIND_LED_BLINK_MS) {
    lastBindLedToggle = now;
    bindLedState = !bindLedState;
    digitalWrite(LED_BUILTIN, bindLedState);
  }
}

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  Serial2.begin(CRSF_BAUDRATE, SERIAL_8N1, 16, 17);

  pinMode(BTN_CH5,     INPUT_PULLUP);
  pinMode(BTN_CH9,     INPUT_PULLUP);
  pinMode(BTN_BIND,    INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++) channels[i] = CRSF_1500;
  goIdle();   // set controlled channels to safe values

  Serial.println("==============================================");
  Serial.println(" ESP32 ELRS Button Controller — INAV Mode 2");
  Serial.println("==============================================");
  Serial.println(" GPIO4  = CH5 button (arm/flight)");
  Serial.println(" GPIO18 = CH9 button (disarm / CH9 one-shot)");
  Serial.println(" GPIO19 = BIND button (hold 3s)");
  Serial.println("----------------------------------------------");
  Serial.println(" IDLE   + CH5 → ARMED  (CH5=2000, CH3=0)");
  Serial.println(" ARMED  + CH5 → FLIGHT (CH6=2000)");
  Serial.println(" ARMED  + CH9 → IDLE   (disarm)");
  Serial.println(" FLIGHT + CH9 → CH9=2000 (one-shot)");
  Serial.println("==============================================");
}

// ═══════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {

  // BIND hold check (always runs)
  if (bindButtonHeld()) {
    if (!bindModeActive) enterBindMode();
    else                 exitBindMode();
  }

  // Channel buttons blocked during bind
  if (!bindModeActive) {

    // CH5 — drives state forward: IDLE→ARMED→FLIGHT
    if (buttonPressed(0, BTN_CH5)) {
      switch (droneState) {
        case STATE_IDLE:   goArmed();  break;
        case STATE_ARMED:  goFlight(); break;
        case STATE_FLIGHT: break;   // no-op
      }
    }

    // CH9 — disarm if ARMED, one-shot CH9=2000 if FLIGHT
    if (buttonPressed(1, BTN_CH9)) {
      switch (droneState) {
        case STATE_IDLE:   break;   // no-op
        case STATE_ARMED:  goIdle(); break;   // emergency disarm
        case STATE_FLIGHT:
          channels[8] = CRSF_2000;
          Serial.println("[BTN] CH9 → 2000 (one-shot)");
          break;
      }
    }
  }

  handleBindLed();

  // CRSF at ~250Hz
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 4) {
    lastSend = millis();
    sendCRSF();
  }
}