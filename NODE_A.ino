#include <RadioLib.h>
#include <SPI.h>
#include <nrf_gpio.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
 
#define NODE_ID       'A'
 
#define BUILD_DATE  __DATE__
#define BUILD_TIME  __TIME__
 
#define USE_REPEATER  1
 
#define KEYPAD_SELFTEST 0
 
#define LORA_FREQ   915.0
#define LORA_BW     125.0
#define LORA_SF     12
#define LORA_CR     5
#define LORA_SYNC   0x12
#define LORA_PWR    22
#define LORA_PRE    8
#define LORA_TCXO   0.0
#define LORA_LDO    true
 
#define LORA_NSS    1
#define LORA_DIO1   23
#define LORA_RESET  11
#define LORA_SCLK   30
#define LORA_MISO   29
#define LORA_MOSI   28
 
#define BUSY_GPIO   NRF_GPIO_PIN_MAP(1, 4)
#define USE_BUSY_PIN     0
#define BUSY_TIMEOUT_MS  2000
 
#define LCD_SDA_IDX  21
#define LCD_SCL_IDX  20
#define LCD_COLS     16
#define LCD_ROWS     2
 
const uint8_t ROWS = 5, COLS = 4;
 
const uint32_t colPins[COLS] = {
  NRF_GPIO_PIN_MAP(1,  0),
  NRF_GPIO_PIN_MAP(1,  6),
  NRF_GPIO_PIN_MAP(0,  2),
  NRF_GPIO_PIN_MAP(1, 15)
};
 
#define ROW3_ALT_PIN  1
 
#if ROW3_ALT_PIN
  #define ROW3_GPIO   NRF_GPIO_PIN_MAP(1,  4)
  #define ROW3_LABEL  "D8/P1.04"
#else
  #define ROW3_GPIO   NRF_GPIO_PIN_MAP(0, 10)
  #define ROW3_LABEL  "D16/P0.10"
#endif
 
#if ROW3_ALT_PIN && USE_BUSY_PIN
  #error "XUNG DOT CHAN D8: ROW3_ALT_PIN va USE_BUSY_PIN khong the cung bat. Hay dat USE_BUSY_PIN = 0."
#endif
 
const uint32_t rowPins[ROWS] = {
  NRF_GPIO_PIN_MAP(1, 13),
  NRF_GPIO_PIN_MAP(1, 11),
  ROW3_GPIO,
  NRF_GPIO_PIN_MAP(0,  9),
  NRF_GPIO_PIN_MAP(0,  8)
};
 
const char keyMap[ROWS][COLS] = {
  { 'A', 'B', '#', '*' },
  { '1', '2', '3', 'U' },
  { '4', '5', '6', 'D' },
  { '7', '8', '9', 'X' },
  { 'L', '0', 'R', 'E' }
};
 
#if USE_REPEATER
  #define ACK_TIMEOUT_MS   15000
  #define CAD_WAIT_MS       3000
#else
  #define ACK_TIMEOUT_MS    6000
  #define CAD_WAIT_MS        800
#endif
 
#define CAD_WAIT_ALARM_MS  3000
 
#define MAX_RETRY          2
#define MAX_HOP            2
 
#define ACK_DELAY_MS     120
 
#define MAX_MSG_LEN       32
#define RF_HEADER_LEN     10
 
#define TYPE_DATA       'D'
#define TYPE_ACK        'K'
#define TYPE_ALARM      'F'
#define TYPE_CLEAR      'C'
#define DST_BROADCAST   '*'
 
#define IS_VALID_PKT_TYPE(t)                                    \
  ((t) == TYPE_DATA || (t) == TYPE_ACK ||                       \
   (t) == TYPE_ALARM || (t) == TYPE_CLEAR)
 
struct Pkt {
  char     src;
  char     dst;
  char     type;
  uint8_t  hop;
  uint16_t seq;
  String   payload;
  float    rssi;
  float    snr;
};
 
SPIClass lorasSPI(NRF_SPIM2, LORA_MISO, LORA_SCLK, LORA_MOSI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RESET, RADIOLIB_NC, lorasSPI);
 
LiquidCrystal_I2C *lcd = NULL;
bool lcdOk = false;
 
volatile bool receivedFlag = false;
void setFlag(void) { receivedFlag = true; }
 
bool     radioReady = false;
uint16_t mySeq      = 0;
 
uint32_t statTx = 0, statRx = 0, statAckOk = 0, statAckFail = 0;
 
String inputBuffer = "";
String lastMsg     = "";
String lastMsgSrc  = "";
long   lastRSSI    = 0;
long   lastSNR10   = 0;
uint16_t lastSeq   = 0;
uint8_t  lastHop   = 0;
 
#define AUTO_BACK_MS 0
 
#define SCREEN_EDIT    0
#define SCREEN_MSG     1
#define SCREEN_STATS   2
#define SCREEN_ALARM   3
#define SCREEN_LINK    4
 
#define MSG_LINE0_CHARS  (LCD_COLS - 2)
#define MSG_L0_RX_CHARS  (LCD_COLS - 5)
#define MSG_VISIBLE_RX   (MSG_L0_RX_CHARS + LCD_COLS)
 
#define MSG_VISIBLE      (MSG_LINE0_CHARS + LCD_COLS)
 
#define MSG_SCROLL_PAD   3
 
#define MSG_CAPACITY     (MSG_VISIBLE - MSG_SCROLL_PAD)
#define MSG_CAPACITY_RX  (MSG_VISIBLE_RX - MSG_SCROLL_PAD)
 
bool msgNeedScroll = false;
uint8_t msgScrollPos = 0;
unsigned long msgScrollAt = 0;
#define SCROLL_STEP_MS   400
 
bool          transientOn    = false;
String        transientL0    = "";
String        transientL1    = "";
unsigned long transientUntil = 0;
 
#if USE_REPEATER
  #define ALARM_REPEAT_MS        8000
  #define ALARM_REPEAT_ACKED_MS  30000
#else
  #define ALARM_REPEAT_MS        4000
  #define ALARM_REPEAT_ACKED_MS  20000
#endif
 
#define ALARM_BLINK_MS  400
 
#define BUZZER_ENABLED     1
#define BUZZER_PIN         2
 
#define BUZZER_FREQ_ALARM  2000
#define BUZZER_FREQ_CLEAR  900
 
#define BUZZER_ON_MS       200
#define BUZZER_OFF_MS      300
 
#define BUZZER_CLEAR_MS    900
 
#define BUZZER_BEEP_RESULT      1
 
#define BUZZER_FREQ_TX_OK       1800
#define BUZZER_FREQ_TX_FAIL     600
 
#define BUZZER_OK_BEEP_MS       80
#define BUZZER_OK_GAP_MS        70
#define BUZZER_OK_COUNT         2
 
#define BUZZER_FAIL_BEEP_MS     150
#define BUZZER_FAIL_GAP_MS      120
#define BUZZER_FAIL_COUNT       3
 
#define ALARM_RX_HOLD_MS  45000
 
bool          alarmActive    = false;
 
uint16_t      alarmAttempt   = 0;
uint16_t      alarmLastSeq   = 0;
unsigned long alarmNextTxAt  = 0;
unsigned long alarmStartedAt = 0;
 
bool          alarmRxActive  = false;
char          alarmRxSrc     = '\0';
unsigned long alarmRxAt      = 0;
 
bool          blinkOn        = true;
unsigned long blinkAt        = 0;
 
String        alarmNotice      = "";
unsigned long alarmNoticeUntil = 0;
#define ALARM_NOTICE_MS  1500
 
bool alarmRxDismissed = false;
 
uint8_t       buzzerState   = 0;
unsigned long buzzerAt      = 0;
bool          buzzerOn      = false;
 
uint8_t       resBeepLeft   = 0;
uint16_t      resBeepFreq   = 0;
uint16_t      resBeepOnMs   = 0;
uint16_t      resBeepGapMs  = 0;
 
void buzzerSendResult(bool ok);
void buzzerSilence();
 
bool alarmAcked   = false;
bool alarmAckEver = false;
 
unsigned long lastDebounceTime = 0;
char lastKey = '\0';
 
uint8_t screenMode = SCREEN_EDIT;
unsigned long modeSwitchAt = 0;
 
bool needRedraw = true;
 
#define DEDUP_SLOTS 8
struct DedupEntry { char src; uint16_t seq; char type; bool used; };
DedupEntry dedup[DEDUP_SLOTS];
uint8_t dedupIdx = 0;
 
void waitBusy(uint32_t timeoutMs = BUSY_TIMEOUT_MS) {
#if USE_BUSY_PIN
  uint32_t t = millis();
  while (nrf_gpio_pin_read(BUSY_GPIO) == 1) {
    if (millis() - t > timeoutMs) break;
    delay(1);
  }
#else
  (void)timeoutMs;
#endif
}
 
bool enterRx(void) {
  waitBusy();
  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  int16_t st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) {
    if (Serial) { Serial.print("[LOI] startReceive ma: "); Serial.println(st); }
    return false;
  }
  receivedFlag = false;
  return true;
}
 
bool readRfPacket(Pkt &p) {
  uint32_t irq = radio.getIrqFlags();
 
  if (!(irq & RADIOLIB_SX126X_IRQ_RX_DONE)) {
    radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
    return false;
  }
 
  String raw;
  int16_t st = radio.readData(raw);
  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
 
  if (st == RADIOLIB_ERR_CRC_MISMATCH) {
    if (Serial) Serial.println("[!] Goi loi CRC - co song nhung du lieu hong");
    return false;
  }
  if (st != RADIOLIB_ERR_NONE || raw.length() == 0) {
    if (st != RADIOLIB_ERR_NONE && Serial) {
      Serial.print("[!] readData loi ma: "); Serial.println(st);
    }
    return false;
  }
 
  p.rssi = radio.getRSSI();
  p.snr  = radio.getSNR();
 
  int c1 = raw.indexOf(','); if (c1 < 0) return false;
  int c2 = raw.indexOf(',', c1 + 1); if (c2 < 0) return false;
  int c3 = raw.indexOf(',', c2 + 1); if (c3 < 0) return false;
  int c4 = raw.indexOf(',', c3 + 1); if (c4 < 0) return false;
  int c5 = raw.indexOf(',', c4 + 1); if (c5 < 0) return false;
 
  p.src     = raw.charAt(0);
  p.dst     = raw.charAt(c1 + 1);
  p.type    = raw.charAt(c2 + 1);
  p.hop     = (uint8_t)raw.substring(c3 + 1, c4).toInt();
  p.seq     = (uint16_t)raw.substring(c4 + 1, c5).toInt();
  p.payload = raw.substring(c5 + 1);
 
  if (!IS_VALID_PKT_TYPE(p.type)) return false;
  return true;
}
 
String buildPacket(char dst, char type, uint8_t hop, uint16_t seq, const String &payload) {
  String s = String(NODE_ID) + ',' + dst + ',' + type + ',';
  s += (unsigned)hop; s += ',';
  s += (unsigned)seq; s += ',';
  s += payload;
  return s;
}
 
bool waitForChannelFree(uint32_t maxWaitMs) {
  uint32_t t0 = millis();
  for (;;) {
    int16_t st = radio.scanChannel();
    if (st == RADIOLIB_CHANNEL_FREE) return true;
    if (st != RADIOLIB_LORA_DETECTED) {
      if (Serial) { Serial.print("[CAD] loi "); Serial.println(st); }
      return true;
    }
    if (millis() - t0 > maxWaitMs) {
      if (Serial) Serial.println("[CAD] het cho, kenh van ban");
      return false;
    }
    delay(20);
  }
}
 
int16_t txPacket(const String &raw) {
  String tmp = raw;
 
  bool alarmOnNet  = alarmRxActive;
  bool iAmAlarming = alarmActive;
  bool isUrgentPkt = (raw.length() > 4) &&
                     (raw[2] == TYPE_ALARM || raw[2] == TYPE_CLEAR);
 
  uint32_t cadWait = CAD_WAIT_MS;
  if (alarmOnNet || iAmAlarming) {
    if (!isUrgentPkt) cadWait = CAD_WAIT_ALARM_MS;
  }
 
  waitForChannelFree(cadWait);
  waitBusy();
  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  int16_t st = radio.transmit(tmp);
  receivedFlag = false;
  enterRx();
  return st;
}
 
bool seenBefore(char src, uint16_t seq, char type) {
  for (uint8_t i = 0; i < DEDUP_SLOTS; i++) {
    if (dedup[i].used && dedup[i].src == src &&
        dedup[i].seq == seq && dedup[i].type == type) return true;
  }
  return false;
}
 
void rememberPacket(char src, uint16_t seq, char type) {
  dedup[dedupIdx].src  = src;
  dedup[dedupIdx].seq  = seq;
  dedup[dedupIdx].type = type;
  dedup[dedupIdx].used = true;
  dedupIdx = (dedupIdx + 1) % DEDUP_SLOTS;
}
 
bool pollRfPacket(Pkt &out, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    uint32_t irq = radio.getIrqFlags();
    if (receivedFlag || (irq & RADIOLIB_SX126X_IRQ_RX_DONE)) {
      receivedFlag = false;
      if (readRfPacket(out)) return true;
      enterRx();
    }
    delay(1);
  }
  return false;
}
 
void lcdLine(uint8_t row, const String &text) {
  if (!lcdOk) return;
  lcd->setCursor(0, row);
  String s = text;
  if (s.length() > LCD_COLS) s = s.substring(0, LCD_COLS);
  lcd->print(s);
  for (uint8_t i = s.length(); i < LCD_COLS; i++) lcd->write(' ');
}
 
void lcdStatus(const String &line0, const String &line1) {
  if (!lcdOk) return;
  lcdLine(0, line0);
  lcdLine(1, line1);
}
 
void showTransient(const String &line0, const String &line1,
                   unsigned long durationMs) {
  transientOn    = true;
  transientL0    = line0;
  transientL1    = line1;
  transientUntil = millis() + durationMs;
  needRedraw     = true;
}
 
void drawEditScreen() {
  if (!lcdOk) return;
 
  String prefix;
  if (alarmRxActive) {
    prefix = String(NODE_ID) + "!:";
  } else {
    prefix = String(NODE_ID) + ":";
  }
 
  String tail = inputBuffer;
  if (tail.length() > MSG_VISIBLE) {
    tail = tail.substring(tail.length() - MSG_VISIBLE);
  }
 
  uint8_t room0 = LCD_COLS - prefix.length();
  String part0, part1;
  if (tail.length() > room0) {
    part0 = tail.substring(0, room0);
    part1 = tail.substring(room0);
  } else {
    part0 = tail;
    part1 = "";
  }
 
  lcdLine(0, prefix + part0);
  lcdLine(1, part1);
}
 
void drawMsgScreen() {
  if (!lcdOk) return;
 
  String prefix = "TU " + String(lastMsgSrc);
  if (alarmRxActive) prefix += "!";
  else               prefix += ":";
 
  String cur = lastMsg;
  if (msgNeedScroll) {
 
    String padded = String("   ") + lastMsg;
    cur = padded.substring(msgScrollPos, msgScrollPos + MSG_VISIBLE_RX);
  }
 
  String part0, part1;
  if (cur.length() > MSG_L0_RX_CHARS) {
    part0 = cur.substring(0, MSG_L0_RX_CHARS);
    part1 = cur.substring(MSG_L0_RX_CHARS);
  } else {
    part0 = cur;
    part1 = "";
  }
 
  lcdLine(0, prefix + part0);
  lcdLine(1, part1);
}
 
void drawStatsScreen() {
  if (!lcdOk) return;
 
  String l0 = lastMsgSrc + " " + String(lastRSSI) + "dBm";
  l0 += " S" + String(lastSNR10 / 10.0, 1);
  lcdLine(0, l0);
 
  String l1 = "TX" + String(statTx);
  l1 += " RX" + String(statRx);
  lcdLine(1, l1);
}
 
void drawLinkScreen() {
  if (!lcdOk) return;
 
  String l0 = "HOP" + String(lastHop);
  l0 += " SEQ" + String(lastSeq);
  lcdLine(0, l0);
 
  String l1 = "OK" + String(statAckOk);
  l1 += " F" + String(statAckFail);
  lcdLine(1, l1);
}
 
void drawAlarmScreen() {
  if (!lcdOk) return;
 
  lcdLine(0, "CO CHAY XAY RA!");
 
  const char *SUFFIX = " *=huy";
  const uint8_t SUF_LEN = 6;
  const uint8_t STATUS_MAX = LCD_COLS - SUF_LEN;
 
  String st;
  if (alarmActive) {
 
    uint16_t shownAttempt = (alarmAttempt == 0) ? 1 : alarmAttempt;
    st = "x";
    st += (shownAttempt > 999 ? 999 : shownAttempt);
    if (alarmAcked) st += " OK";
  } else {
 
    st = "tu ";
    st += alarmRxSrc;
    st += " lap";
  }
  if (st.length() > STATUS_MAX) st = st.substring(0, STATUS_MAX);
 
  lcdLine(1, st + SUFFIX);
}
 
void redraw() {
 
  if (screenMode == SCREEN_ALARM) {
    drawAlarmScreen();
    drawAlarmNoticeIfDue();
    needRedraw = false;
    return;
  }
 
  if (transientOn) {
    if ((long)(millis() - transientUntil) > 0) {
      transientOn = false;
    } else {
      lcdLine(0, transientL0);
      lcdLine(1, transientL1);
      drawAlarmNoticeIfDue();
      needRedraw = false;
      return;
    }
  }
 
if      (screenMode == SCREEN_EDIT)  drawEditScreen();
  else if (screenMode == SCREEN_MSG)   drawMsgScreen();
  else if (screenMode == SCREEN_STATS) drawStatsScreen();
  else                                 drawLinkScreen();
 
  drawAlarmNoticeIfDue();
  needRedraw = false;
}
 
void drawAlarmNoticeIfDue() {
  if (!lcdOk) return;
  if (alarmNotice.length() == 0) return;
  if ((long)(millis() - alarmNoticeUntil) >= 0) {
    alarmNotice = "";
    return;
  }
  lcdLine(1, alarmNotice);
}
 
void initKeypad() {
  for (uint8_t c = 0; c < COLS; c++) {
    nrf_gpio_cfg_output(colPins[c]);
    nrf_gpio_pin_set(colPins[c]);
  }
  for (uint8_t r = 0; r < ROWS; r++) {
    nrf_gpio_cfg_input(rowPins[r], NRF_GPIO_PIN_PULLUP);
  }
}
 
void ensureNfcPinsAsGpio(void) {
  uint32_t uicr = NRF_UICR->NFCPINS;
  bool     prot = ((uicr & UICR_NFCPINS_PROTECT_Msk) ==
                   (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos));
 
  if (Serial) {
    Serial.print("[NFC] UICR->NFCPINS = 0x");
    Serial.print(uicr, HEX);
    Serial.print("  -> ");
    Serial.println(prot ? "DANG CHAN (P0.09/P0.10 khong lam GPIO duoc)"
                        : "khong chan (P0.09/P0.10 la GPIO binh thuong)");
  }
 
  if (!prot) return;
 
  if (Serial) {
    Serial.println("[NFC] Dang mo khoa NFC trong UICR...");
    Serial.println("[NFC] May se TU RESET ngay sau day. Doi no khoi dong lai.");
    Serial.flush();
  }
  delay(300);
 
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
  NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
 
  NVIC_SystemReset();
}
 
void printRowRawStates(void) {
  if (!Serial) return;
  Serial.print("  [RAW HANG]");
  for (uint8_t r = 0; r < ROWS; r++) {
    Serial.print(" H");
    Serial.print(r + 1);
    Serial.print('=');
    Serial.print(nrf_gpio_pin_read(rowPins[r]));
  }
  Serial.println();
}
 
void printColumnScanOnce(void) {
  if (!Serial) return;
 
  const char *colName[COLS] = { "D6 ", "D9 ", "D19", "D18" };
 
  for (uint8_t c = 0; c < COLS; c++) {
    nrf_gpio_pin_clear(colPins[c]);
    delayMicroseconds(20);
 
    Serial.print("  [COT");
    Serial.print(c + 1);
    Serial.print(' ');
    Serial.print(colName[c]);
    Serial.print("]");
    for (uint8_t r = 0; r < ROWS; r++) {
      Serial.print(' ');
      Serial.print(nrf_gpio_pin_read(rowPins[r]));
    }
 
    nrf_gpio_pin_set(colPins[c]);
    Serial.println();
  }
}
 
void testColPinsElectrical(void) {
  if (!Serial) return;
 
  const char *colName[COLS] = {
    "Cot 1 (D6 /P1.00)", "Cot 2 (D9 /P1.06)",
    "Cot 3 (D19/P0.02)", "Cot 4 (D18/P1.15)"
  };
 
  Serial.println("------------------------------------------");
  Serial.println("  TEST DIEN TUNG CHAN COT (ep OUTPUT roi doc lai)");
  Serial.println("  Khong can ban phim. Chan nao [HONG] la chan do chet.");
  Serial.println("------------------------------------------");
 
  uint8_t failCount = 0;
  for (uint8_t c = 0; c < COLS; c++) {
    uint32_t pin = colPins[c];
 
    nrf_gpio_cfg_output(pin);
    nrf_gpio_pin_clear(pin);
    delayMicroseconds(50);
    uint32_t low = nrf_gpio_pin_read(pin);
    nrf_gpio_pin_set(pin);
    delayMicroseconds(50);
    uint32_t high = nrf_gpio_pin_read(pin);
 
    bool ok = (low == 0) && (high == 1);
    if (!ok) failCount++;
 
    Serial.print("  ");
    Serial.print(colName[c]);
    Serial.print("  ->  LOW="); Serial.print(low);
    Serial.print("  HIGH=");    Serial.print(high);
    Serial.println(ok ? "   [OK]" : "   [HONG]");
  }
 
  Serial.println("------------------------------------------");
  if (failCount == 0) {
    Serial.println("  => Ca 4 chan cot deu tot o muc GPIO.");
    Serial.println("     Neu van co phim chet => NGHI VAN: switch phim");
    Serial.println("     hoac day HANG, khong phai chan cot.");
  } else {
    Serial.print("  => Co "); Serial.print(failCount);
    Serial.println(" chan cot HONG.");
    Serial.println("     Moi phim thuoc cot do se chet theo.");
  }
  Serial.println("------------------------------------------");
 
  initKeypad();
}
 
void testRowPinsElectrical(void) {
  if (!Serial) return;
 
  const char *rowName[ROWS] = {
    "Hang 1 (D15/P1.13)", "Hang 2 (D14/P1.11)",
    "Hang 3 (D16/P0.10 NFC2)", "Hang 4 (D10/P0.09 NFC1)", "Hang 5 (D0/P0.08)"
  };
#if ROW3_ALT_PIN
  rowName[2] = "Hang 3 (D8/P1.04)";
#endif
 
  Serial.println("------------------------------------------");
  Serial.println("  TEST DIEN TUNG CHAN HANG (ep OUTPUT roi doc lai)");
  Serial.println("------------------------------------------");
 
  uint8_t failCount = 0;
 
  for (uint8_t r = 0; r < ROWS; r++) {
    uint32_t pin = rowPins[r];
 
    nrf_gpio_cfg_output(pin);
 
    nrf_gpio_pin_clear(pin);
    delayMicroseconds(50);
    uint32_t low  = nrf_gpio_pin_read(pin);
 
    nrf_gpio_pin_set(pin);
    delayMicroseconds(50);
    uint32_t high = nrf_gpio_pin_read(pin);
 
    bool ok = (low == 0) && (high == 1);
    if (!ok) failCount++;
 
    Serial.print("  ");
    Serial.print(rowName[r]);
    Serial.print("  ->  LOW="); Serial.print(low);
    Serial.print("  HIGH=");    Serial.print(high);
    Serial.println(ok ? "   [OK]" : "   [HONG]");
  }
 
  Serial.println("------------------------------------------");
  if (failCount == 0) {
    Serial.println("  => Tat ca chan hang deu tot.");
    Serial.println("     Neu phim van khong an, loi nam o day COT.");
  } else {
    Serial.print("  => Co "); Serial.print(failCount);
    Serial.println(" chan hang HONG.");
    Serial.println("     Xem dong [HONG] o tren de biet chan nao.");
  }
  Serial.println("------------------------------------------");
 
#if ROW3_ALT_PIN
 
  {
    uint32_t oldPin = NRF_GPIO_PIN_MAP(0, 10);
    nrf_gpio_cfg_output(oldPin);
    nrf_gpio_pin_clear(oldPin);
    delayMicroseconds(50);
    uint32_t lo = nrf_gpio_pin_read(oldPin);
    nrf_gpio_pin_set(oldPin);
    delayMicroseconds(50);
    uint32_t hi = nrf_gpio_pin_read(oldPin);
    bool ok = (lo == 0) && (hi == 1);
 
    Serial.print("  [Chan cu] D16/P0.10  ->  LOW="); Serial.print(lo);
    Serial.print("  HIGH="); Serial.print(hi);
    Serial.println(ok ? "   [OK - chan tot, loi nam o day noi]"
                      : "   [HONG - chan chet, phai dung D8]");
    Serial.println("------------------------------------------");
 
    nrf_gpio_cfg_input(oldPin, NRF_GPIO_PIN_NOPULL);
  }
#endif
 
  initKeypad();
}
 
char scanKeypad() {
  for (uint8_t c = 0; c < COLS; c++) {
    nrf_gpio_pin_clear(colPins[c]);
    delayMicroseconds(15);
    for (uint8_t r = 0; r < ROWS; r++) {
      if (nrf_gpio_pin_read(rowPins[r]) == 0) {
        nrf_gpio_pin_set(colPins[c]);
        return keyMap[r][c];
      }
    }
    nrf_gpio_pin_set(colPins[c]);
  }
  return '\0';
}
 
bool scanKeypadRC(uint8_t &rowOut, uint8_t &colOut) {
  for (uint8_t c = 0; c < COLS; c++) {
    nrf_gpio_pin_clear(colPins[c]);
    delayMicroseconds(15);
    for (uint8_t r = 0; r < ROWS; r++) {
      if (nrf_gpio_pin_read(rowPins[r]) == 0) {
        nrf_gpio_pin_set(colPins[c]);
        rowOut = r; colOut = c;
        return true;
      }
    }
    nrf_gpio_pin_set(colPins[c]);
  }
  return false;
}
 
#define SELFTEST_AUTOEXIT_SEC  30
 
void keypadSelfTest() {
  const unsigned long deadline = millis() + (unsigned long)SELFTEST_AUTOEXIT_SEC * 1000UL;
 
  if (Serial) {
    Serial.println("==========================================");
    Serial.println("  CHE DO CHAN DOAN BAN PHIM");
    Serial.print("  TU THOAT sau "); Serial.print(SELFTEST_AUTOEXIT_SEC);
    Serial.println(" giay (hoac bam 'E', hoac go 'q')");
    Serial.println("==========================================");
    Serial.println("  CACH DO - QUAN TRONG:");
    Serial.println("  1) KHONG giu phim nao -> in [RAW HANG] + [COT..]");
    Serial.println("  2) GIU phim F2 (hang1 cot2) va nhin dong [COT2]:");
    Serial.println("       [COT2] 0 1 1 1 1  -> cot D9 TOT, switch TOT");
    Serial.println("       [COT2] 1 1 1 1 1  -> cot D9 chet HOAC switch F2 hong");
    Serial.println("  3) Lap lai voi phim '#' (COT3) va '*' (COT4).");
    Serial.println("  4) Bam phim bang tay -> in toa do HANG/COT + ky tu");
    Serial.println("==========================================");
  }
 
  const char *rowGpio[ROWS] = { "D15/P1.13", "D14/P1.11", ROW3_LABEL,
                                "D10/P0.09", "D0/P0.08" };
  const char *colGpio[COLS] = { "D6/P1.00", "D9/P1.06", "D19/P0.02", "D18/P1.15" };
 
  uint8_t r = 0, c = 0;
  unsigned long lastRaw = 0;
 
  while (true) {
 
    if ((long)(millis() - deadline) >= 0) {
      if (Serial) {
        Serial.println();
        Serial.print("[CHAN DOAN] Het ");
        Serial.print(SELFTEST_AUTOEXIT_SEC);
        Serial.println(" giay -> TU THOAT, tiep tuc khoi dong binh thuong.");
      }
      return;
    }
 
    if (Serial.available() > 0) {
      char ch = (char)Serial.read();
      if (ch == 'q' || ch == 'Q') { Serial.println("[CHAN DOAN] Thoat."); return; }
    }
 
    if (millis() - lastRaw >= 500) {
      lastRaw = millis();
      printRowRawStates();
      printColumnScanOnce();
      Serial.println("  ^ GIU 1 phim roi so sanh 4 dong [COT..] o tren");
    }
 
    if (scanKeypadRC(r, c)) {
      char mapped = keyMap[r][c];
 
      const char *role;
      switch (mapped) {
        case 'E': role = "CHUC NANG: thoat selftest / GUI tin"; break;
        case 'U': role = "CHUC NANG: xoa lui 1 ky tu";          break;
        case 'X': role = "CHUC NANG: xoa het";                  break;
        case 'D': role = "CHUC NANG: doi man hinh";             break;
        case '#': role = "KHAN CAP: BAO CHAY (SOS)";            break;
        case '*': role = "KHAN CAP: HUY bao dong";              break;
        default:  role = "NHAP LIEU: hien tren LCD";            break;
      }
 
      if (Serial) {
        Serial.print("  >>> HANG="); Serial.print(r);
        Serial.print(" ("); Serial.print(rowGpio[r]); Serial.print(")");
        Serial.print("  COT="); Serial.print(c);
        Serial.print(" ("); Serial.print(colGpio[c]); Serial.print(")");
        Serial.print("   ->  '"); Serial.print(mapped);
        Serial.print("'  ["); Serial.print(role); Serial.println("]");
      }
      if (lcdOk) {
 
        String tag = "phim nhap lieu";
        if (mapped == '#' || mapped == '*')      tag = "phim KHAN CAP";
        else if (c == 3)                          tag = "phim chuc nang";
        lcdLine(0, "H" + String(r) + " C" + String(c) + " -> '" + String(mapped) + "'");
        lcdLine(1, tag);
      }
 
      if (mapped == 'E') { Serial.println("[CHAN DOAN] Thoat bang phim E."); return; }
 
      delay(250);
      while (scanKeypadRC(r, c)) { delay(20); }
    }
    delay(5);
  }
}
 
void deliverToUser(const Pkt &p) {
  statRx++;
  lastMsg    = p.payload;
  lastMsgSrc = String(p.src);
  lastRSSI   = (long)p.rssi;
  lastSNR10  = (long)(p.snr * 10);
  lastSeq    = p.seq;
  lastHop    = p.hop;
 
  msgNeedScroll = (lastMsg.length() > MSG_CAPACITY_RX);
  msgScrollPos  = 0;
  msgScrollAt   = millis();
 
  transientOn = false;
 
  if (!alarmActive && !alarmRxActive) {
    screenMode   = SCREEN_MSG;
    modeSwitchAt = millis();
  }
  needRedraw   = true;
 
  if (Serial) {
    Serial.println("******************************************");
    Serial.print("[RX #"); Serial.print(statRx);
    Serial.print("] \""); Serial.print(p.payload); Serial.println("\"");
    Serial.print("  src="); Serial.print(p.src);
    Serial.print("  hop="); Serial.print(p.hop);
    Serial.print("  seq="); Serial.println(p.seq);
    Serial.print("  RSSI: "); Serial.print(p.rssi, 1); Serial.print(" dBm");
    Serial.print(" | SNR: ");  Serial.print(p.snr, 1);  Serial.println(" dB");
    Serial.println("******************************************");
  }
 
}
 
void sendAckFor(const Pkt &p) {
  delay(ACK_DELAY_MS);
  String ack = buildPacket(p.src, TYPE_ACK, 0, p.seq, "");
  int16_t st = txPacket(ack);
  if (Serial) {
    if (st == RADIOLIB_ERR_NONE) Serial.println("[ACK] da gui xac nhan");
    else { Serial.print("[!] gui ACK loi ma: "); Serial.println(st); }
  }
}
 
void handleDataPacket(const Pkt &p) {
  if (p.src == NODE_ID) return;
  if (p.dst != NODE_ID && p.dst != DST_BROADCAST) return;
 
  if (seenBefore(p.src, p.seq, p.type)) return;
  rememberPacket(p.src, p.seq, p.type);
 
  deliverToUser(p);
  sendAckFor(p);
}
 
void handleIncoming() {
  uint32_t irq = radio.getIrqFlags();
  if (!receivedFlag && !(irq & RADIOLIB_SX126X_IRQ_RX_DONE)) return;
  receivedFlag = false;
 
  Pkt p;
  if (!readRfPacket(p)) { enterRx(); return; }
 
  if (p.type == TYPE_DATA) {
    handleDataPacket(p);
  } else if (p.type == TYPE_ALARM) {
 
    handleAlarmPacket(p);
  } else if (p.type == TYPE_CLEAR) {
 
    handleClearPacket(p);
  } else if (p.type == TYPE_ACK) {
 
    if (p.dst == NODE_ID && alarmActive && p.seq == alarmLastSeq) {
      if (!alarmAcked && Serial) {
        Serial.print("[SOS] Bao dong DA DUOC xac nhan (seq=");
        Serial.print(p.seq);
        Serial.println("). Chuyen sang che do lap cham.");
      }
      alarmAcked   = true;
      alarmAckEver = true;
      needRedraw = true;
    }
    else if (p.dst == NODE_ID && Serial) {
 
      Serial.print("[ACK] nhan xac nhan lang (seq=");
      Serial.print(p.seq); Serial.println(")");
    }
  }
  enterRx();
}
 
bool sendWithAck(const String &msg, uint32_t &rttMs) {
  for (uint8_t attempt = 1; attempt <= MAX_RETRY + 1; attempt++) {
 
  alarmAcked = false;
 
  mySeq = (mySeq + 1) % 1000;
    uint16_t seq = mySeq;
 
    String packet = buildPacket(DST_BROADCAST, TYPE_DATA, 0, seq, msg);
 
    if (Serial) {
      Serial.print("  [LAN "); Serial.print(attempt);
      Serial.print("/"); Serial.print(MAX_RETRY + 1);
      Serial.print("] seq="); Serial.print(seq);
      Serial.print(" len="); Serial.print(packet.length());
      Serial.println(" byte, dang phat...");
    }
    if (lcdOk) {
      String s = "Dang gui ";
      s += attempt; s += "/"; s += (MAX_RETRY + 1);
      lcdStatus(s, msg);
    }
 
    uint32_t t_start = millis();
    int16_t st = txPacket(packet);
    if (st != RADIOLIB_ERR_NONE && Serial) {
      Serial.print("  PHAT LOI ma: "); Serial.println(st);
    }
 
    uint32_t t0 = millis();
    uint32_t deadline = t0 + ACK_TIMEOUT_MS;
 
    while (millis() < deadline) {
      Pkt p;
      if (!pollRfPacket(p, deadline - millis())) break;
 
      if (p.type == TYPE_ACK && p.dst == NODE_ID && p.seq == seq) {
        rttMs = millis() - t_start;
        return true;
      }
 
      if (p.type == TYPE_DATA && p.src != NODE_ID &&
          (p.dst == NODE_ID || p.dst == DST_BROADCAST)) {
        if (!seenBefore(p.src, p.seq, p.type)) {
          rememberPacket(p.src, p.seq, p.type);
          deliverToUser(p);
          sendAckFor(p);
        }
        continue;
      }
    }
    if (Serial) Serial.println("  khong co ACK");
  }
  return false;
}
 
void sendMessage(const String &msg) {
  if (!radioReady) {
    if (Serial) Serial.println("[!] Radio chua san sang");
    showTransient("Radio loi", "chua san sang", 3000);
    return;
  }
 
  if (alarmActive) {
    if (Serial) {
      Serial.println("[SOS] Node nay dang phat bao dong - tu choi gui tin thuong.");
      Serial.println("      Huy bao dong bang '*' (hoac lenh 'huy') roi moi gui.");
    }
 
    return;
  }
 
  statTx++;
  if (Serial) {
    Serial.println();
    Serial.println("----------------------------------------");
    Serial.print("[TX #"); Serial.print(statTx);
    Serial.print("] GUI: \""); Serial.print(msg); Serial.println("\"");
    Serial.println("----------------------------------------");
  }
 
  uint32_t rtt = 0;
 
  uint32_t rxBefore = statRx;
 
  bool ok = sendWithAck(msg, rtt);
 
  bool gotMsgDuringSend = (statRx != rxBefore);
 
  if (ok) {
    statAckOk++;
    if (Serial) { Serial.print("-> [OK] ACK, RTT "); Serial.print(rtt); Serial.println(" ms"); }
 
    buzzerSendResult(true);
 
    if (gotMsgDuringSend) {
 
      screenMode   = SCREEN_MSG;
      modeSwitchAt = millis();
      needRedraw   = true;
    } else {
 
      showTransient("OK da gui", "RTT " + String((long)rtt) + " ms", 3000);
    }
  } else {
    statAckFail++;
    if (Serial) Serial.println("-> [FAIL] khong nhan duoc ACK");
 
    buzzerSendResult(false);
 
    if (gotMsgDuringSend) {
      screenMode   = SCREEN_MSG;
      modeSwitchAt = millis();
      needRedraw   = true;
    } else {
      showTransient("FAIL ko ACK", "thu lai hoac doi vi tri", 3000);
    }
  }
 
}
 
void alarmStart() {
  alarmActive   = true;
  alarmAcked    = false;
  alarmAckEver  = false;
  alarmAttempt  = 0;
  alarmStartedAt = millis();
  alarmNextTxAt = millis();
  blinkOn       = true;
  blinkAt       = millis();
 
  alarmRxDismissed = true;
 
  screenMode  = SCREEN_ALARM;
  transientOn = false;
  needRedraw  = true;
 
  if (Serial) {
    Serial.println();
    Serial.println("##########################################");
    Serial.print("#  [SOS] NODE "); Serial.print(NODE_ID);
    Serial.println(" KICH HOAT BAO DONG  #");
    Serial.println("#  Phim '*' de huy bao dong");
    Serial.println("##########################################");
  }
}
 
void alarmCancel() {
  if (!alarmActive) {
 
    alarmRxActive = false;
    if (screenMode == SCREEN_ALARM) {
      screenMode = SCREEN_EDIT;
      needRedraw = true;
    }
    if (lcdOk && !blinkOn) { lcd->backlight(); blinkOn = true; }
 
    if (Serial) {
      Serial.print("[SOS] Da AN canh bao tu node ");
      Serial.print(alarmRxSrc);
      Serial.println(". Canh bao se HIEN LAI khi node do phat goi lap ke tiep.");
      Serial.print("      Chi node ");
      Serial.print(alarmRxSrc);
      Serial.println(" (nguon phat) moi huy duoc bang phim '*'.");
    }
    return;
  }
 
  alarmActive  = false;
  alarmAcked   = false;
  alarmAckEver = false;
 
  mySeq = (mySeq + 1) % 1000;
  String pkt = buildPacket(DST_BROADCAST, TYPE_CLEAR, 0, mySeq, "HUY BAO DONG");
  txPacket(pkt);
 
  if (Serial) {
    Serial.println();
    Serial.println("##########################################");
    Serial.print("#  [SOS] NODE "); Serial.print(NODE_ID);
    Serial.println(" DA HUY BAO DONG  #");
    Serial.println("##########################################");
  }
 
  screenMode = SCREEN_EDIT;
  needRedraw = true;
 
  if (lcdOk && !blinkOn) { lcd->backlight(); blinkOn = true; }
}
 
void alarmTick() {
  if (!alarmActive || !radioReady) return;
 
  if ((long)(millis() - alarmNextTxAt) < 0) return;
 
  mySeq = (mySeq + 1) % 1000;
  alarmLastSeq = mySeq;
  alarmAttempt++;
 
  String pkt = buildPacket(DST_BROADCAST, TYPE_ALARM, 0, mySeq, "BAO CHAY");
 
  if (Serial) {
    Serial.print("[SOS] Phat bao dong lan "); Serial.print(alarmAttempt);
    Serial.print(" seq="); Serial.println(mySeq);
  }
 
  int16_t st = txPacket(pkt);
  if (st != RADIOLIB_ERR_NONE && Serial) {
    Serial.print("[SOS] phat loi ma: "); Serial.println(st);
  }
 
  unsigned long gap = alarmAckEver ? ALARM_REPEAT_ACKED_MS : ALARM_REPEAT_MS;
  alarmNextTxAt = millis() + gap;
 
  needRedraw = true;
}
 
void handleAlarmPacket(const Pkt &p) {
 
  if (p.src == NODE_ID) return;
 
  if (seenBefore(p.src, p.seq, p.type)) {
 
    alarmRxAt = millis();
    return;
  }
  rememberPacket(p.src, p.seq, p.type);
 
  bool newEpisode = (!alarmRxActive) || (alarmRxSrc != p.src);
  if (newEpisode) alarmRxDismissed = false;
 
  alarmRxActive = true;
  alarmRxSrc    = p.src;
  alarmRxAt     = millis();
 
  if (!alarmActive && !alarmRxDismissed) {
    screenMode  = SCREEN_ALARM;
    transientOn = false;
    needRedraw  = true;
  } else {
    needRedraw = true;
  }
 
  if (Serial) {
    Serial.println();
    Serial.println("##########################################");
    Serial.print("#  [CANH BAO] NHAN BAO DONG TU NODE ");
    Serial.print(p.src);
    Serial.print("  #");
    Serial.print("  RSSI "); Serial.print(p.rssi, 1); Serial.print(" dBm");
    Serial.print(" | SNR "); Serial.print(p.snr, 1);  Serial.println(" dB");
    Serial.println("#  Node khac dang bao chay. Kiem tra khu vuc!");
    Serial.println("##########################################");
  }
 
  sendAckFor(p);
}
 
void handleClearPacket(const Pkt &p) {
 
  if (p.src == NODE_ID) return;
 
  if (seenBefore(p.src, p.seq, p.type)) return;
  rememberPacket(p.src, p.seq, p.type);
 
  if (Serial) {
    Serial.println();
    Serial.print("[SOS] Node "); Serial.print(p.src);
    Serial.println(" DA HUY bao dong.");
  }
 
  if (!alarmActive && p.src == alarmRxSrc) {
    alarmRxActive = false;
    alarmRxSrc    = '\0';
    if (screenMode == SCREEN_ALARM) {
      screenMode = SCREEN_EDIT;
      needRedraw = true;
    }
    if (lcdOk) showTransient("Da het bao dong", String("tu node ") + p.src, 4000);
  }
  sendAckFor(p);
}
 
void buzzerTick() {
#if !BUZZER_ENABLED
  return;
#else
  bool anyAlarm = alarmActive || alarmRxActive;
  unsigned long now = millis();
 
  if (anyAlarm) {
 
    if (buzzerState == 3) {
      resBeepLeft = 0;
    }
    if (buzzerState != 1) {
      buzzerState = 1;
      buzzerOn    = true;
      buzzerAt    = now;
      tone(BUZZER_PIN, BUZZER_FREQ_ALARM);
      return;
    }
 
    if (buzzerOn) {
      if (now - buzzerAt >= BUZZER_ON_MS) {
        noTone(BUZZER_PIN);
        buzzerOn = false;
        buzzerAt = now;
      }
    } else {
      if (now - buzzerAt >= BUZZER_OFF_MS) {
        tone(BUZZER_PIN, BUZZER_FREQ_ALARM);
        buzzerOn = true;
        buzzerAt = now;
      }
    }
    return;
  }
 
  if (buzzerState == 3) {
    if (buzzerOn) {
 
      if (now - buzzerAt >= resBeepOnMs) {
        noTone(BUZZER_PIN);
        buzzerOn = false;
        buzzerAt = now;
        resBeepLeft--;
      }
    } else {
 
      if (now - buzzerAt >= resBeepGapMs) {
        if (resBeepLeft == 0) {
 
          buzzerState = 0;
        } else {
          tone(BUZZER_PIN, resBeepFreq);
          buzzerOn = true;
          buzzerAt = now;
        }
      }
    }
    return;
  }
 
  if (buzzerState == 2) {
    if (now - buzzerAt >= BUZZER_CLEAR_MS) {
      noTone(BUZZER_PIN);
      buzzerState = 0;
      buzzerOn    = false;
      buzzerAt    = now;
    }
    return;
  }
 
  if (buzzerState == 1) {
 
    noTone(BUZZER_PIN);
    tone(BUZZER_PIN, BUZZER_FREQ_CLEAR);
    buzzerState = 2;
    buzzerAt    = now;
    buzzerOn    = true;
    if (Serial) Serial.println("[BUZZER] Bao dong ket thuc -> tieng tram");
  }
#endif
}
 
void buzzerSendResult(bool ok) {
#if BUZZER_ENABLED && BUZZER_BEEP_RESULT
 
  if (alarmActive || alarmRxActive) {
    if (Serial) Serial.println("[BUZZER] Dang bao dong -> bo qua tieng ket qua");
    return;
  }
 
  if (ok) {
    resBeepFreq  = BUZZER_FREQ_TX_OK;
    resBeepOnMs  = BUZZER_OK_BEEP_MS;
    resBeepGapMs = BUZZER_OK_GAP_MS;
    resBeepLeft  = BUZZER_OK_COUNT;
  } else {
    resBeepFreq  = BUZZER_FREQ_TX_FAIL;
    resBeepOnMs  = BUZZER_FAIL_BEEP_MS;
    resBeepGapMs = BUZZER_FAIL_GAP_MS;
    resBeepLeft  = BUZZER_FAIL_COUNT;
  }
 
  buzzerState = 3;
  buzzerOn    = true;
  buzzerAt    = millis();
  tone(BUZZER_PIN, resBeepFreq);
 
  if (Serial) {
    Serial.print("[BUZZER] Ket qua gui: ");
    Serial.print(ok ? "OK" : "FAIL");
    Serial.print(" -> ");
    Serial.print(resBeepLeft);
    Serial.print(" tieng @ ");
    Serial.print(resBeepFreq);
    Serial.println(" Hz");
  }
#endif
}
 
void buzzerSilence() {
  noTone(BUZZER_PIN);
  buzzerState = 0;
  buzzerOn    = false;
  resBeepLeft = 0;
}
 
void alarmBlinkTick() {
  bool anyAlarm = alarmActive || alarmRxActive;
 
  if (!anyAlarm) {
 
    if (lcdOk && !blinkOn) { lcd->backlight(); blinkOn = true; }
    return;
  }
 
  if (!alarmActive && alarmRxActive &&
      (millis() - alarmRxAt > ALARM_RX_HOLD_MS)) {
    alarmRxActive = false;
    if (screenMode == SCREEN_ALARM) {
      screenMode = SCREEN_EDIT;
      needRedraw = true;
    }
    if (Serial) {
      Serial.println("[SOS] Het nghe bao dong tu node khac -> tat canh bao.");
    }
    return;
  }
 
  if (!lcdOk) return;
  if ((long)(millis() - blinkAt) < ALARM_BLINK_MS) return;
 
  blinkAt = millis();
  blinkOn = !blinkOn;
  if (blinkOn) lcd->backlight(); else lcd->noBacklight();
}
 
void handleKey(char key) {
 
  if (key == '#') {
    if (alarmActive) {
 
      alarmNotice      = "Dang bao dong";
      alarmNoticeUntil = millis() + ALARM_NOTICE_MS;
      if (Serial) Serial.println("[SOS] Bao dong dang chay. Bam '*' de huy.");
      needRedraw = true;
    } else {
 
      if (alarmRxActive && Serial) {
        Serial.println("[SOS] Node nay cung phat bao dong (truoc do dang nghe tu node khac).");
      }
      alarmStart();
    }
    return;
  }
 
  if (key == '*') {
    if (alarmActive) {
 
      alarmCancel();
    } else if (alarmRxActive) {
 
      alarmRxDismissed = true;
      if (screenMode == SCREEN_ALARM) {
        screenMode = SCREEN_EDIT;
      }
      alarmNotice      = "Da thay, go tin";
      alarmNoticeUntil = millis() + ALARM_NOTICE_MS;
      if (Serial) {
        Serial.println("[SOS] Da XAC NHAN canh bao. Quay ve go tin duoc.");
        Serial.println("      Canh bao van con hoat dong (dau '!' tren man hinh).");
        Serial.print("      Chi node ");
        Serial.print(alarmRxSrc);
        Serial.println(" (nguon phat) moi huy het duoc bang '*'.");
      }
      needRedraw = true;
    } else {
 
      if (Serial) Serial.println("[SOS] Khong co bao dong nao de huy.");
      if (lcdOk) showTransient("Ko co bao dong", "'#'=SOS '*'=huy", 1500);
    }
    return;
  }
 
  if (alarmActive) {
    if (key == 'D') {
 
      switch (screenMode) {
        case SCREEN_MSG:   screenMode = SCREEN_STATS; break;
        case SCREEN_STATS: screenMode = SCREEN_LINK;  break;
        case SCREEN_LINK:  screenMode = SCREEN_ALARM; break;
        default:           screenMode = SCREEN_MSG;   break;
      }
 
      modeSwitchAt = millis();
 
      msgScrollPos = 0;
      msgScrollAt  = millis();
      needRedraw = true;
      return;
    }
 
    if (Serial) {
      Serial.println("[SOS] Node nay dang phat bao dong. Chi nhan '#'/'*'/'D'.");
    }
    alarmNotice      = "Dang phat, chi D";
    alarmNoticeUntil = millis() + ALARM_NOTICE_MS;
    needRedraw = true;
    return;
  }
 
  if (key != 'E' && transientOn) {
    transientOn = false;
  }
 
  if (key == 'D') {
    switch (screenMode) {
      case SCREEN_EDIT:  screenMode = SCREEN_MSG;   break;
      case SCREEN_MSG:   screenMode = SCREEN_STATS; break;
      case SCREEN_STATS: screenMode = SCREEN_LINK;  break;
      case SCREEN_LINK:  screenMode = SCREEN_EDIT;  break;
      default:           screenMode = SCREEN_MSG;   break;
    }
 
    modeSwitchAt = millis();
 
    msgScrollPos = 0;
    msgScrollAt  = millis();
    needRedraw = true;
    return;
  }
 
  if (screenMode != SCREEN_EDIT) {
 
    if (screenMode == SCREEN_ALARM && alarmRxActive && !alarmActive) {
      alarmRxDismissed = true;
    }
    screenMode = SCREEN_EDIT;
    needRedraw = true;
  }
 
  if (key == 'E') {
    if (inputBuffer.length() == 0) {
 
      showTransient("Chua co noi dung", "", 900);
      return;
    }
    String outgoing = inputBuffer;
    inputBuffer = "";
    sendMessage(outgoing);
    return;
  }
 
  if (key == 'U') {
    if (inputBuffer.length() > 0) inputBuffer.remove(inputBuffer.length() - 1);
    needRedraw = true;
    return;
  }
 
  if (key == 'X') {
    inputBuffer = "";
    needRedraw = true;
    return;
  }
 
  if (inputBuffer.length() < MAX_MSG_LEN) {
    inputBuffer += key;
    needRedraw = true;
  } else {
 
    showTransient("Tin qua dai", String(MAX_MSG_LEN) + " ky tu max", 900);
  }
}
 
void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) delay(10);
  delay(200);
 
  Serial.println();
  Serial.println("==========================================");
  Serial.print("  LORA TERMINAL  ID='"); Serial.print(NODE_ID);
  Serial.println("'  (v6 co SOS)");
 
  Serial.print("  Firmware build: ");
  Serial.print(BUILD_DATE); Serial.print(" "); Serial.println(BUILD_TIME);
  Serial.println("==========================================");
 
  Wire.setPins(LCD_SDA_IDX, LCD_SCL_IDX);
  Wire.begin();
 
  Serial.print("[I2C] setPins SDA=idx"); Serial.print(LCD_SDA_IDX);
  Serial.print("(P0.31/D21) SCL=idx");   Serial.print(LCD_SCL_IDX);
  Serial.println("(P0.29/D20)");
 
  uint8_t found = 0;
  const uint8_t cand[] = { 0x27, 0x3F, 0x20, 0x21, 0x22, 0x23,
                           0x24, 0x25, 0x26, 0x38, 0x39, 0x3A,
                           0x3B, 0x3C, 0x3D, 0x3E };
  Serial.print("[I2C] Dang quet bus... ");
  for (uint8_t i = 0; i < sizeof(cand); i++) {
    Wire.beginTransmission(cand[i]);
    if (Wire.endTransmission() == 0) { found = cand[i]; break; }
  }
 
  if (found != 0) {
    Serial.print("thay LCD tai 0x"); Serial.println(found, HEX);
    lcd = new LiquidCrystal_I2C(found, LCD_COLS, LCD_ROWS);
    lcd->init();
    lcd->backlight();
    lcdOk = true;
 
    lcdStatus(String("Node '") + NODE_ID + "' dang khoi dong", "SX1262...");
  } else {
    Serial.println("KHONG thay LCD!");
    Serial.println("  -> Kiem tra: SDA=D21, SCL=D20, VCC=3.3V, GND chung");
    Serial.println("  -> Van chay binh thuong, chi khong co man hinh.");
  }
 
  ensureNfcPinsAsGpio();
 
initKeypad();
  Serial.println("[OK] Phim 5x4: Cot D6/D9/D19/D18");
  Serial.println("                Hang D15/D14/D8/D10/D0  (hang1 -> hang5)");
  Serial.print("                Hang 3 o "); Serial.println(ROW3_LABEL);
 
#if BUZZER_ENABLED
 
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerSilence();
  Serial.print("[OK] Buzzer: P0.10 (SuperMini D16), index ");
  Serial.print(BUZZER_PIN);
  Serial.print(", bao dong ");
  Serial.print(BUZZER_FREQ_ALARM);
  Serial.print(" Hz / het ");
  Serial.print(BUZZER_FREQ_CLEAR);
  Serial.println(" Hz");
 
  tone(BUZZER_PIN, 2000, 150);
  delay(250);
#else
  Serial.println("[--] Buzzer: TAT (BUZZER_ENABLED = 0)");
#endif
 
#if KEYPAD_SELFTEST
 
  testRowPinsElectrical();
  Serial.println();
 
  testColPinsElectrical();
  Serial.println();
 
  keypadSelfTest();
#endif
 
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, LOW);  delay(150);
  digitalWrite(LORA_RESET, HIGH); delay(300);
 
  pinMode(LORA_NSS, OUTPUT);
  digitalWrite(LORA_NSS, HIGH); delay(200);
  lorasSPI.begin();
 
  waitBusy(3000);
 
  Serial.println("[..] Dang khoi tao SX1262...");
  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                          LORA_SYNC, LORA_PWR, LORA_PRE, LORA_TCXO, LORA_LDO);
 
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("==========================================");
    Serial.print("[FAIL] SX1262 THAT BAI! Ma loi: "); Serial.println(state);
    Serial.println("  -706/-707 = SPI/TCXO -> kiem tra NSS/MOSI/MISO/SCK");
    Serial.println("  -2        = SPI khong thong -> kiem tra 3V3/GND");
    Serial.println("==========================================");
    if (lcdOk) lcdStatus("LOI SX1262", String("ma ") + state);
    while (true) {
      if (Serial) { Serial.print("[FAIL] SX1262 loi "); Serial.println(state); }
      delay(2000);
    }
  }
 
  radioReady = true;
  Serial.println("[OK] SX1262 KHOI TAO THANH CONG!");
  Serial.print("  Tan so : "); Serial.print(LORA_FREQ, 3); Serial.println(" MHz");
  Serial.print("  BW     : "); Serial.print(LORA_BW, 1);   Serial.println(" kHz");
  Serial.print("  SF     : "); Serial.println(LORA_SF);
  Serial.print("  CR     : 4/"); Serial.println(LORA_CR);
  Serial.print("  Sync   : 0x"); Serial.println(LORA_SYNC, HEX);
  Serial.print("  Power  : +"); Serial.print(LORA_PWR); Serial.println(" dBm");
  Serial.print("  Repeater: "); Serial.println(USE_REPEATER ? "CO" : "KHONG");
  Serial.print("  ACK timeout: "); Serial.print(ACK_TIMEOUT_MS); Serial.println(" ms");
  Serial.println("==========================================");
  Serial.println("[HD] Bam phim roi bam [E] de GUI");
  Serial.println("     [U] xoa 1 ky tu | [X] xoa het");
  Serial.println("     [D] chuyen man hinh: soan -> tin nhan -> thong so -> ...");
  Serial.println("     Bam phim bat ky khac [D] -> tro ve man hinh soan");
  Serial.println();
  Serial.println("     >>>  PHIM KHAN CAP  <<<");
  Serial.println("     [#] = BAO CHAY (SOS). Phat lap lai, den nen nhap nhay.");
  Serial.println("     [*] = HUY bao dong. Cac node khac cung tat canh bao.");
  Serial.println("     Trong luc bao dong chi nhan duoc [#] [*] [D].");
  Serial.println("     ('#' va '*' KHONG con dung de go chu trong tin nhan.)");
  Serial.println();
  Serial.println("     14 phim nhap lieu: 0-9 A B L R");
  Serial.println("     Hoac go tin trong Serial Monitor roi Enter");
  Serial.println("     Lenh Serial: 'sos' = bao dong, 'huy' = tat bao dong");
  Serial.println();
 
  if (lcdOk) {
 
    lcdStatus(String("Node '") + NODE_ID + "'", String(BUILD_TIME));
    delay(3000);
    lcdStatus(String("Node '") + NODE_ID + "' san sang", "RF OK  SF" + String(LORA_SF));
    delay(1500);
  }
 
  radio.setDio1Action(setFlag);
  if (enterRx()) Serial.println("[READY] Dang lang nghe...");
 
  needRedraw = true;
  redraw();
}
 
void loop() {
 
  if (alarmActive || alarmRxActive) {
 
    nrf_gpio_pin_clear(colPins[3]);
    delayMicroseconds(20);
    bool starPressed = (nrf_gpio_pin_read(rowPins[0]) == 0);
    nrf_gpio_pin_set(colPins[3]);
 
    nrf_gpio_pin_clear(colPins[2]);
    delayMicroseconds(20);
    bool hashPressed = (nrf_gpio_pin_read(rowPins[0]) == 0);
    nrf_gpio_pin_set(colPins[2]);
 
    static bool lastStar = false;
    static bool lastHash = false;
    if (starPressed && !lastStar && (millis() - lastDebounceTime > 300)) {
      lastDebounceTime = millis();
      handleKey('*');
    }
    lastStar = starPressed;
    if (hashPressed && !lastHash && (millis() - lastDebounceTime > 300)) {
      lastDebounceTime = millis();
      handleKey('#');
    }
    lastHash = hashPressed;
  }
 
  if (radioReady) handleIncoming();
 
alarmTick();
  alarmBlinkTick();
  buzzerTick();
 
  char key = scanKeypad();
  if (key != '\0' && (key != lastKey || (millis() - lastDebounceTime > 600))) {
    bool isRepeatHold = (key == lastKey);
 
    if (key != lastKey) {
      lastDebounceTime = millis();
    } else if (key == '*' || key == '#') {
 
      lastDebounceTime = millis();
    } else {
 
      lastKey = key;
    }
 
    if (!isRepeatHold || key == '*' || key == '#') {
      lastKey = key;
      handleKey(key);
    }
  }
  if (key == '\0') lastKey = '\0';
 
  if (Serial.available() > 0) {
 
    Serial.setTimeout(50);
    String m = Serial.readStringUntil('\n');
    m.trim();
    if (m.length() > MAX_MSG_LEN) m = m.substring(0, MAX_MSG_LEN);
    if (m.length() == 0) {
 
    } else if (m.equalsIgnoreCase("sos")) {
      if (alarmActive) {
        if (Serial) Serial.println("[SOS] Bao dong dang chay. Go 'huy' de tat.");
      } else {
        alarmStart();
      }
    } else if (m.equalsIgnoreCase("huy") || m.equalsIgnoreCase("clear")) {
      alarmCancel();
    } else if (alarmActive) {
 
      if (Serial) {
        Serial.println("[SOS] Dang bao dong - tu choi gui tin thuong.");
        Serial.println("      Go 'huy' de tat bao dong roi moi gui tin.");
      }
    } else {
      sendMessage(m);
    }
  }
 
  if (screenMode == SCREEN_MSG && msgNeedScroll && lcdOk && !transientOn) {
    if (millis() - msgScrollAt >= SCROLL_STEP_MS) {
      msgScrollAt = millis();
 
      uint8_t maxPos = (uint8_t)(MSG_SCROLL_PAD + lastMsg.length()
                                 - MSG_VISIBLE_RX);
 
      if (msgScrollPos < maxPos) {
        msgScrollPos++;
        needRedraw = true;
      }
    }
  }
 
  if (transientOn && lcdOk && (long)(millis() - transientUntil) > 0) {
    transientOn = false;
    needRedraw  = true;
  }
 
  if (needRedraw) redraw();
 
#if AUTO_BACK_MS != 0
  if (screenMode == SCREEN_MSG && !transientOn) {
    unsigned long backAfter = AUTO_BACK_MS;
    if (msgNeedScroll) {
unsigned long scrollMs =
        (unsigned long)(MSG_SCROLL_PAD + lastMsg.length()
                        - MSG_VISIBLE_RX) * SCROLL_STEP_MS;
      backAfter = scrollMs + 4000;
    }
    if (millis() - modeSwitchAt > backAfter) {
      screenMode = SCREEN_EDIT;
      needRedraw = true;
    }
  }
#endif
}
