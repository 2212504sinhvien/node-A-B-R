#include <RadioLib.h>
#include <SPI.h>
#include <nrf_gpio.h>
 
#define NODE_ROLE     ROLE_REPEATER
 
#define NODE_ID       'R'
 
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
 
#define MAX_HOP            2
 
#define CAD_WAIT_MS      2000
 
#define ACK_DELAY_MS      120
 
#define USE_REPEATER       1
#if USE_REPEATER
  #define ACK_TIMEOUT_MS   15000
#else
  #define ACK_TIMEOUT_MS    6000
#endif
 
#define MAX_RETRY          1
 
#define MAX_MSG_LEN        32
 
#define RF_HEADER_LEN      10
#define RF_PACKET_MAX      (RF_HEADER_LEN + MAX_MSG_LEN)
 
#define UART_RX_PIN  21
#define UART_TX_PIN  20
#define UART_BAUD    9600
#define UART_LINE_MAX 40
 
#define PROTO_TX_REQ   'T'
#define PROTO_MSG      'M'
#define PROTO_QUAL     'Q'
#define PROTO_ACKSTAT  'K'
#define PROTO_ERR      'E'
 
#define ROLE_ENDPOINT   0
#define ROLE_REPEATER   1
 
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
 
volatile bool receivedFlag = false;
void setFlag(void) { receivedFlag = true; }
 
bool     radioReady = false;
uint16_t mySeq      = 0;
 
uint32_t statTx = 0, statRx = 0, statAckOk = 0, statAckFail = 0;
uint32_t statRelay = 0, statDropHop = 0, statDropDup = 0;
 
char          ackSeenSrc   = '\0';
uint16_t      ackSeenSeq   = 0;
unsigned long ackSeenAt    = 0;
bool          ackSeenValid = false;
 
#define ACK_SEEN_WINDOW_MS   5000
 
#define ACK_AWARE_RELAY      0
 
uint32_t statSkipAcked = 0;
 
String uartLine = "";
 
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
 
bool sniffAckFor(char srcData, uint16_t seqData, uint32_t waitMs,
                 Pkt *outNewData) {
  uint32_t t0 = millis();
  uint32_t soAckNghe = 0;
 
  while (millis() - t0 < waitMs) {
    uint32_t irq = radio.getIrqFlags();
 
    if (!receivedFlag && !(irq & RADIOLIB_SX126X_IRQ_RX_DONE)) {
      delay(1);
      continue;
    }
    receivedFlag = false;
 
    Pkt q;
    if (!readRfPacket(q)) { enterRx(); continue; }
 
    if (q.type == TYPE_ACK) {
      soAckNghe++;
      ackSeenSrc   = q.dst;
      ackSeenSeq   = q.seq;
      ackSeenAt    = millis();
      ackSeenValid = true;
 
      if (Serial) {
        Serial.print("[R] Nghe ACK #"); Serial.print(soAckNghe);
        Serial.print(": "); Serial.print(q.src);
        Serial.print(" xac nhan seq="); Serial.print(q.seq);
        Serial.print(" dst="); Serial.print(q.dst);
        Serial.print(" (RSSI "); Serial.print(q.rssi, 1);
        Serial.println(" dBm)");
      }
 
      if (q.dst == srcData && q.seq == seqData) {
        enterRx();
        return true;
      }
    }
 
    else if (q.type == TYPE_DATA) {
 
      if (!seenBefore(q.src, q.seq, q.type)) {
        if (Serial) {
          Serial.print("[R] Trong luc sniff, nhan DATA moi: src=");
          Serial.print(q.src);
          Serial.print(" seq="); Serial.print(q.seq);
          Serial.println(" -> thoat sniff de xu ly ngay");
        }
        if (outNewData != NULL) {
          *outNewData = q;
        }
        enterRx();
        return false;
      }
    }
    else if (Serial) {
      Serial.print("[R] Nghe goi khac: type=");
      Serial.print(q.type);
      Serial.print(" src="); Serial.print(q.src);
      Serial.print(" seq="); Serial.println(q.seq);
    }
    enterRx();
  }
 
  if (Serial) {
    Serial.print("[R] Het ");
    Serial.print(waitMs);
    Serial.print(" ms, nghe duoc ");
    Serial.print(soAckNghe);
    Serial.println(" goi ACK");
  }
  return false;
}
 
int16_t txPacket(const String &raw) {
  String tmp = raw;
 
  waitForChannelFree(CAD_WAIT_MS);
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
 
#if NODE_ROLE == ROLE_ENDPOINT
void toNano(char tag, const String &body) {
  Serial1.print(tag);
  Serial1.println(body);
}
void toNanoKV(char tag, long a, long b) {
  Serial1.print(tag); Serial1.print(a); Serial1.print(';'); Serial1.println(b);
}
#endif
 
#if NODE_ROLE == ROLE_REPEATER
 
void handleRepeater(void) {
  uint32_t irq = radio.getIrqFlags();
  if (!receivedFlag && !(irq & RADIOLIB_SX126X_IRQ_RX_DONE)) return;
  receivedFlag = false;
 
  Pkt p;
  if (!readRfPacket(p)) { enterRx(); return; }
 
  if (p.hop >= MAX_HOP) {
    statDropHop++;
    if (Serial) {
      Serial.print("[R] Huy goi hop="); Serial.print(p.hop);
      Serial.println(" (dat gioi han MAX_HOP)");
    }
    enterRx();
    return;
  }
 
  if (seenBefore(p.src, p.seq, p.type)) {
    statDropDup++;
    enterRx();
    return;
  }
  rememberPacket(p.src, p.seq, p.type);
 
  if (p.dst == NODE_ID) {
    if (Serial) Serial.println("[R] Goi gui cho chinh repeater, bo qua");
    enterRx();
    return;
  }
 
#if ACK_AWARE_RELAY
  if (p.type == TYPE_DATA) {
 
    enterRx();
 
    unsigned long sniffStart = millis();
    Pkt pendingData;
    bool gotAck = sniffAckFor(p.src, p.seq, 1500, &pendingData);
 
    if (gotAck) {
      statSkipAcked++;
      if (Serial) {
        Serial.print("[R] BO QUA phat lai seq=");
        Serial.print(p.seq);
        Serial.print(" tu ");
        Serial.print(p.src);
        Serial.print(": node dich DA tra ACK sau ");
        Serial.print(millis() - sniffStart);
        Serial.println(" ms -> tin da den truc tiep, khong can relay");
      }
      enterRx();
      return;
    }
 
    if (Serial) {
      Serial.print("[R] Khong nghe ACK sau ");
      Serial.print(millis() - sniffStart);
      Serial.println(" ms -> phat lai vi node dich chua nhan duoc");
    }
  }
#endif
 
  statRelay++;
  if (Serial) {
    Serial.print("[R] Phat lai #"); Serial.print(statRelay);
    Serial.print("  src="); Serial.print(p.src);
    Serial.print(" dst="); Serial.print(p.dst);
    Serial.print(" type="); Serial.print(p.type);
    Serial.print(" hop="); Serial.print(p.hop);
    Serial.print(" -> ");   Serial.print(p.hop + 1);
    Serial.print("  seq="); Serial.print(p.seq);
    Serial.print("  RSSI="); Serial.print(p.rssi, 1);
    Serial.print("dBm SNR="); Serial.print(p.snr, 1); Serial.println("dB");
  }
 
  String fwd = String(p.src) + ',' + p.dst + ',' + p.type + ',';
  fwd += (unsigned)(p.hop + 1); fwd += ',';
  fwd += (unsigned)p.seq; fwd += ',';
  fwd += p.payload;
 
  int16_t st = txPacket(fwd);
  if (st != RADIOLIB_ERR_NONE && Serial) {
    Serial.print("[R] phat lai loi ma: "); Serial.println(st);
  }
}
 
#endif
 
#if NODE_ROLE == ROLE_ENDPOINT
 
void deliverToUser(const Pkt &p) {
  statRx++;
  if (Serial) {
    Serial.println("******************************************");
    Serial.print("[RX #"); Serial.print(statRx);
    Serial.print("] \""); Serial.print(p.payload); Serial.println("\"");
    Serial.print("  src="); Serial.print(p.src);
    Serial.print("  hop="); Serial.print(p.hop);
    Serial.print("  seq="); Serial.println(p.seq);
    Serial.print("  RSSI: "); Serial.print(p.rssi, 1); Serial.print(" dBm");
    Serial.print(" | SNR: "); Serial.print(p.snr, 1); Serial.println(" dB");
    Serial.println("******************************************");
  }
  toNano(PROTO_MSG, p.payload);
  toNanoKV(PROTO_QUAL, (long)p.rssi, (long)(p.snr * 10));
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
 
  if (seenBefore(p.src, p.seq, p.type)) { statDropDup++; return; }
  rememberPacket(p.src, p.seq, p.type);
 
  deliverToUser(p);
  sendAckFor(p);
}
 
void handleIncoming(void) {
  uint32_t irq = radio.getIrqFlags();
  if (!receivedFlag && !(irq & RADIOLIB_SX126X_IRQ_RX_DONE)) return;
  receivedFlag = false;
 
  Pkt p;
  if (!readRfPacket(p)) { enterRx(); return; }
 
  if (p.type == TYPE_DATA) {
    handleDataPacket(p);
  } else if (p.type == TYPE_ACK) {
 
    if (p.dst == NODE_ID && Serial) {
      Serial.print("[ACK] nhan xac nhan lang (seq=");
      Serial.print(p.seq); Serial.println(")");
    }
  }
  enterRx();
}
 
bool sendWithAck(const String &msg, uint32_t &rttMs) {
  for (uint8_t attempt = 1; attempt <= MAX_RETRY + 1; attempt++) {
 
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
        rttMs = millis() - t0;
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
    toNanoKV(PROTO_ACKSTAT, 0, 0);
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
  bool ok = sendWithAck(msg, rtt);
 
  if (ok) {
    statAckOk++;
    if (Serial) { Serial.print("-> [OK] ACK, RTT "); Serial.print(rtt); Serial.println(" ms"); }
  } else {
    statAckFail++;
    if (Serial) Serial.println("-> [FAIL] khong nhan duoc ACK");
  }
  if (Serial) {
    Serial.print("-> Thanh cong: "); Serial.print(statAckOk);
    Serial.print(" | That bai: "); Serial.println(statAckFail);
    Serial.println("----------------------------------------");
  }
 
  toNanoKV(PROTO_ACKSTAT, ok ? 1 : 0, (long)rtt);
  enterRx();
}
 
#endif
 
void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) delay(10);
  delay(200);
 
  Serial.println();
  Serial.println("==========================================");
#if NODE_ROLE == ROLE_REPEATER
  Serial.print("  LORA REPEATER  ID='"); Serial.print(NODE_ID); Serial.println("'");
#else
  Serial.print("  LORA ENDPOINT  ID='"); Serial.print(NODE_ID); Serial.println("'");
#endif
  Serial.println("  nRF52840 + SX1262  (v4)");
  Serial.println("==========================================");
 
  for (uint8_t i = 0; i < DEDUP_SLOTS; i++) dedup[i].used = false;
 
#if NODE_ROLE == ROLE_ENDPOINT
 
  Serial1.setPins(UART_RX_PIN, UART_TX_PIN);
  Serial1.begin(UART_BAUD);
 
  nrf_gpio_cfg_input(g_ADigitalPinMap[UART_RX_PIN], NRF_GPIO_PIN_PULLUP);
  Serial.println("[OK] UART toi Nano: RX=D21(P0.31) TX=D20(P0.29)");
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
#if NODE_ROLE == ROLE_ENDPOINT
    toNano(PROTO_ERR, String(state));
#endif
    while (true) {
      if (Serial) { Serial.print("[FAIL] SX1262 loi "); Serial.println(state); }
#if NODE_ROLE == ROLE_ENDPOINT
      toNano(PROTO_ERR, String(state));
#endif
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
  Serial.print("  MAX_HOP: "); Serial.println(MAX_HOP);
  Serial.print("  ACK timeout: "); Serial.print(ACK_TIMEOUT_MS); Serial.println(" ms");
  Serial.println("==========================================");
 
  radio.setDio1Action(setFlag);
  if (enterRx()) {
#if NODE_ROLE == ROLE_REPEATER
    Serial.println("[READY] Repeater dang lang nghe de phat lai...");
#else
    Serial.println("[READY] Endpoint dang lang nghe...");
#endif
  }
  Serial.println();
}
 
void loop() {
  if (!radioReady) return;
 
#if NODE_ROLE == ROLE_REPEATER
 
  handleRepeater();
 
  static uint32_t lastStat = 0;
  if (millis() - lastStat >= 30000) {
    lastStat = millis();
    if (Serial) {
      Serial.println("----------------------------------------");
      Serial.print("[R] THONG KE  Da phat lai: "); Serial.print(statRelay);
      Serial.print(" | Bo qua vi co ACK: "); Serial.print(statSkipAcked);
      Serial.print(" | Huy do hop: "); Serial.print(statDropHop);
      Serial.print(" | Huy do trung: "); Serial.println(statDropDup);
 
      if (statSkipAcked > 0) {
        Serial.println("     -> 'Bo qua vi co ACK' > 0 nghia la cac node");
        Serial.println("        dang o GAN NHAU, tin den dich truc tiep.");
        Serial.println("        Repeater KHONG can phat lai -> tot.");
      }
      if (statRelay > 0 && statSkipAcked == 0) {
        Serial.println("     -> Chua lan nao bo qua => cac node o XA nhau,");
        Serial.println("        repeater dang lam dung viec phat lai.");
      }
      Serial.println("----------------------------------------");
    }
  }
 
#else
 
  handleIncoming();
 
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
 
    if (c == '\n' || c == '\r') {
      if (uartLine.length() > 0) {
        String line = uartLine;
        uartLine = "";
 
        char tag = line.charAt(0);
        if (tag == PROTO_TX_REQ) {
          String body = line.substring(1);
          if (body.length() > MAX_MSG_LEN) {
            if (Serial) {
              Serial.print("[!] Tin "); Serial.print(body.length());
              Serial.print(" ky tu > gioi han "); Serial.print(MAX_MSG_LEN);
              Serial.println(", da cat ngan");
            }
            body = body.substring(0, MAX_MSG_LEN);
          }
          if (body.length() > 0) sendMessage(body);
        }
      }
      continue;
    }
 
    if (uartLine.length() == 0) {
      if (c != PROTO_TX_REQ && c != PROTO_MSG && c != PROTO_QUAL &&
          c != PROTO_ACKSTAT && c != PROTO_ERR) continue;
    }
 
    if (uartLine.length() < UART_LINE_MAX) uartLine += c;
 
  }
 
  if (Serial.available() > 0) {
    String m = Serial.readStringUntil('\n');
    m.trim();
    if (m.length() > MAX_MSG_LEN) m = m.substring(0, MAX_MSG_LEN);
    if (m.length() > 0) sendMessage(m);
  }
 
#endif
}
