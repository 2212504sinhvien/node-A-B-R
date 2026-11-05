/*
 * =====================================================================
 *  LORA NODE A - nRF52840 SuperMini + SX1262 + LCD 1602 I2C + Phim 5x4
 *  BO FIRMWARE CHUAN - SF12 - 3 NODE (A, B, R)
 * =====================================================================
 *
 *  THU MUC: D:\lora_backup\loraAB\
 *    lora_node_A\lora_node_A.ino   <- FILE NAY (NODE_ID = 'A')
 *    lora_node_B\lora_node_B.ino   (NODE_ID = 'B')
 *    lora_node_R\lora_node_R.ino   (NODE_ID = 'R', repeater)
 *
 *  !!! CA 3 NODE PHAI CO CUNG THONG SO RF !!!
 *    LORA_FREQ = 915.0   LORA_BW = 125.0   LORA_SF = 12
 *    LORA_CR   = 5       LORA_SYNC = 0x12  LORA_PWR = 22
 *    Lech BAT KY thong so nao = KHONG BAO GIO nhan duoc nhau.
 *    (Loi nay da tung lam hong toan bo he thong: TX phat SF12, RX thu SF9.)
 *
 *  Moi thiet bi gom 1 board nRF52840 duy nhat dam nhan tat ca:
 *    - SX1262 : thu phat LoRa
 *    - LCD I2C: hien thi
 *    - Phim 5x4: nhap lieu
 *    - Buzzer  : coi bao dong + bao ket qua gui tin
 *
 *  NGAN SACH CHAN: 18/18 GPIO da dung
 *    SX1262   : 6 chan
 *    LCD I2C  : 2 chan
 *    Phim 5x4 : 9 chan
 *    Buzzer   : 1 chan
 *
 * ---------------------------------------------------------------------
 *  SO DO NOI DAY
 * ---------------------------------------------------------------------
 *
 *                         [ CONG USB-C ]
 *                   +------------------------+
 *   Phim Hang5  D0  | . P0.08        RAW .   |  (KHONG dung)
 *   LoRa RST    D1  | . P0.06         B+ .   |
 *   LoRa MISO   D2  | . P0.17        GND .   |  LCD GND + LoRa GND + Buzzer GND
 *   LoRa MOSI   D3  | . P0.20        RST .   |
 *   LoRa SCK    D4  | . P0.22       3.3V .   |  LCD VCC (3.3V!) + LoRa VCC
 *   LoRa NSS    D5  | . P0.24      P0.31 . D21|  LCD SDA
 *   Phim Cot1   D6  | . P1.00      P0.29 . D20|  LCD SCL
 *   LoRa DIO1   D7  | . P0.11      P0.02 . D19|  Phim Cot3
 *   Phim Hang3  D8  | . P1.04      P1.15 . D18|  Phim Cot4
 *   Phim Cot2   D9  | . P1.06      P1.13 . D15|  Phim Hang1
 *                   |                P1.11 . D14|  Phim Hang2
 *                   |                P0.10 . D16|  BUZZER (coi)
 *                   |                P0.09 . D10|  Phim Hang4
 *                   +------------------------+
 *
 *  BAN PHIM MA TRAN 5x4:
 *    Cot : Cot1=D6  Cot2=D9  Cot3=D19  Cot4=D18
 *    Hang: Hang1=D15  Hang2=D14  Hang3=D8  Hang4=D10  Hang5=D0
 *
 *  SX1262 Ra-01SH:
 *    3V3 -> 3.3V   GND -> GND   NSS -> D5   MOSI -> D3
 *    MISO -> D2    SCK -> D4    RST -> D1   DIO1 -> D7
 *    BUSY: KHONG CAN NOI (USE_BUSY_PIN = 0)
 *
 *  LCD 1602 I2C:
 *    VCC -> 3.3V   GND -> GND   SDA -> D21   SCL -> D20
 *
 *  BUZZER (coi):
 *    Chan tin hieu -> D16 (P0.10)   Chan con lai -> GND
 *    Loai coi dien tu 16 ohm, chay truc tiep 3.3V, KHONG can mach khuech dai.
 *
 * ---------------------------------------------------------------------
 *  !!! CANH BAO NGUON LCD - DOC KY TRUOC KHI CAM !!!
 * ---------------------------------------------------------------------
 *  KHONG cam LCD VCC vao RAW/5V. Voi nRF52840 dieu do SAI va co the
 *  lam hong chip.
 *
 *  Ly do: module LCD I2C dung chip PCF8574, tren do co 2 dien tro keo len
 *  (pull-up) tu SDA/SCL len VCC. Neu VCC = 5V thi 2 day tin hieu se bi
 *  keo len 5V. nRF52840 KHONG chiu duoc 5V (gioi han tuyet doi 3.6V)
 *  -> qua ap, co the chay chan hoac hong chip.
 *
 *  Giai phap: cap LCD bang 3.3V. An toan tuyet doi.
 *
 * ---------------------------------------------------------------------
 *  !!! CANH BAO NGUON PIN - QUAN TRONG KHI CHAY BANG PIN !!!
 * ---------------------------------------------------------------------
 *  Khi PHAT o +22 dBm, SX1262 keo dong dinh ~120 mA.
 *  Cong them nRF52 (~10mA) + LCD den nen (~20mA) = tong ~150 mA dinh.
 *
 *  Neu nguon pin khong cap du 150 mA -> SUT AP khi phat -> chip reset
 *  -> khong gui duoc (nhung log Serial van bao da phat).
 *
 *  Neu gap hien tuong "cam USB thi gui duoc, dung pin thi khong":
 *    (1) Them tu 220uF + 100nF sat chan VCC cua SX1262
 *    (2) Hoac giam LORA_PWR xuong 17 (dong giam con ~92 mA)
 *    (3) Kiem tra pin da sac day (>= 4.0V) va day nguon du to
 *
 * ---------------------------------------------------------------------
 *  VI SAO LCD BAT BUOC PHAI O D21/D20
 * ---------------------------------------------------------------------
 *  Wire.setPins() dich so chan qua mang g_ADigitalPinMap[] cua variant
 *  Feather. Mang do KHONG co index nao tro toi P1.00 (D6), P1.06 (D9),
 *  P1.04 (D8), P1.11 (D14), P1.13 (D15).
 *    -> LCD (dung Wire) chi dat duoc o D21 (idx21 = P0.31) va
 *       D20 (idx20 = P0.29).
 *    -> Ban phim dung nrf_gpio truc tiep (so port/pin tho) nen dat o dau
 *       cung duoc, ke ca D6/D9 khong co trong mang.
 *
 *  !!! BAY NGUY HIEM: Wire mac dinh dung PIN_WIRE_SCL = index 23 = P0.11,
 *  trung KHOP chan DIO1 cua LoRa. Neu goi Wire.begin() ma khong goi
 *  Wire.setPins(21, 20) TRUOC, TWIM0 se ghi de P0.11 thanh SCL va
 *  NGAT NHAN TIN CHET NGAY. Code duoi day goi setPins() dau tien.
 *
 * ---------------------------------------------------------------------
 *  GIAO THUC RF (tuong thich voi lora_node_R.ino)
 * ---------------------------------------------------------------------
 *    <src>,<dst>,<type>,<hop>,<seq>,<payload>
 *    src  : ky tu node tao goi ('A','B','R'...)
 *    dst  : '*' broadcast hoac ky tu node nhan
 *    type : 'D' du lieu | 'K' ACK | 'F' bao chay | 'C' huy bao dong
 *    hop  : so lan da phat lai (0 = goi goc)
 *    seq  : so thu tu, TANG MOI LAN THU (khong phai moi tin)
 *
 * ---------------------------------------------------------------------
 *  PHIM CHUC NANG
 * ---------------------------------------------------------------------
 *    Cot 4 (ngoai cung phai) = 4 phim chuc nang: U / D / X / E
 *    Hang 1 cot 3 va cot 4   = 2 phim khan cap: '#' bao chay, '*' huy
 *    14 phim con lai          = nhap chu (0-9 A B L R)
 *
 *    !!! '#' va '*' KHONG con la phim nhap lieu !!!
 * =====================================================================
 */

#include <RadioLib.h>
#include <SPI.h>
#include <nrf_gpio.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =====================================================================
//  >>> CAU HINH NODE - DOI O DAY <<<
// =====================================================================
#define NODE_ID       'A'      // NODE A. KHONG DUOC TRUNG voi cac node khac.

// ---------------------------------------------------------------------
//  DAU VAN TAY PHIEN BAN - dung de biet board dang chay ban nao
// ---------------------------------------------------------------------
// Van de that su da xay ra: 2 file giong nhau tung byte (chi khac NODE_ID),
// nhung mot board chay duoc ban phim con board kia thi khong. Ve mat logic
// dieu do KHONG THE xay ra -> ket luan: board do dang chay FIRMWARE CU,
// chua duoc nap lai.
//
// __DATE__ va __TIME__ la macro chuan cua C, tu dong lay thoi diem BIEN DICH.
// Khong the lam gia, khong can nho cap nhat thu cong.
// Neu 2 board in ra 2 thoi diem khac nhau -> chung dang chay 2 ban khac nhau.
#define BUILD_DATE  __DATE__
#define BUILD_TIME  __TIME__

// 1 = duong truyen co repeater, 0 = 2 node noi thang nhau
// >>> PHAI DOI THANH 1 KHI DUNG REPEATER <<<
// Tinh toan da chung minh: qua repeater, 1 goi di-ve mat ~5.3 giay
// (A phat 1.3s -> R relay 1.3s -> B ACK 1.2s -> R relay ACK 1.2s).
// Neu de = 0: chu ky lap bao dong 4000 ms < 5300 ms
//   -> A phat goi lan 2 DE LEN CHINH ACK dang ve -> va cham lien tuc.
// Nen khi co repeater: chu ky lap = 8000 ms, ACK timeout = 10000 ms.
#define USE_REPEATER  1

// 1 = vao che do CHAN DOAN BAN PHIM luc khoi dong (chi dung khi test,
//     khong gui/nhan LoRa). 0 = chay binh thuong.
// >>> DA DAT = 0: ban phim node A da duoc test thanh cong TAT CA cac nut. <<<
// Dat lai = 1 neu can kiem tra lai ban phim (vi du khi sua node B,
// hoac sau khi sua day noi).
#define KEYPAD_SELFTEST 0

// =====================================================================
//  THONG SO RF - PHAI GIONG NHAU 100% TREN MOI NODE
// =====================================================================
#define LORA_FREQ   915.0
#define LORA_BW     125.0
#define LORA_SF     12       // <<< 2 NODE PHAI TRUNG NHAU. Sai = khong nhan duoc
#define LORA_CR     5
#define LORA_SYNC   0x12
#define LORA_PWR    22
#define LORA_PRE    8
#define LORA_TCXO   0.0      // 0.0 = XTAL (Ra-01SH). KHONG doi thanh 1.6
#define LORA_LDO    true

// =====================================================================
//  CHAN LORA - so index kieu Feather (khong phai nhan SuperMini)
// =====================================================================
#define LORA_NSS    1    // SuperMini D5  -> P0.24
#define LORA_DIO1   23   // SuperMini D7  -> P0.11   (trung Wire SCL mac dinh!)
#define LORA_RESET  11   // SuperMini D1  -> P0.06
#define LORA_SCLK   30   // SuperMini D4  -> P0.22
#define LORA_MISO   29   // SuperMini D2  -> P0.17
#define LORA_MOSI   28   // SuperMini D3  -> P0.20

#define BUSY_GPIO   NRF_GPIO_PIN_MAP(1, 4)   // SuperMini D8 -> P1.04
#define USE_BUSY_PIN     0    // 0 = khong dung BUSY (da test OK, khoi can noi day)
#define BUSY_TIMEOUT_MS  2000

// =====================================================================
//  CHAN LCD I2C - so index kieu Feather
// =====================================================================
#define LCD_SDA_IDX  21   // SuperMini D21 -> P0.31
#define LCD_SCL_IDX  20   // SuperMini D20 -> P0.29
#define LCD_COLS     16
#define LCD_ROWS     2

// =====================================================================
//  CHAN BAN PHIM - GPIO tho (dung nrf_gpio, khong qua variant map)
// =====================================================================
// Cot 1 va Cot 2 DA DOI SANG D6/D9 de nhuong D21/D20 cho LCD.
// 7 chan con lai giu nguyen nhu ban da chay thanh cong.
const uint8_t ROWS = 5, COLS = 4;

const uint32_t colPins[COLS] = {
  NRF_GPIO_PIN_MAP(1,  0),   // Cot 1 -> SuperMini D6  (P1.00)  [DA DOI tu D21]
  NRF_GPIO_PIN_MAP(1,  6),   // Cot 2 -> SuperMini D9  (P1.06)  [DA DOI tu D20]
  NRF_GPIO_PIN_MAP(0,  2),   // Cot 3 -> SuperMini D19 (P0.02)
  NRF_GPIO_PIN_MAP(1, 15)    // Cot 4 -> SuperMini D18 (P1.15)
};

// ---------------------------------------------------------------------
//  CHON CHAN CHO HANG 3
// ---------------------------------------------------------------------
// Hang 3 ban dau dat o D16 (P0.10 = NFC2). Test thuc te cho thay
// chi hang nay chet, 4 hang con lai deu tot.
//
//   ROW3_ALT_PIN = 0 : hang 3 o D16 (P0.10, NFC2)  - DA LOAI BO
//   ROW3_ALT_PIN = 1 : hang 3 o D8  (P1.04)        - MAC DINH hien tai
//
// >>> DA DAT = 1 vi ket qua test thuc te cho thay hang 3 chet. <<<
//
// Suy luan loai tru NFC (da chung minh, khong phai doan):
//   Hang 3 = P0.10 (NFC2), hang 4 = P0.09 (NFC1).
//   Ca HAI bi khoa boi CUNG mot bit UICR->NFCPINS, khong the khoa
//   rieng tung chan.
//   Test thuc te: hang 4 HOAT DONG, chi hang 3 chet.
//   => Bit NFC dang TAT, nen P0.10 KHONG bi khoa NFC.
//   => Nguyen nhan la chan D16/P0.10 hoac day noi toi no, KHONG phai NFC.
//
// D8 (P1.04) la chan GPIO thuong, khong lien quan NFC, va dang trong
// (vi USE_BUSY_PIN = 0).
//
// >>> VIEC CAN LAM VE PHAN CUNG: doi day hang 3 tu D16 sang D8 <<<
#define ROW3_ALT_PIN  1

#if ROW3_ALT_PIN
  #define ROW3_GPIO   NRF_GPIO_PIN_MAP(1,  4)   // D8  / P1.04
  #define ROW3_LABEL  "D8/P1.04"
#else
  #define ROW3_GPIO   NRF_GPIO_PIN_MAP(0, 10)   // D16 / P0.10 (NFC2)
  #define ROW3_LABEL  "D16/P0.10"
#endif

// Chan D8 (P1.04) chi duoc phep phuc vu MOT muc dich.
// Neu ca hai co cung bat, ban phim va LoRa se tranh nhau chan nay
// -> hang 3 bam ra ket qua lung tung, rat kho truy. Chan ngay tu luc dich.
#if ROW3_ALT_PIN && USE_BUSY_PIN
  #error "XUNG DOT CHAN D8: ROW3_ALT_PIN va USE_BUSY_PIN khong the cung bat. Hay dat USE_BUSY_PIN = 0."
#endif

// !!! THU TU HANG DA DUOC DAO LAI theo ket qua test thuc te !!!
// Test tren mach A: bam phim [F2] (hang 1) ra '0', bam [2] (hang 2) ra '8'.
//   '0' = keyMap[4][1] (hang 5)   ->  hang vat ly 1 bi doc thanh hang 5
//   '8' = keyMap[3][1] (hang 4)   ->  hang vat ly 2 bi doc thanh hang 4
// Quy luat: hang vat ly R bi map thanh (6 - R) = THUA MANG rowPins NGUOC.
// Cot van dung (ca 2 lan deu ra cot 2) nen colPins khong doi.
// => Day hang vat ly 1 nam o D15 (P1.13), khong phai D0 nhu tai lieu cu ghi.
// NEU ban cam lai dau connector ban phim theo chieu nguoc lai,
// chi can dao nguoc 5 dong duoi day.
const uint32_t rowPins[ROWS] = {
  NRF_GPIO_PIN_MAP(1, 13),   // Hang 1 -> SuperMini D15 (P1.13)
  NRF_GPIO_PIN_MAP(1, 11),   // Hang 2 -> SuperMini D14 (P1.11)
  ROW3_GPIO,                 // Hang 3 -> D16 (P0.10) hoac D8 (P1.04)
  NRF_GPIO_PIN_MAP(0,  9),   // Hang 4 -> SuperMini D10 (P0.09)  NFC1
  NRF_GPIO_PIN_MAP(0,  8)    // Hang 5 -> SuperMini D0  (P0.08)
};

// QUY UOC PHIM - nhin ban phim se thay ngay:
//   COT 4 (ben phai nhat) = 4 phim chuc nang: U / D / X / E
//   HANG 1 cot 3 va cot 4 = phim KHAN CAP: # = BAO CHAY, * = HUY
//   14 phim con lai       = nhap chu (0-9 A B L R)
//
//   !!! '#' va '*' KHONG con la phim nhap lieu !!!
//   Gia phai tra: khong go duoc 2 ky tu do trong tin nhan.
//   Doi lai co nut khan cap rieng, khong can them chan phan cung nao.
const char keyMap[ROWS][COLS] = {
  { 'A', 'B', '#', '*' },   // Hang 1 : nhap A, B | '#'=BAO CHAY  '*'=HUY
  { '1', '2', '3', 'U' },   // Hang 2 : nhap 1 2 3 | U = xoa lui 1 ky tu
  { '4', '5', '6', 'D' },   // Hang 3 : nhap 4 5 6 | D = doi man hinh
  { '7', '8', '9', 'X' },   // Hang 4 : nhap 7 8 9 | X = xoa het
  { 'L', '0', 'R', 'E' }    // Hang 5 : nhap L 0 R | E = ENTER / GUI
};

// =====================================================================
//  THOI GIAN - tinh theo on-air time that o SF12/BW125
// =====================================================================
// On-air (doi chieu log thuc te cua ban):
//    goi ACK  ~11 byte -> ~ 991 ms
//    goi DATA ~33 byte -> ~1647 ms
//    goi SOS  ~20 byte -> ~1319 ms
//
// --- KHONG repeater (A va B noi thang) ---
//   phat DATA 1647 + cho ACK_DELAY 120 + ACK ve 991 = ~2758 ms
//   Thuc do: RTT ~3200 ms
//
// --- CO repeater (A-B khong nghe nhau truc tiep) ---
//   1. A phat DATA                       1647 ms
//   2. R cho CAD + phat lai DATA         2000+1647 = 3647 ms
//   3. B cho ACK_DELAY + tra ACK          120+ 991 = 1111 ms
//   4. R cho CAD + phat lai ACK          2000+ 991 = 2991 ms
//   ---------------------------------------------------
//   TONG RTT toi thieu                             9396 ms
//
// !!! DA SUA LOI NGHIEM TRONG (toi uu cho > 1km co repeater) !!!
// Truoc day: ACK_TIMEOUT_MS = 10000 ms trong khi RTT can 11396 ms
//   (vi CAD cua R la 3000 ms) -> A HET TIMEOUT TRUOC khi ACK ve
//   -> A tuong that bai -> retry -> ty le thanh cong chi 1/2.
//
// Da sua 2 cho:
//   (1) Giam CAD_WAIT_MS cua repeater tu 3000 -> 2000 ms
//       (xem lora_node_R.ino) -> RTT giam con 9396 ms
//   (2) Tang ACK_TIMEOUT_MS tu 10000 -> 15000 ms
//       -> du phong 1.6 lan so voi RTT toi thieu
// ---------------------------------------------------------------------
#if USE_REPEATER
  #define ACK_TIMEOUT_MS   15000
  #define CAD_WAIT_MS       3000
#else
  #define ACK_TIMEOUT_MS    6000
  #define CAD_WAIT_MS        800   // 2 node noi thang, it nguy co va cham
#endif

// ---------------------------------------------------------------------
//  CAD KHI DANG CO BAO DONG - "Nhuong duong cho tin khan cap"
// ---------------------------------------------------------------------
// Goi bao dong 20 byte  = ~1319 ms tren khong khi.
// Goi tin dai nhat 42 byte = ~2138 ms.
//
// CAD_WAIT_MS = 800 KHONG du phu trum bat ky goi nao trong 2 goi do:
// neu node B bat dau cho kenh luc node A moi phat duoc nua goi bao dong
// (con ~660 ms nua), B het kiem nhan sau 800 ms roi PHAT CHONG LEN
// -> ca 2 goi vo.
//
// Giai phap: khi tren mang co bao dong (minh phat hoac nghe duoc),
// tang kien nhan len 3000 ms - dai hon goi dai nhat (2138 ms)
// => B chac chan cho duoc luc ben kia noi xong.
// Chi phi: tin thuong cham toi da ~3 giay, CHI khi co bao dong.
// Binh thuong (khong bao dong) giu nguyen 800 ms cho nhanh.
#define CAD_WAIT_ALARM_MS  3000

// ---------------------------------------------------------------------
//  SO LAN THU LAI KHI KHONG NHAN DUOC ACK
// ---------------------------------------------------------------------
// !!! DA TANG TU 1 LEN 2 (giam mat goi khi co repeater, khoang cach xa) !!!
//
// MAX_RETRY = 1 -> thu toi da 2 luot  (ban cu)
// MAX_RETRY = 2 -> thu toi da 3 luot  (ban nay)
//
// Ly do: khi A-B o xa nhau (>1km) va phai di qua repeater, RTT len tới
// ~9400 ms. Chi can 1 goi bi nhieu hoac va cham la mat -> phai retry.
// Voi 3 luot thu, kha nang ca 3 deu that bai la rat thap.
//
// Chi phi: khi that bai hoan toan, thoi gian cho tang tu ~30s len ~45s.
// Doi lai: ty le thanh cong cao hon ro ret.
#define MAX_RETRY          2
#define MAX_HOP            2

#define ACK_DELAY_MS     120       // cho ben kia kip quay ve RX truoc khi tra ACK

#define MAX_MSG_LEN       32       // tang so nay se keo dai on-air time
#define RF_HEADER_LEN     10       // "A,B,D,0,999," = 10 ky tu

// =====================================================================
//  KIEU GOI TIN
// =====================================================================
#define TYPE_DATA       'D'   // tin nhan thuong
#define TYPE_ACK        'K'   // xac nhan
#define TYPE_ALARM      'F'   // BAO CHAY / KHAN CAP (Fire) - uu tien cao nhat
#define TYPE_CLEAR      'C'   // huy bao dong (Clear)
#define DST_BROADCAST   '*'

// Loc loai goi hop le. Dung o readRfPacket() de loai goi rac.
//
// !!! QUAN TRONG !!!
// Moi khi them loai goi moi, PHAI them vao day. Bo loc cu viet cung:
//     if (p.type != TYPE_DATA && p.type != TYPE_ACK) return false;
// Neu quen, goi moi se bi VUT IM LANG (khong co log nao), va trong he
// thong bao chay thi do la loi nghiem trong nhat.
//
// !!! VI SAO DUNG #define MA KHONG DUNG HAM !!!
// Arduino tu sinh prototype cho moi ham va chen chung ngay TRUOC
// dinh nghia ham dau tien trong file. Neu viet ham o day (truoc struct Pkt)
// thi tat ca prototype bi chen truoc khi Pkt ton tai -> bao loi
// "'Pkt' was not declared in this scope" va "redeclared as different
// kind of entity". Dung macro de tranh hoan toan van de nay.
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

// =====================================================================
//  DOI TUONG TOAN CUC
// =====================================================================
SPIClass lorasSPI(NRF_SPIM2, LORA_MISO, LORA_SCLK, LORA_MOSI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RESET, RADIOLIB_NC, lorasSPI);

// LCD: dia chi tuy module (0x27 hoac 0x3F) nen phai quet luc chay.
// Constructor cua LiquidCrystal_I2C KHONG goi Wire.begin() (da verify),
// chi init() moi goi -> ta kip setPins() truoc do.
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
uint16_t lastSeq   = 0;   // seq cua tin nhan cuoi, hien o man hinh thong so
uint8_t  lastHop   = 0;   // so lan da bi phat lai (0 = truc tiep)

// ---------------------------------------------------------------------
//  MAN HINH TIN NHAN: GIU MAI MAI, KHONG TU TAT
// ---------------------------------------------------------------------
// Code cu tu dong quay ve man hinh soan sau AUTO_BACK_MS (8s) hoac
// sau khi cuon xong + 4s. Ban muon doc tin va so lieu lau hon nen
// da bo che do tu tat. Gio tin nhan O LAI cho den khi ban bam phim.
// Bam bat ky phim nhap lieu nao -> ve man hinh soan.
// Bam 'D' -> chuyen qua lai giua tin nhan va thong so chi tiet.
// Dat 0 = khong bao gio tu quay ve man hinh soan.
#define AUTO_BACK_MS 0

// Man hinh: 0 = soan tin, 1 = tin nhan cuoi, 2 = thong so, 3 = lien ket
#define SCREEN_EDIT    0
#define SCREEN_MSG     1
#define SCREEN_STATS   2   // thong so song + bo dem TX/RX/OK/F
#define SCREEN_ALARM   3   // bao dong khan cap - uu tien cao nhat
#define SCREEN_LINK    4   // hop / seq / RTT - chan doan repeater

// ---------------------------------------------------------------------
//  SO KY TU TIN NHAN HIEN DUOC CUNG LUC (tin chiem CA 2 DONG)
// ---------------------------------------------------------------------
//  MAN SOAN TIN - tien to ngan "B:" (2 cot):
//    Dong 0: "B:" + 14 ky tu tin = 16 cot
//    Dong 1: 16 ky tu tin
//    => 30 ky tu tin hien cung luc
//
//  MAN NHAN TIN - tien to ro nghia "TU A:" (5 cot) theo yeu cau:
//    Dong 0: "TU A:" + 11 ky tu tin = 16 cot
//    Dong 1: 16 ky tu tin
//    => 27 ky tu tin hien cung luc
//
//  VI SAO 2 MAN KHAC NHAU: man soan tin khong can noi "TU" (nguoi dung
//  biet minh dang go), nen de tien to ngan de co nhieu cho hon.
//  Man nhan tin can ro nguon gui -> dung 5 cot, chap nhan it hon 3 cot.
#define MSG_LINE0_CHARS  (LCD_COLS - 2)   // 14 cho man SOAN
#define MSG_L0_RX_CHARS  (LCD_COLS - 5)   // 11 cho man NHAN ("TU A:")
#define MSG_VISIBLE_RX   (MSG_L0_RX_CHARS + LCD_COLS)   // 27

// Man SOAN: tong ky tu tin hien duoc
#define MSG_VISIBLE      (MSG_LINE0_CHARS + LCD_COLS)   // 30

// So khoang trang dem truoc tin khi cuon: de tin "chay vao" tu phai sang
// trai thay vi hien ngay tu ky tu dau (trong tu nhien hon).
#define MSG_SCROLL_PAD   3

// Suc chua thuc te khi KHONG cuon (da mo phong de tranh mat ky tu cuoi):
//   Man SOAN: 30 - 3 = 27
//   Man NHAN: 27 - 3 = 24
#define MSG_CAPACITY     (MSG_VISIBLE - MSG_SCROLL_PAD)        // 27
#define MSG_CAPACITY_RX  (MSG_VISIBLE_RX - MSG_SCROLL_PAD)     // 24

// ---------------------------------------------------------------------
//  CUON TIN NHAN DAI TREN LCD 16x2
// ---------------------------------------------------------------------
// Tin nhan toi da MAX_MSG_LEN (32) ky tu nhung LCD chi hien 16 cot.
// Code cu chi cat lay 16 ky tu dau -> phan sau bi mat vinh vien,
// ban khong the doc het tin khi di test ngoai duong.
// Vay khi tin dai hon LCD_COLS, tu dong cuon trai->phai tung buoc.
bool msgNeedScroll = false;   // true khi lastMsg dai hon 16 cot
uint8_t msgScrollPos = 0;     // vi tri cat hien tai trong chuoi da dem
unsigned long msgScrollAt = 0;
#define SCROLL_STEP_MS   400  // toc do cuon, moi 400ms dich 1 ky tu

// ---------------------------------------------------------------------
//  MAN HINH TAM THOI (transient)
// ---------------------------------------------------------------------
// Dung de bao ket qua ngan han: OK da gui + RTT, FAIL, loi...
//
// Vi sao can: code cu goi lcdStatus(...) roi ngay sau do dat
// needRedraw = true. loop() chay redraw() va ghi de man hinh LUON,
// nen ban khong bao gio kip thay "OK da gui" va chi so RTT.
// Day cung chinh la nguyen nhan lam mat RSSI tren LCD truoc day.
//
// Quy tac duy nhat: MUON HIEN GI tren LCD thi chi cap nhat bien,
// roi dat needRedraw = true. CHI co redraw() duoc phep ve.
// Khong duoc goi lcdLine()/lcdStatus() truc tiep tu noi khac.
bool          transientOn    = false;
String        transientL0    = "";
String        transientL1    = "";
unsigned long transientUntil = 0;

// ---------------------------------------------------------------------
//  BAO DONG KHAN CAP (SOS) - phim '#' kich hoat, phim '*' huy
// ---------------------------------------------------------------------
// NGUYEN TAC THIET KE QUAN TRONG NHAT: KHONG BLOCKING.
//
// sendWithAck() cua tin thuong chan loop() toi ~18 giay (2 luot x
// [CAD + phat 2.1s + cho ACK 6s]). Neu bao dong dung kieu do va lap
// 10 lan thi loop() bi chan ~90 giay, trong luc do:
//   - khong quet duoc ban phim  -> KHONG THE HUY bao dong neu bam nham
//   - khong nhaps nhay duoc LCD
//   - khong nhan duoc tin phan hoi
// Vi vay bao dong phai phat theo KIEU TICH (tick): moi luot loop() chi
// phat 1 goi roi tra quyen ngay, dung millis() de canh lan phat ke tiep.
//
// KHONG dung delay() o bat ky dau trong luong bao dong.

// Chu ky phat lai khi CHUA duoc node nao xac nhan.
// PHAI lon hon on-air time (2.1s goi dai nhat) de khong de chong len
// chinh goi truoc cua minh, va (neu co repeater) de R kip phat lai.
#if USE_REPEATER
  #define ALARM_REPEAT_MS        8000   // chua co ACK: lap moi 8 giay
  #define ALARM_REPEAT_ACKED_MS  30000  // da co ACK: lap cham lai de giu song
#else
  #define ALARM_REPEAT_MS        4000
  #define ALARM_REPEAT_ACKED_MS  20000
#endif

// Toc do nhaps nhay den nen LCD (ms). Dung den nen vi chi ton ~1ms I2C,
// khong anh huong den thoi gian danh cho RF.
#define ALARM_BLINK_MS  400

// ---------------------------------------------------------------------
//  BUZZER (coi bao dong) - chan P0.10
// ---------------------------------------------------------------------
//  !!! CANH BAO VE SO CHAN - DOC KY !!!
//  SuperMini silkscreen "D16" = GPIO P0.10 = FEATHER INDEX 2
//  (variant.cpp: "10,  // D2 is P0.10 (NFC2)")
//  => PHAI dung tone(2, ...) KHONG duoc dung tone(16, ...)
//     vi tone() tra g_map[pin]:
//       tone(2)  -> g_ADigitalPinMap[2]  = 10 = P0.10  <- DUNG
//       tone(16) -> g_ADigitalPinMap[16] = 30 = P0.30  <- SAI (chan khac)
//
//  P0.10 hien TRONG vi hang 3 ban phim da doi sang P1.04 (D8).
//  Da test bang sketch rieng: buzzer keu ro => chan P0.10 TOT.
//  (Dieu nay cung chung minh: hang 3 ban phim chet truoc day la do
//   DAY LONG, khong phai chan chip hong.)
//
//  XUNG DOT PERIPHERAL: da kiem tra nrf52840.h
//    NRF_PWM2_BASE  = 0x40022000   <- tone() dung cai nay
//    NRF_SPIM2_BASE = 0x40023000   <- LoRa SPI dung cai nay
//  => Dia chi KHAC NHAU => 2 peripheral rieng => KHONG xung dot.
//  Buzzer keu khong anh huong RF.
//
//  tone() dung PWM phan cung nen KHONG chiem CPU:
//  do tu sketch test, moi buoc quet chi ~177ms thay vi ~300ms
//  => tone() tra ve ngay, phan cung tu phat du thoi gian.
//  => Buzzer keu lien tuc ma khong lam cham RF hay mat phim.
// ---------------------------------------------------------------------
#define BUZZER_ENABLED     1
#define BUZZER_PIN         2       // Feather index 2 = P0.10 (SuperMini D16)

// Tan so (Hz). 16-ohm buzzer thuong cong huong ~2kHz.
// !!! DO THUC TE BANG BUZZER CUA DO AN: 2000 Hz LA TO NHAT !!!
// (buzzer 16 ohm co tan so danh dinh 2 kHz - khop voi quet 1000-4000 Hz)
// Ban dau dat 2500 Hz; sau khi quet thi 2000 Hz nghe to nhat -> doi lai.
#define BUZZER_FREQ_ALARM  2000    // bao chay: to nhat theo do thuc te
#define BUZZER_FREQ_CLEAR  900     // het bao dong: tram, diu

// Nhip beep khi bao dong (ms): keu 200ms, nghi 300ms -> nhu coi hut
#define BUZZER_ON_MS       200
#define BUZZER_OFF_MS      300

// Thoi luong tieng "het bao dong" (ms)
#define BUZZER_CLEAR_MS    900

// ---------------------------------------------------------------------
//  AM BAO KET QUA GUI TIN (OK / FAIL) - chi bao 1 lan, khong lap
// ---------------------------------------------------------------------
// Muc dich: nguoi dung biet ngay tin da gui duoc hay chua ma KHONG can
// nhin man hinh LCD. Rat huu ich khi di do khoang cach ngoai duong.
//
// Quy uoc am thanh (khong trung voi bao dong / huy bao dong):
//   - GUI THANH CONG : tan so CAO, 2 tieng NGAN  -> nghe nhe nhang, tich cuc
//   - GUI THAT BAI   : tan so TRAM, 3 tieng DAI   -> nghe nang ne, can chu y
//
// So sanh voi 2 am hien co:
//   bao dong  = 2000 Hz, lap vo han (coi hut)
//   huy bao   =  900 Hz, 1 tieng dai 900 ms
//   => OK 1800 Hz / FAIL 600 Hz khong trung tan so nao -> khong nham lan.
//
// Chi bao MOT LAN cho moi ket qua, KHONG lap lai nhu bao dong.
#define BUZZER_BEEP_RESULT      1     // 1 = co bao ket qua gui tin, 0 = tat

#define BUZZER_FREQ_TX_OK       1800  // gui thanh cong: cao, gon
#define BUZZER_FREQ_TX_FAIL     600   // gui that bai: tram, nang

// Nhip tieng THANH CONG: "tit-tit" (2 tieng ngan)
#define BUZZER_OK_BEEP_MS       80    // moi tieng keu 80 ms
#define BUZZER_OK_GAP_MS        70    // nghi giua 2 tieng
#define BUZZER_OK_COUNT         2     // 2 tieng

// Nhip tieng THAT BAI: "bup-bup-bup" (3 tieng dai hon)
#define BUZZER_FAIL_BEEP_MS     150
#define BUZZER_FAIL_GAP_MS      120
#define BUZZER_FAIL_COUNT       3

// Sau bao lau thi thoi hien bao dong NHAN TU NODE KHAC, neu khong con
// nghe goi lap lai nua. Moi lan nhan duoc goi lap, dong ho nay reset.
// => Chung nao ben kia con phat, ben nay con hien. Mat song thi tu tat.
#define ALARM_RX_HOLD_MS  45000

// --- Trang thai bao dong do MINH phat ra ---
bool          alarmActive    = false;  // dang o che do bao dong
// LUU Y: alarmAcked / alarmAckEver khai bao o duoi (muc "Hai trang thai
// ACK rieng biet"). KHONG khai bao lai o day -> loi "redefinition".
uint16_t      alarmAttempt   = 0;      // so lan da phat
uint16_t      alarmLastSeq   = 0;      // seq cua goi cuoi, de doi chieu ACK
unsigned long alarmNextTxAt  = 0;      // thoi diem phat goi ke tiep
unsigned long alarmStartedAt = 0;

// --- Trang thai bao dong NHAN TU NODE KHAC ---
bool          alarmRxActive  = false;
char          alarmRxSrc     = '\0';   // node nao bao dong
unsigned long alarmRxAt      = 0;      // lan cuoi nghe thay goi lap

// --- Nhaps nhay den nen ---
bool          blinkOn        = true;
unsigned long blinkAt        = 0;

// --- Thong bao ngan han TREN LCD khi mot phim bi chan luc bao dong ---
// Van de that su da xay ra: nguoi dung bam F2/#/* tren node dang NHAN bao
// dong va thay "khong an", tuong ban phim hong hoac day long.
// Nguyen nhan: guard chan phim dung nhu thiet ke, nhung phan hoi chi duoc
// in ra SERIAL - ma node chay pin ngoai hien truong thi KHONG co Serial.
//
// Bien nay cho phep LCD noi ra ly do, trong 1.5 giay, che tam man hinh
// bao dong. KHONG dung showTransient() vi no bi SCREEN_ALARM che mat.
String        alarmNotice      = "";
unsigned long alarmNoticeUntil = 0;
#define ALARM_NOTICE_MS  1500

// --- Da "xac nhan" canh bao nhan tu node khac ---
// Node chi la NGHE bao dong thi khong nen bi khoa ban phim: nguoi van can
// go tin phan hoi (VD "da so tan"). Khi true, man hinh bao dong khong
// chiem lai LCD nua, chi con danh dau nho tren man soan tin.
bool alarmRxDismissed = false;

// ---------------------------------------------------------------------
//  TRANG THAI BUZZER
// ---------------------------------------------------------------------
// buzzerState : 0 = im
//               1 = dang beep bao dong (lap vo han)
//               2 = vua phat tieng huy bao dong
//               3 = dang phat chuoi tieng bao KET QUA GUI TIN (OK hoac FAIL)
// buzzerAt    : moc thoi gian cho nhip beep ke tiep
//
// Dung millis() de tao nhip beep - KHONG dung delay() vi:
//   delay() chan loop() -> khong nhaps nhay duoc, khong quet phim duoc,
//   va quan trong nhat la KHONG NHAN DUOC GOI RF trong luc cho.
// tone() da tra ve ngay (PWM phan cung) nen viec bat/tat chi ton vai us.
uint8_t       buzzerState   = 0;
unsigned long buzzerAt      = 0;
bool          buzzerOn      = false;   // dang trong pha keu (giua ON va OFF)

// --- Trang thai cho chuoi tieng bao KET QUA GUI TIN (state 3) ---
// Khac voi bao dong: chi phat MOT CHUOI huu han roi tu tat, khong lap.
//   resBeepLeft  : so tieng con lai can phat
//   resBeepFreq  : tan so dang phat (1800 Hz OK / 600 Hz FAIL)
//   resBeepOnMs  : thoi luong moi tieng
//   resBeepGapMs : thoi gian nghi giua 2 tieng
uint8_t       resBeepLeft   = 0;
uint16_t      resBeepFreq   = 0;
uint16_t      resBeepOnMs   = 0;
uint16_t      resBeepGapMs  = 0;

// --- Khai bao truoc (prototype) ---
// buzzerSendResult() duoc goi trong sendMessage() (o TREN) nhung dinh nghia
// o duoi. Khai bao tuong minh o day de ro rang.
void buzzerSendResult(bool ok);
void buzzerSilence();

// --- Hai trang thai ACK rieng biet ---
// alarmAcked    : chi tinh cho LAN PHAT GAN NHAT. Reset truoc moi lan phat.
//                 Dung cho LCD hien "OK". Neu dung bien "dinh" (sticky) thi
//                 khi ben kia mat song, LCD van hien "OK" mai mai -> nguoi
//                 dung tuong bao dong van duoc nhan. Rat nguy hiem.
// alarmAckEver  : dinh (sticky), chi reset khi bat dau/huy bao dong.
//                 Dung de quyet dinh chu ky lap: da tung co nguoi nhan thi
//                 lap cham lai (20s) de tiet kiem pin va bot chen kenh.
// TACH 2 BIEN vi mot bien duy khong the vua phan anh trung thuc link
// hien tai, vua giu duoc loi ich tiet kiem pin.
bool alarmAcked   = false;
bool alarmAckEver = false;

unsigned long lastDebounceTime = 0;
char lastKey = '\0';

// Man hinh hien tai: SCREEN_EDIT / SCREEN_MSG / SCREEN_STATS
// (gia tri khoi tao = SCREEN_EDIT, dinh nghia o tren)
uint8_t screenMode = SCREEN_EDIT;
unsigned long modeSwitchAt = 0;
// LUU Y: AUTO_BACK_MS duoc dinh nghia O TREN (dong ~323) = 0,
// nghia la khong tu quay ve man hinh soan. KHONG dinh nghia lai o day.

bool needRedraw = true;

// ---------------------------------------------------------------------
//  CACHE CHONG LAP (dedup) - ngan mot goi den 2 lan bi xu ly 2 lan
// ---------------------------------------------------------------------
#define DEDUP_SLOTS 8
struct DedupEntry { char src; uint16_t seq; char type; bool used; };
DedupEntry dedup[DEDUP_SLOTS];
uint8_t dedupIdx = 0;

// =====================================================================
//  HAM HO TRO RF
// =====================================================================
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

  // Loc ngat gia / glitch DIO1: chi xu ly khi co bit RX_DONE that
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

  p.rssi = radio.getRSSI();   // doc packet status, hop le sau readData()
  p.snr  = radio.getSNR();

  // --- Parse: src,dst,type,hop,seq,payload ---
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

  // Loc goi rac. Dung macro chung de khong bao gio quen loai goi moi.
  // (Cu: chi chap nhan 'D'/'K' -> goi bao chay 'F' se bi VUT IM LANG.)
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

// CAD: cho kenh trong truoc khi phat, tranh va cham
// scanChannel() tra ve RADIOLIB_LORA_DETECTED (-702) hoac RADIOLIB_CHANNEL_FREE (-15)
bool waitForChannelFree(uint32_t maxWaitMs) {
  uint32_t t0 = millis();
  for (;;) {
    int16_t st = radio.scanChannel();
    if (st == RADIOLIB_CHANNEL_FREE) return true;
    if (st != RADIOLIB_LORA_DETECTED) {
      if (Serial) { Serial.print("[CAD] loi "); Serial.println(st); }
      return true;   // loi khac thi khong nen chan phat mai
    }
    if (millis() - t0 > maxWaitMs) {
      if (Serial) Serial.println("[CAD] het cho, kenh van ban");
      return false;
    }
    delay(20);
  }
}

int16_t txPacket(const String &raw) {
  String tmp = raw;   // transmit() can String& (lvalue)

  // -----------------------------------------------------------------
  //  NHUONG DUONG CHO BAO DONG (tin khan cap).
  //
  //  Khi tren mang co bao dong - hoac MINH dang phat, hoac MINH nghe
  //  thay bao dong cua node khac - thi tang kien nhan cho kenh trong
  //  tu 800 ms len 3000 ms. Ly do chi tiet xem comment CAD_WAIT_ALARM_MS.
  //
  //  KHONG anh huong gi den ban phim: day chi la cho luc PHAT song.
  //
  //  2 loai goi KHONG phai nhuong duong (phat nhanh, khong cho 3 giay):
  //    - Goi bao dong ('F') cua CHINH MINH: can di nhanh de tang co hoi
  //    - Goi huy bao dong ('C'): can di nhanh de cac node tat canh bao som
  //  Chi goi THUONG moi nhuong duong.
  // -----------------------------------------------------------------
  bool alarmOnNet  = alarmRxActive;   // nghe bao dong tu node khac
  bool iAmAlarming = alarmActive;     // minh dang phat bao dong
  bool isUrgentPkt = (raw.length() > 4) &&
                     (raw[2] == TYPE_ALARM || raw[2] == TYPE_CLEAR);

  uint32_t cadWait = CAD_WAIT_MS;                        // mac dinh
  if (alarmOnNet || iAmAlarming) {
    if (!isUrgentPkt) cadWait = CAD_WAIT_ALARM_MS;       // tin thuong -> nhuong duong
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

// =====================================================================
//  LCD
// =====================================================================
// In 1 dong, tu xoa phan thua bang khoang trang
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

// ---------------------------------------------------------------------
//  Bao ket qua ngan han tren LCD (VD "OK da gui" + RTT)
//  Chi cap nhat bien, KHONG ve truc tiep -> khong bi redraw() ghi de.
// ---------------------------------------------------------------------
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

  // -----------------------------------------------------------------
  //  MAN HINH SOAN TIN - TIN NHAN CHIEM CA 2 DONG
  // -----------------------------------------------------------------
  //  !!! DA SUA THEO YEU CAU !!!
  //  Ban cu: dong 0 = "B TX:3 RX:2" (bo dem chiem cho), dong 1 = tin.
  //  Ban moi: dong 0 = "B:" + tin (phan 1), dong 1 = tin (phan 2).
  //  => Tin hien duoc 30 ky tu thay vi 16.
  //  => Bo dem TX/RX da chuyen sang man thong so (bam 'D').
  //
  //  Tien to "B:" = 2 cot. Neu dang NGHE bao dong tu node khac thi
  //  dung "B!:" = 3 cot de van con dau hieu canh bao (nguoi dung da gat
  //  man hinh canh bao di nhung bao dong VAN CON HIEU LUC).
  //  Phan con lai cua dong 0 tinh theo do dai tien to thuc te.
  // -----------------------------------------------------------------
  String prefix;
  if (alarmRxActive) {
    prefix = String(NODE_ID) + "!:";   // 3 ky tu
  } else {
    prefix = String(NODE_ID) + ":";    // 2 ky tu
  }

  // Hien 30 ky tu CUOI cua buffer (cho thay vi tri dang go).
  // Khi buffer ngan hon thi hien het, khong dem khoang trang.
  String tail = inputBuffer;
  if (tail.length() > MSG_VISIBLE) {
    tail = tail.substring(tail.length() - MSG_VISIBLE);
  }

  uint8_t room0 = LCD_COLS - prefix.length();   // 14 hoac 13 cot con lai
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

// -----------------------------------------------------------------
  //  MAN HINH TIN NHAN - TIN CHIEM CA 2 DONG
  // -----------------------------------------------------------------
  //  !!! DA SUA THEO YEU CAU !!!
  //  Tien to ro nghia "TU A:" / "TU B:" thay vi chi "A:" / "B:".
  //
  //  TIEN TO LUON DUNG 5 KY TU (quan trong cho toan cuon):
  //    binh thuong : "TU A:"   (5)
  //    co bao dong : "TU A!"   (5)  <- doi ':' thanh '!' khi dang nghe
  //                                 canh bao, GIU NGUYEN do dai
  //  Neu de '!' lam ky tu thu 6 thi do dai doi -> toan cuon phuc tap
  //  va de sinh bug tran cot.
  //
  //  Do dai - DA MO PHONG:
  //    dong 0: "TU A:" (5) + 11 ky tu tin = 16 cot
  //    dong 1: 16 ky tu tin
  //    => 27 ky tu tin hien cung luc
  //
  //  CUON: bat dau khi L > 24 (27 tru 3 cot dem), maxPos = 3 + L - 27.
  //  Da mo phong: khong tran cot o moi do dai 1..32, ky tu cuoi luon hien.
  // -----------------------------------------------------------------
  String prefix = "TU " + String(lastMsgSrc);
  if (alarmRxActive) prefix += "!";   // 5 ky tu - dau hieu con canh bao
  else               prefix += ":";   // 5 ky tu

  String cur = lastMsg;
  if (msgNeedScroll) {
    // 3 khoang trang dem truoc de tin "chay vao" tu phai sang trai
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

// ---------------------------------------------------------------------
//  MAN HINH THONG SO CHI TIET (bam phim 'D' khi dang xem tin nhan)
// ---------------------------------------------------------------------
// LCD 1602 chi co 2 dong nen khong the vua hien het tin nhan vua hien
// day du so lieu. Man hinh nay tach rieng cac chi so de ban ghi chep
// khi di do khoang cach, dac biet la de so sanh bien duong so voi -137dBm.
//
//  !!! THIET KE LAI - TACH THANH 2 MAN (STATS + LINK) !!!
//  Ly do: 4 bo dem x 3 chu so = 12 chu so + 4 nhan = 16 ky tu, khong con
//  cho khoang trang -> TRAN 16 cot khi bo dem lon (da mo phong, phat hien
//  2 truong hop tran). Tach ra thi moi man con nhieu cho:
//    STATS: chat luong song  + khoi luong tin (TX/RX)
//    LINK : duong truyen (hop/seq) + do tin cay (ACK ok/fail)
//
//  Do dai da MO PHONG, tat ca <= 16 cot ke ca khi 3 chu so:
  //    STATS d0: "A -127dBm S-20.0" = 16 (XAU NHAT, vua khit)
  //              "A -50dBm S11.0"   = 14 (binh thuong)
  //    STATS d1: "TX999 RX999"  = 11
  //    LINK  d0: "HOP2 SEQ999"  = 11   LINK  d1: "OK999 F999"   = 10
  // ---------------------------------------------------------------------
void drawStatsScreen() {
  if (!lcdOk) return;

  // Dong 0: NGUON + chat luong song (kem don vi dBm theo yeu cau).
  //   SNR la chi so QUAN TRONG HON RSSI de danh gia LoRa:
  //   link khoe thuong SNR duong (+5 den +15 dB); khi SNR ve gan 0 dB
  //   la dang tien gan nguong mat goi, du RSSI con cao.
  //
  //   DO DAI - DA MO PHONG:
  //     "A -127dBm S-20.0" = 16 ky tu -> VUA KHIT 16 cot (truong hop xau nhat)
  //     "A -50dBm S11.0"   = 14 ky tu
  //   Neu sau nay them gi vao dong nay, PHAI mo phong lai do dai truoc,
  //   vi 16 cot da kin khi RSSI -127 dBm va SNR am.
  String l0 = lastMsgSrc + " " + String(lastRSSI) + "dBm";
  l0 += " S" + String(lastSNR10 / 10.0, 1);
  lcdLine(0, l0);

  // Dong 1: khoi luong tin cua chinh node nay
  //   "TX999 RX999" = 11 ky tu -> con du 5 cot, khong bao gio tran
  String l1 = "TX" + String(statTx);
  l1 += " RX" + String(statRx);
  lcdLine(1, l1);
}

// ---------------------------------------------------------------------
//  MAN HINH LIEN KET (bam 'D' lan thu 2) - chan doan repeater
// ---------------------------------------------------------------------
// Man nay phuc vu viec test repeater: hop > 0 nghia la goi da di qua
// it nhat 1 repeater. seq de doi chieu voi log Serial ben kia.
// OK/F la so lan ACK thanh cong / that bai -> danh gia do tin cay.
void drawLinkScreen() {
  if (!lcdOk) return;

  // Dong 0: duong truyen cua tin nhan cuoi
  //   HOP0 = truc tiep,  HOP1 = qua 1 repeater
  String l0 = "HOP" + String(lastHop);
  l0 += " SEQ" + String(lastSeq);
  lcdLine(0, l0);

  // Dong 1: do tin cay gui tin
  //   OK = so lan ben kia xac nhan, F = so lan that bai
  String l1 = "OK" + String(statAckOk);
  l1 += " F" + String(statAckFail);
  lcdLine(1, l1);
}

// ---------------------------------------------------------------------
//  MAN HINH BAO DONG - uu tien cao nhat, che man hinh khac
// ---------------------------------------------------------------------
// Dung chu in hoa va dau ! de noi bat tren LCD 16x2 khong co tieng Viet.
// Do dai da kiem tra vua khop 16 cot:
//   "!! BAO CHAY !!"     = 14 ky tu
//   "tu A  lan 12/999"   = 16 ky tu (dai nhat khi attempt 3 chu so)
//   "tu A  DA XAC NHAN"  = 17 -> se bi cat, nen dung ban ngan ben duoi
void drawAlarmScreen() {
  if (!lcdOk) return;

// Dong 0: tieu de bao dong.
  // !!! DA DOI THEO YEU CAU: "CO CHAY XAY RA!" thay vi "!! BAO CHAY !!" !!!
  // Do dai = 16 ky tu -> VUA KHIT 1 dong LCD, khong bi cat.
  // Van giu dau '!' o cuoi lam dau hieu khan cap, vi LCD 1602 don sac
  // KHONG the lam chu nhap nhay hay doi mau - dau '!' la tin hieu duy nhat
  // con lai ngoai den nen nhap nhay.
  lcdLine(0, "CO CHAY XAY RA!");

  // -----------------------------------------------------------------
  //  Dong 1: trang thai + huong dan huy.
  //
  //  !!! DA SUA LOI TRAN 16 COT !!!
  //  Ban dau viet: l1 += " *'='huy"  -> nhieu truong hop thanh 18-20 ky tu.
  //  lcdLine() cat tu PHAI sang, nghia la phan "*'='huy" (huong dan HUY
  //  bao dong) bi CAT MAT - dung vao luc nguoi dung can no nhat.
  //
  //  Cach sua: dat SUFFIX co dinh " *=huy" (6 ky tu), va CAT phan trang
  //  thai xuong con 10 ky tu de tong LUON <= 16. Uu tien giu suffix hon
  //  giu trang thai, vi nguoi dung phai biet cach tat canh bao.
  // -----------------------------------------------------------------
  const char *SUFFIX = " *=huy";
  const uint8_t SUF_LEN = 6;
  const uint8_t STATUS_MAX = LCD_COLS - SUF_LEN;   // 10

  String st;
  if (alarmActive) {
    // Node nay DANG phat bao dong.
    // Dung "x<lan>" thay vi "phat x<lan>" de du cho duoc ca chu "OK":
    //   "x12 OK" = 6 ky tu; neu de "phat x12 OK" = 10 ky tu thi khi ghep
    //   suffix 6 ky tu se bi cat mat chu K -> hien "phat x12 O" trong
    //   nhu loi hien thi.
    //
    // max(1, alarmAttempt): alarmStart() dat alarmAttempt = 0, con
    // alarmTick() moi ++ truoc khi phat. Giua 2 buoc do la 1 vong loop
    // (~vai ms), LCD se loe "x0" roi doi thanh "x1". Hien "x1" ngay tu
    // dau cho khoi khien nguoi dung nghi he thong dem sai.
    uint16_t shownAttempt = (alarmAttempt == 0) ? 1 : alarmAttempt;
    st = "x";
    st += (shownAttempt > 999 ? 999 : shownAttempt);   // gioi han 3 chu so
    if (alarmAcked) st += " OK";
  } else {
    // Node nay NHAN bao dong tu node khac.
    // Dung tu ngan ("tu A lap") thay vi "tu A dang lap" de khong bi cat
    // giua chung tao ra khoang trang doi trong nhu loi hien thi.
    st = "tu ";
    st += alarmRxSrc;
    st += " lap";
  }
  if (st.length() > STATUS_MAX) st = st.substring(0, STATUS_MAX);

  lcdLine(1, st + SUFFIX);
}

void redraw() {
  // -----------------------------------------------------------------
  //  UU TIEN 1 (cao nhat): MAN HINH BAO DONG.
  //  PHAI xu ly TRUOC transient. Neu de transient truoc thi luc vua
  //  bam '#' xong, dong "OK da gui" hoac "Chua co noi dung" se che mat
  //  bao dong - dung vao luc nguoi dung can thay nhat.
  // -----------------------------------------------------------------
  if (screenMode == SCREEN_ALARM) {
    drawAlarmScreen();
    drawAlarmNoticeIfDue();   // lop phu thong bao, neu dang den han
    needRedraw = false;
    return;
  }

  // UU TIEN 2: man hinh tam thoi (ket qua gui)
  // Neu da het thoi gian hien thi thi tu dong tat va ve man hinh binh thuong.
  if (transientOn) {
    if ((long)(millis() - transientUntil) > 0) {
      transientOn = false;      // het gio -> tro ve man hinh thuong
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

  // -----------------------------------------------------------------
  //  LOP PHU THONG BAO - PHAI nam ngoai cac man hinh.
  //  !!! DA SUA LOI !!!
  //  Ban dau doan nay nam TRONG drawAlarmScreen(). Hau qua: khi bam '*'
  //  tren node chi NGHE bao dong, screenMode doi sang SCREEN_EDIT nen
  //  drawAlarmScreen() khong con duoc goi -> thong bao "Da thay, go tin"
  //  KHONG BAO GIO hien ra. Nguoi dung bam '*' ma thay khong co gi,
  //  dung la trieu chung "3 phim khong an" ma ban da bao.
  //  Nay tach thanh ham rieng, goi o MOI nhanh cua redraw().
  // -----------------------------------------------------------------
  drawAlarmNoticeIfDue();
  needRedraw = false;
}

// ---------------------------------------------------------------------
//  Ve lop phu thong bao len DONG 1, trong ALARM_NOTICE_MS.
//  Giu nguyen dong 0 de khong che mat tieu de man hinh dang hien.
//  Do dai da kiem tra: 3 chuoi deu <= 16 cot.
// ---------------------------------------------------------------------
void drawAlarmNoticeIfDue() {
  if (!lcdOk) return;
  if (alarmNotice.length() == 0) return;
  if ((long)(millis() - alarmNoticeUntil) >= 0) {
    alarmNotice = "";     // het han -> xoa de khong ve lai nua
    return;
  }
  lcdLine(1, alarmNotice);
}

// =====================================================================
//  BAN PHIM
// =====================================================================
void initKeypad() {
  for (uint8_t c = 0; c < COLS; c++) {
    nrf_gpio_cfg_output(colPins[c]);
    nrf_gpio_pin_set(colPins[c]);       // idle = HIGH
  }
  for (uint8_t r = 0; r < ROWS; r++) {
    nrf_gpio_cfg_input(rowPins[r], NRF_GPIO_PIN_PULLUP);
  }
}

// ---------------------------------------------------------------------
//  MO KHOA NFC CHO P0.09 / P0.10
// ---------------------------------------------------------------------
// Hang 3 cua ban phim nam tren P0.10 (= NFC2), hang 4 tren P0.09 (= NFC1).
// nRF52840 co bit UICR->NFCPINS: neu bit nay = 1 thi 2 chan bi danh rieng
// cho NFC va KHONG the dung lam GPIO -> doc luon tra ve gia tri co dinh,
// khong bao gio thay doi khi bam phim.
//
// Core Adafruit nRF52 1.7.0 KHONG dinh nghia CONFIG_NFCT_PINS_AS_GPIOS
// (da kiem tra platform.txt), nen khoi mo khoa trong
// system_nrf52840.c dong 148-158 KHONG duoc bien dich.
//
// Ham duoi day lam dung viec khoi do (sao chep logic tu Nordic) de
// dam bao P0.09/P0.10 dung duoc lam GPIO, bat ke bootloader da dat gi.
//
// LUU Y: chi ghi UICR khi that su can, va goi NVIC_SystemReset() sau do
// (bat buoc de thay doi co hieu luc). Nghia la MAY SE RESET 1 LAN o
// lan chay dau tien, roi tu lan sau khong con ghi nua.
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

  if (!prot) return;   // khong can lam gi

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

  NVIC_SystemReset();   // bat buoc de thay doi UICR co hieu luc
}

// ---------------------------------------------------------------------
//  IN TRANG THAI THO CUA TUNG CHAN HANG
// ---------------------------------------------------------------------
// Dung de phan biet 2 nguyen nhan khac nhau:
//   - Chan hang KHONG BAO GIO xuong 0  -> loi GPIO/NFC/day dut
//   - Chan hang xuong 0 nhung van khong ra phim -> loi cot hoac keyMap
// Gia tri mong doi khi khong bam: tat ca = 1 (co dien tro keo len).
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

// ---------------------------------------------------------------------
//  QUET TUNG COT - VA DIEM MU CUA printRowRawStates()
// ---------------------------------------------------------------------
// !!! DIEM MU NGHIEM TRONG DA DUOC VA !!!
// printRowRawStates() doc hang khi KHONG keo cot nao xuong LOW. Khi do
// moi hang deu = 1 (pull-up) du ban phim tot hay hong. Nghia la:
//   NEU MOT CHAN COT CHET -> khong hang nao xuong duoc 0
//   -> ham do van in "H1=1 H2=1 H3=1 H4=1 H5=1" = TRONG NHU BINH THUONG
//   -> ban ket luan "phan cung tot" trong khi that ra cot chet.
//
// Ham nay khac phuc: keo LAN LUOT tung cot xuong LOW roi doc ca 5 hang.
// CACH DOC KET QUA: GIU MOT PHIM va nhin 4 dong in ra.
//   - Phim do thuoc cot nao thi cot do phai co DUNG 1 bit = 0
//   - Neu ca 5 bit van = 1 khi dang giu phim => COT do chet
//
// Vi du giu phim F2 (hang 1, cot 2), ket qua BINH THUONG:
//   [COT1] 1 1 1 1 1
//   [COT2] 0 1 1 1 1   <- H1=0 => F2 o day, cot D9 TOT
//   [COT3] 1 1 1 1 1
//   [COT4] 1 1 1 1 1
// Neu [COT2] van in "1 1 1 1 1" khi dang giu F2 => dut day D9 hoac
// switch phim F2 hong. Do la loi PHAN CUNG, khong phai firmware.
void printColumnScanOnce(void) {
  if (!Serial) return;

  const char *colName[COLS] = { "D6 ", "D9 ", "D19", "D18" };

  for (uint8_t c = 0; c < COLS; c++) {
    nrf_gpio_pin_clear(colPins[c]);      // keo cot nay xuong LOW
    delayMicroseconds(20);

    Serial.print("  [COT");
    Serial.print(c + 1);
    Serial.print(' ');
    Serial.print(colName[c]);
    Serial.print("]");
    for (uint8_t r = 0; r < ROWS; r++) {
      Serial.print(' ');
      Serial.print(nrf_gpio_pin_read(rowPins[r]));   // H1..H5
    }

    nrf_gpio_pin_set(colPins[c]);        // tra cot ve HIGH
    Serial.println();
  }
}

// ---------------------------------------------------------------------
//  TEST DIEN TUNG CHAN COT (ep OUTPUT roi doc lai)
// ---------------------------------------------------------------------
// Giong testRowPinsElectrical() nhung cho 4 chan cot.
// Phep do nay KHONG can ban phim: do truc tiep tren chan GPIO cua chip.
// Chan cot tot phai doc ve LOW=0 va HIGH=1 khi bi ep. Neu khong => chan chet.
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

  initKeypad();   // tra ve che do quet phim binh thuong
}

// ---------------------------------------------------------------------
//  TEST DIEN TUNG CHAN HANG - BANG CHUNG DUT DIEM
// ---------------------------------------------------------------------
// Moi chan hang duoc tam cau hinh lam OUTPUT roi bi ep ve LOW, sau do
// doc lai. Chan khoe thi doc ve dung 0. Chan bi khoa (NFC) hoac day dut
// thi doc ve sai.
//
// Day la phep do TRUC TIEP tren silicon, khong suy luan tu UICR, nen
// phan biet duoc:
//   - P0.10 (hang 3) hong nhung P0.09 (hang 4) tot -> KHONG phai loi NFC,
//     vi ca hai bi khoa boi cung mot bit UICR. Do la loi day/han.
//   - Ca P0.09 va P0.10 deu hong  -> dung la NFC chua duoc mo khoa.
//
// Sau khi test xong phai goi lai initKeypad() de tra ve che do INPUT+PULLUP.
void testRowPinsElectrical(void) {
  if (!Serial) return;

  // Khong de const vi co the can sua ten hang 3 theo ROW3_ALT_PIN
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

    // Tam chuyen sang OUTPUT de chu dong dieu khien muc dien
    nrf_gpio_cfg_output(pin);

    nrf_gpio_pin_clear(pin);            // ep ve LOW
    delayMicroseconds(50);
    uint32_t low  = nrf_gpio_pin_read(pin);

    nrf_gpio_pin_set(pin);              // ep ve HIGH
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
  // -----------------------------------------------------------------
  //  Do rieng chan D16 (P0.10) - chan hang 3 CU.
  //  Muc dich: lay bang chung DUT DIEM ve nguyen nhan hang 3 chet.
  //
  //   [HONG] -> chan P0.10 chet that (khong dieu khien duoc muc dien).
  //             Cach sua dung: dung D8, nhu code dang lam.
  //   [OK]   -> chan van tot, vay loi la DAY NOI / tiep xuc kem giua
  //             phim va D16. Co the cam lai day vao D16 cung duoc.
  //
  //  Phep do nay khong anh huong gi den ban phim vi D16 khong con
  //  nam trong rowPins[] nua.
  // -----------------------------------------------------------------
  {
    uint32_t oldPin = NRF_GPIO_PIN_MAP(0, 10);   // D16 / P0.10
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

    // Tra D16 ve trang thai trung tinh, khong giu no lam OUTPUT
    nrf_gpio_cfg_input(oldPin, NRF_GPIO_PIN_NOPULL);
  }
#endif

  initKeypad();   // tra ve INPUT + PULLUP de quet phim binh thuong
}

char scanKeypad() {
  for (uint8_t c = 0; c < COLS; c++) {
    nrf_gpio_pin_clear(colPins[c]);     // keo cot nay xuong LOW
    delayMicroseconds(15);
    for (uint8_t r = 0; r < ROWS; r++) {
      if (nrf_gpio_pin_read(rowPins[r]) == 0) {   // hang nay dang LOW = phim
        nrf_gpio_pin_set(colPins[c]);
        return keyMap[r][c];
      }
    }
    nrf_gpio_pin_set(colPins[c]);
  }
  return '\0';
}

// ---------------------------------------------------------------------
//  Giong scanKeypad() nhung tra ve TOA DO hang/cot thay vi ky tu.
//  Chi dung cho che do tu chan doan (KEYPAD_SELFTEST).
// ---------------------------------------------------------------------
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

// ---------------------------------------------------------------------
//  CHAN DOAN BAN PHIM
// ---------------------------------------------------------------------
// Bam lan luot tung phim. May in ra:
//   - toa do hang/cot ma code NHIN THAY
//   - GPIO that cua hang do
//   - ky tu ma keyMap tra ve
// Nho do biet ngay la lech HANG hay lech COT, thay vi phai doan.
//   Phim [F2] dung phai in:  HANG=0 COT=1  ->  'B'
//   Phim [2]  dung phai in:  HANG=1 COT=1  ->  '2'
//   Phim [0]  dung phai in:  HANG=4 COT=1  ->  '0'
// Neu HANG sai  -> dao thu tu rowPins[]
// Neu COT  sai  -> dao thu tu colPins[]
//
// !!! GIOI HAN QUAN TRONG - PHAI BIET DE KHONG BI LUA !!!
// Che do nay chi kiem tra PHAN CUNG (quet ma tran -> ra dung toa do).
// No KHONG chay qua handleKey(), nen khong phat hien duoc phim bi
// "nuot" o tang xu ly. Da tung xay ra: self-test bao 20 phim deu tot,
// nhung handleKey() lai return som voi A/B/L/R/# nen hang 1 va nut
// L/R hang 5 van khong nhap duoc chu.
// => VI VAY man hinh nay in them cot "vai tro" de ban thay ky tu do
//    se duoc xu ly the nao. VA VAN PHAI TEST THAT bang cach tat
//    self-test roi bam phim xem chu co len LCD khong.
//  Sau bao lau thi TU THOAT khoi che do chan doan (giay).
//  !!! CANH BAO AN TOAN !!!
//  keypadSelfTest() dung loop rieng va KHONG khoi tao LoRa. Neu ban bat
//  KEYPAD_SELFTEST = 1 de sua ban phim roi QUEN dat lai 0, node se ket
//  mai mai trong chan doan va khong bao gio phat/nhan song duoc.
//  Tu thoat sau 30 giay de loi do khong the lam hong mot node.
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
    // --- TU THOAT THEO THOI GIAN (chong ket vinh vien) ---
    if ((long)(millis() - deadline) >= 0) {
      if (Serial) {
        Serial.println();
        Serial.print("[CHAN DOAN] Het ");
        Serial.print(SELFTEST_AUTOEXIT_SEC);
        Serial.println(" giay -> TU THOAT, tiep tuc khoi dong binh thuong.");
      }
      return;
    }

    // Thoat bang Serial
    if (Serial.available() > 0) {
      char ch = (char)Serial.read();
      if (ch == 'q' || ch == 'Q') { Serial.println("[CHAN DOAN] Thoat."); return; }
    }

    // Moi 500ms: in trang thai hang va quet tung cot.
    // PHAI in ca 2: [RAW HANG] mot minh khong phat hien duoc cot chet.
    if (millis() - lastRaw >= 500) {
      lastRaw = millis();
      printRowRawStates();
      printColumnScanOnce();
      Serial.println("  ^ GIU 1 phim roi so sanh 4 dong [COT..] o tren");
    }

    if (scanKeypadRC(r, c)) {
      char mapped = keyMap[r][c];

      // -----------------------------------------------------------------
      //  CHAN DOAN TREN LCD - cho node chay PIN, khong co Serial Monitor.
      //  Voi moi phim an, hien ngay toa do + ky tu + vai tro tren LCD.
      //  Neu bam phim ma LCD KHONG doi gi -> tin hieu khong vao duoc
      //  chip -> loi phan cung (day hoac switch), khong phai firmware.
      //  (Day la phan LCD cu da co, giu nguyen.)
      // -----------------------------------------------------------------

      // Mo ta ky tu nay se duoc xu ly the nao o che do chay that.
      // In ra de ban doi chieu TRUC TIEP, khong con cho "nuot" phim.
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
        // Nhan ngan gon, deu <= 16 cot
        String tag = "phim nhap lieu";
        if (mapped == '#' || mapped == '*')      tag = "phim KHAN CAP";
        else if (c == 3)                          tag = "phim chuc nang";
        lcdLine(0, "H" + String(r) + " C" + String(c) + " -> '" + String(mapped) + "'");
        lcdLine(1, tag);
      }

      // Phim E = thoat
      if (mapped == 'E') { Serial.println("[CHAN DOAN] Thoat bang phim E."); return; }

      // Cho nha tay de khong in lien tuc 1 phim
      delay(250);
      while (scanKeypadRC(r, c)) { delay(20); }
    }
    delay(5);
  }
}

// =====================================================================
//  XU LY TIN NHAN
// =====================================================================
void deliverToUser(const Pkt &p) {
  statRx++;
  lastMsg    = p.payload;
  lastMsgSrc = String(p.src);
  lastRSSI   = (long)p.rssi;
  lastSNR10  = (long)(p.snr * 10);
  lastSeq    = p.seq;
  lastHop    = p.hop;

// Khoi dong trang thai cuon cho tin nhan moi.
  // Cuon khi tin dai hon SUC CHUA THUC TE cua MAN NHAN (24 = 27 - 3).
  // Da mo phong: dung 27 thi tin 25-27 ky tu bi mat 3 ky tu cuoi.
  msgNeedScroll = (lastMsg.length() > MSG_CAPACITY_RX);
  msgScrollPos  = 0;
  msgScrollAt   = millis();

  // -----------------------------------------------------------------
  //  TIN NHAN MOI UUU TIEN HON man hinh tam thoi.
  //  Neu khong huy, tin nhan vua toi se bi an sau transient
  //  (VD ban vua gui xong, "OK da gui" con dang hien 3 giay) ->
  //  ban tuong la khong nhan duoc tin.
  // -----------------------------------------------------------------
  transientOn = false;

  // -----------------------------------------------------------------
  //  NHUNG MAN HINH BAO DONG CO UU TIEN CAO HON TIN NHAN THUONG.
  //  Neu khong co dieu kien nay, mot tin nhan thuong den trong luc
  //  bao dong se keo man hinh ve SCREEN_MSG va CHE MAT canh bao chay
  //  - dung vao luc nguoi dung can thay canh bao nhat.
  //
  //  Luu y: ta VAN luu lastMsg va VAN tra ACK (ben kia biet la nhan
  //  duoc), chi khong cho no chiem man hinh. Bam 'D' van xem duoc tin.
  // -----------------------------------------------------------------
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

  // -----------------------------------------------------------------
  //  KHONG GHI LCD O DAY.
  // -----------------------------------------------------------------
  // Code cu co khoi "if (lcdOk) { lcdLine(1, RSSI/SNR); }" o cho nay.
  // No gay loi: needRedraw = true o tren se khien loop() goi redraw()
  // -> drawMsgScreen(), va ham do ve CA HAI dong. Ket qua la dong RSSI
  // vua ghi bi ghi de ngay trong cung mot luot loop, nen LCD chi bao
  // gio hien tin nhan ma khong bao gio hien RSSI/SNR.
  //
  // QUY TAC: chi drawMsgScreen() duoc phep ve man hinh tin nhan.
  // Cac ham khac chi cap nhat bien roi dat needRedraw = true.
  // -----------------------------------------------------------------
}

void sendAckFor(const Pkt &p) {
  delay(ACK_DELAY_MS);   // cho ben kia kip quay ve RX
  String ack = buildPacket(p.src, TYPE_ACK, 0, p.seq, "");
  int16_t st = txPacket(ack);
  if (Serial) {
    if (st == RADIOLIB_ERR_NONE) Serial.println("[ACK] da gui xac nhan");
    else { Serial.print("[!] gui ACK loi ma: "); Serial.println(st); }
  }
}

void handleDataPacket(const Pkt &p) {
  if (p.src == NODE_ID) return;                              // bo qua goi cua chinh minh
  if (p.dst != NODE_ID && p.dst != DST_BROADCAST) return;    // khong gui cho minh

  if (seenBefore(p.src, p.seq, p.type)) return;              // chong xu ly trung
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
    // BAO DONG tu node khac - uu tien cao nhat
    handleAlarmPacket(p);
  } else if (p.type == TYPE_CLEAR) {
    // HUY bao dong tu node khac
    handleClearPacket(p);
  } else if (p.type == TYPE_ACK) {
    // -----------------------------------------------------------------
    //  ACK cho BAO DONG cua chinh node nay?
    //  Neu dung seq thi danh dau da duoc xac nhan -> alarmTick() se
    //  chuyen sang che do lap CHAM hon (tiet kiem pin, bot chen kenh)
    //  va LCD hien "OK" thay vi dau '*'.
    // -----------------------------------------------------------------
    if (p.dst == NODE_ID && alarmActive && p.seq == alarmLastSeq) {
      if (!alarmAcked && Serial) {
        Serial.print("[SOS] Bao dong DA DUOC xac nhan (seq=");
        Serial.print(p.seq);
        Serial.println("). Chuyen sang che do lap cham.");
      }
      alarmAcked   = true;
      alarmAckEver = true;   // dinh: chi reset khi bat dau/huy bao dong
      needRedraw = true;
    }
    else if (p.dst == NODE_ID && Serial) {
      // ACK den luc khong cho (goi cu, hoac ACK cho node khac) -> chi log
      Serial.print("[ACK] nhan xac nhan lang (seq=");
      Serial.print(p.seq); Serial.println(")");
    }
  }
  enterRx();
}

// =====================================================================
//  GUI TIN CO CHO ACK
// =====================================================================
bool sendWithAck(const String &msg, uint32_t &rttMs) {
  for (uint8_t attempt = 1; attempt <= MAX_RETRY + 1; attempt++) {
    // SEQ TANG MOI LAN THU. Neu giu nguyen seq khi thu lai, cache dedup
    // cua repeater se tuong la goi trung va HUY -> thu lai khong bao gio di.
  // -----------------------------------------------------------------
  //  RESET alarmAcked TRUOC moi lan phat.
  //  !!! DA SUA LOI THAT SU !!!
  //  Ban cu chi set true khi nhan ACK va KHONG BAO GIO reset. Hau qua:
  //  luc dau node B o gan (nhan duoc, hien OK), sau do B di xa mat song,
  //  thi LCD van hien "OK" mai mai -> nguoi dung tuong bao dong van duoc
  //  nhan, trong khi that su khong ai nghe. Vo cung nguy hiem voi bao chay.
  //  Gio "OK" chi hien khi LAN PHAT GAN NHAT that su co nguoi xac nhan.
  // -----------------------------------------------------------------
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

// t_start: moc NGAY TRUOC khi phat -> do duoc ca chieu di (DATA) lan
    // chieu ve (ACK) => RTT dung nghia round-trip day du.
    // t0: moc SAU khi phat xong -> dung cho timeout cho ACK (nhu cu).
    uint32_t t_start = millis();
    int16_t st = txPacket(packet);
    if (st != RADIOLIB_ERR_NONE && Serial) {
      Serial.print("  PHAT LOI ma: "); Serial.println(st);
    }

    uint32_t t0 = millis();
    uint32_t deadline = t0 + ACK_TIMEOUT_MS;

    while (millis() < deadline) {
      Pkt p;
      if (!pollRfPacket(p, deadline - millis())) break;   // het gio

      if (p.type == TYPE_ACK && p.dst == NODE_ID && p.seq == seq) {
        rttMs = millis() - t_start;
        return true;
      }

      // Dang cho ACK ma node khac gui tin sang -> xu ly luon roi cho tiep
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

  // -----------------------------------------------------------------
  //  CHAN O TAN GOC: khong gui tin thuong khi dang bao dong.
  //  Ham nay chan loop() toi ~18 giay (2 luot x [CAD + phat + cho ACK]).
  //  Neu chay trong luc bao dong thi alarmTick() bi tre tung do ->
  //  bao dong mat tac dung lap lai dung luc can nhat.
  //
  //  Da chan o handleKey() va o duong Serial Monitor, nhung giu them
  //  guard nay de sau nay neu co duong goi moi van an toan.
  // -----------------------------------------------------------------
  //  Chi chan khi node nay LA NGUON PHAT bao dong.
  //  !!! DA SUA !!! Ban cu dung (alarmActive || alarmRxActive) -> node chi
  //  NGHE bao dong cung bi cam gui tin. Do la loi nghiem trong ve nghiep vu:
  //  trong tinh huong chay, node nhan can gui duoc tin phan hoi nhu
  //  "da so tan", "lua o huong dong". Cam no la lam he thong vo dung.
  //  Khi node nay tu phat bao dong thi moi can chan, vi moi luot loop()
  //  phai uu tien phat goi lap lai.
  // -----------------------------------------------------------------
  if (alarmActive) {
    if (Serial) {
      Serial.println("[SOS] Node nay dang phat bao dong - tu choi gui tin thuong.");
      Serial.println("      Huy bao dong bang '*' (hoac lenh 'huy') roi moi gui.");
    }
    // KHONG goi showTransient() o day: redraw() uu tien SCREEN_ALARM hon
    // transient, nen thong bao se khong bao gio hien ra. Chi bao qua Serial.
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
  // Ghi nhan so tin DA nhan TRUOC khi gui, de phat hien neu co tin moi
  // den trong luc cho ACK.
  uint32_t rxBefore = statRx;

  bool ok = sendWithAck(msg, rtt);

  bool gotMsgDuringSend = (statRx != rxBefore);

  if (ok) {
    statAckOk++;
    if (Serial) { Serial.print("-> [OK] ACK, RTT "); Serial.print(rtt); Serial.println(" ms"); }

    // Bao bang am thanh: 2 tieng CAO "tit-tit" -> biet gui duoc ma
    // khong can nhin man hinh (rat tien khi do khoang cach ngoai duong).
    // Ham nay tu bo qua neu dang bao dong.
    buzzerSendResult(true);

    if (gotMsgDuringSend) {
      // -----------------------------------------------------------------
      //  CO TIN NHAN MOI DEN TRONG LUC CHO ACK -> UUU TIEN HIEU TIN NHAN.
      //  Neu van hien "OK da gui" 3 giay thi no se CHE MAT tin nhan,
      //  ban se tuong la khong nhan duoc gi ca.
      //  RTT van duoc bao day du tren Serial Monitor.
      // -----------------------------------------------------------------
      screenMode   = SCREEN_MSG;
      modeSwitchAt = millis();
      needRedraw   = true;
    } else {
      // showTransient() chi cap nhat bien, khong ve truc tiep.
      // RTT nay la SO LIEU QUY khi ban di do khoang cach: no cho biet
      // ca chieu di va chieu ve deu thong, va do tre thuc te cua link.
      showTransient("OK da gui", "RTT " + String((long)rtt) + " ms", 3000);
    }
  } else {
    statAckFail++;
    if (Serial) Serial.println("-> [FAIL] khong nhan duoc ACK");

    // Bao bang am thanh: 3 tieng TRAM "bup-bup-bup" -> biet ngay la that bai.
    // Khac han tieng OK nen phan biet duoc chi bang tai.
    buzzerSendResult(false);

    if (gotMsgDuringSend) {
      screenMode   = SCREEN_MSG;
      modeSwitchAt = millis();
      needRedraw   = true;
    } else {
      showTransient("FAIL ko ACK", "thu lai hoac doi vi tri", 3000);
    }
  }

  // KHONG dat needRedraw = true vo dieu kien o day. Cac nhanh o tren da
  // quyet dinh man hinh nao duoc hien. Quy tac: chi mot nguon duy nhat.
}

// =====================================================================
//  BAO DONG KHAN CAP (SOS) - NON-BLOCKING
// =====================================================================

// ---------------------------------------------------------------------
//  KICH HOAT bao dong (bam phim '#')
// ---------------------------------------------------------------------
void alarmStart() {
  alarmActive   = true;
  alarmAcked    = false;
  alarmAckEver  = false;   // dot bao dong moi -> bat dau lap nhanh lai
  alarmAttempt  = 0;
  alarmStartedAt = millis();
  alarmNextTxAt = millis();      // phat ngay luot dau tien
  blinkOn       = true;
  blinkAt       = millis();

  // -----------------------------------------------------------------
  //  !!! DA SUA LOI VONG LUAN QUAN !!!
  //  Truong hop: A dang bao dong -> B nghe (alarmRxActive=true) ->
  //  nguoi B bam phim (dismissed=true) -> B bam '#' phat bao dong rieng ->
  //  bam '*' huy -> alarmActive=false NHUNG alarmRxActive van true
  //  (A con phat) -> goi lap A ke tiep keo man hinh lai SCREEN_ALARM
  //  -> ham handleKey moi lai dismissed... vong lap vo tan.
  //
  //  Fix: khi MINH bat dau phat bao dong, coi nhu da "nhin thay" canh
  //  bao dang nghe: khong cho phep manh SCREEN_ALARM cua bao dong CU
  //  (cua node A) chiem lai man hinh sau khi minh huy bao dong cua minh.
  // -----------------------------------------------------------------
  alarmRxDismissed = true;       // bao dong CU (nghe tu A) da duoc xac nhan

  screenMode  = SCREEN_ALARM;
  transientOn = false;           // bao dong che man hinh tam thoi
  needRedraw  = true;

  // Bao dong KHONG dung den tin nhan dang soan. Giu nguyen inputBuffer de
  // sau khi huy bao dong, nguoi dung van con noi dung dang go.
  if (Serial) {
    Serial.println();
    Serial.println("##########################################");
    Serial.print("#  [SOS] NODE "); Serial.print(NODE_ID);
    Serial.println(" KICH HOAT BAO DONG  #");
    Serial.println("#  Phim '*' de huy bao dong");
    Serial.println("##########################################");
  }
}

// ---------------------------------------------------------------------
//  HUY bao dong (bam phim '*')
//  Phat goi 'C' mot lan de cac node khac tat canh bao cua ho.
// ---------------------------------------------------------------------
void alarmCancel() {
  if (!alarmActive) {
    // -----------------------------------------------------------------
    //  Node nay KHONG phai nguoi phat bao dong -> '*' chi AN tam thoi.
    //
    //  Day la hanh vi CO Y, khong phai loi:
    //  Node khac van dang phat bao dong moi 4 giay (hoac 8 giay neu co
    //  repeater). Khi goi lap ke tiep toi, canh bao se HIEN LAI.
    //
    //  Ly do an toan: neu mot node bat ky co the tat canh bao cua node
    //  khac thi nguoi dung co the vo tinh (hoac co y) lam mat canh bao
    //  chay that. Chi NGUON PHAT moi duoc quyen huy, va khi huy no se
    //  broadcast goi 'C' de tat het cac node con lai.
    // -----------------------------------------------------------------
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

  // Bao cho cac node khac biet da huy, bang goi loai TYPE_CLEAR ('C').
  // Cac node nhan se tat canh bao cua ho va hien "Da het bao dong".
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
  // Tra den nen ve trang thai bat (neu dang o pha tat cua nhap nhay)
  if (lcdOk && !blinkOn) { lcd->backlight(); blinkOn = true; }
}

// ---------------------------------------------------------------------
//  Chay moi luot loop(). PHAT 1 GOI ROI TRA QUYEN NGAY.
//
//  TUYET DOI khong goi sendWithAck() o day. Ham do chan loop() ~18 giay,
//  bao dong lap 10 lan se chan ~90 giay -> ban khong the bam '*' de huy,
//  LCD khong nhap nhay duoc, va khong nhan duoc tin phan hoi.
// ---------------------------------------------------------------------
void alarmTick() {
  if (!alarmActive || !radioReady) return;

  // Chua toi luc phat goi ke tiep
  if ((long)(millis() - alarmNextTxAt) < 0) return;

  // -----------------------------------------------------------------
  //  KHONG can kiem tra "dang o giua tac vu RF khac".
  //  Ly do: ham nay duoc goi truc tiep tu loop(). Neu loop() dang bi
  //  chan boi sendMessage() (toi da ~18 giay) thi alarmTick() cung
  //  khong the chay, nen khong bao gio co 2 tac vu RF chen nhau.
  //  Khi ban bam '#' trong luc dang gui tin thuong, bao dong se doi
  //  sendMessage() ket thuc roi moi phat goi dau tien - do la hanh vi
  //  dung, vi RF chi co mot module nen bat buoc tuan tu.
  // -----------------------------------------------------------------
  mySeq = (mySeq + 1) % 1000;
  alarmLastSeq = mySeq;
  alarmAttempt++;

  // Payload ngan de on-air time nho -> nhieu co hoi phat lai hon.
  String pkt = buildPacket(DST_BROADCAST, TYPE_ALARM, 0, mySeq, "BAO CHAY");

  if (Serial) {
    Serial.print("[SOS] Phat bao dong lan "); Serial.print(alarmAttempt);
    Serial.print(" seq="); Serial.println(mySeq);
  }

  int16_t st = txPacket(pkt);
  if (st != RADIOLIB_ERR_NONE && Serial) {
    Serial.print("[SOS] phat loi ma: "); Serial.println(st);
  }

  // Dat lich lan ke tiep.
  //
  // !!! DUNG alarmAckEver, KHONG dung alarmAcked !!!
  // alarmAcked da bi reset ve false ngay TRUOC lan phat nay (de LCD hien
  // trung thuc "lan nay co ai nhan khong"). Neu dung no o day thi gap LUON
  // = ALARM_REPEAT_MS va che do lap cham de tiet kiem pin chet han.
  // alarmAckEver la bien "dinh": chi reset khi bat dau/huy bao dong, nen
  // no dung de quyet dinh chu ky lap.
  unsigned long gap = alarmAckEver ? ALARM_REPEAT_ACKED_MS : ALARM_REPEAT_MS;
  alarmNextTxAt = millis() + gap;

  needRedraw = true;   // cap nhat dem "phat xN" tren LCD
}

// ---------------------------------------------------------------------
//  Xu ly goi BAO DONG nhan tu node khac.
//  KHONG che man hinh cua chinh node dang phat bao dong.
// ---------------------------------------------------------------------
void handleAlarmPacket(const Pkt &p) {
  // -----------------------------------------------------------------
  //  CHAN GOI BAO DONG CUA CHINH MINH (bat buoc).
  //
  //  Khi co repeater: node A phat bao dong hop=0, R phat lai hop=1
  //  van giu nguyen src='A'. Node A se nhan lai chinh goi cua minh.
  //  Neu khong chan thi A se:
  //    - hien LCD "tu A lap" = tu bao dong chinh minh (gay hoang mang)
  //    - goi sendAckFor() = TU ACK CHO CHINH MINH, lang phi ~1.2s phat
  //      va co the va cham voi goi relay cua R
  //  handleDataPacket() da co guard nay (dong ~1046); ham nay cung phai co.
  // -----------------------------------------------------------------
  if (p.src == NODE_ID) return;

  // -----------------------------------------------------------------
  //  CHONG XU LY TRUNG.
  //  Khi co repeater, cung mot goi bao dong den 2 lan: ban truc tiep
  //  (hop=0) va ban R phat lai (hop=1). Khong loc thi node nay se:
  //    - tra ACK 2 lan -> ton gap doi thoi gian phat, tang nguy co va cham
  //    - in log 2 lan -> kho doc khi truy van
  //  Khoa dedup la (src, seq, type). Moi LAN LAP LAI cua bao dong co
  //  seq KHAC nhau (alarmTick() tang mySeq), nen cac lan lap van duoc
  //  xu ly binh thuong - chi ban sao trung bi chan.
  // -----------------------------------------------------------------
  if (seenBefore(p.src, p.seq, p.type)) {
    // Van cap nhat thoi gian de canh bao khong bi tat som,
    // nhung khong tra ACK lan thu hai.
    alarmRxAt = millis();
    return;
  }
  rememberPacket(p.src, p.seq, p.type);

  // -----------------------------------------------------------------
  //  Day co phai la "DOT BAO DONG MOI" khong?
  //  Moi lan lap lai cua CUNG mot bao dong co seq khac nhau, nen khong
  //  the dung seq de phan biet. Dung 2 tieu chi:
  //    (a) truoc do alarmRxActive = false (da het han hold => dot moi)
  //    (b) nguon phat doi khac
  //  Khi la dot moi -> bo trang thai "da xac nhan" de canh bao hien lai.
  //  Neu khong lam vay, mot bao dong that su MOI se bi bo qua vi nguoi
  //  dung da tung bam '*' cho bao dong cu.
  // -----------------------------------------------------------------
  bool newEpisode = (!alarmRxActive) || (alarmRxSrc != p.src);
  if (newEpisode) alarmRxDismissed = false;

  alarmRxActive = true;
  alarmRxSrc    = p.src;
  alarmRxAt     = millis();

  // Node dang tu phat bao dong thi giu man hinh cua no, chi log them.
  //
  // !!! DIEU KIEN THEM alarmRxDismissed !!!
  // Neu nguoi dung DA xac nhan canh bao nay (bam '*') thi KHONG keo ho
  // ra khoi man hinh dang go tin. Truoc day moi goi lap lai (4 giay/lan)
  // deu gat man hinh ve SCREEN_ALARM -> nguoi dung khong the go duoc tin
  // phan hoi nao, va cam giac la "ban phim bi hong".
  if (!alarmActive && !alarmRxDismissed) {
    screenMode  = SCREEN_ALARM;
    transientOn = false;
    needRedraw  = true;
  } else {
    needRedraw = true;   // cap nhat dau '!' tren man hinh soan tin
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

  // Tra ACK de node kia biet bao dong da duoc nhan.
  // Node phat se dung lai o che do lap cham (tiet kiem pin) khi thay ACK nay.
  sendAckFor(p);
}

// ---------------------------------------------------------------------
//  Xu ly goi HUY bao dong nhan tu node khac
// ---------------------------------------------------------------------
void handleClearPacket(const Pkt &p) {
  // Chan goi huy cua chinh minh (repeater phat lai van giu nguyen src).
  // Khong co guard nay thi node vua bam '*' se nhan lai goi 'C' cua
  // chinh no, in log lan thu hai va tra ACK thua.
  if (p.src == NODE_ID) return;

  // Chong xu ly trung: goi 'C' den 2 lan khi co repeater (hop 0 va hop 1).
  if (seenBefore(p.src, p.seq, p.type)) return;
  rememberPacket(p.src, p.seq, p.type);

  if (Serial) {
    Serial.println();
    Serial.print("[SOS] Node "); Serial.print(p.src);
    Serial.println(" DA HUY bao dong.");
  }

  // Chi tat canh bao NEU canh bao do la cua chinh node nay.
  // Neu node khac huy bao dong cua ho, khong duoc tat canh bao cua minh.
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

// ---------------------------------------------------------------------
//  DIEU KHIEN BUZZER - tao nhip beep khong chan loop()
// ---------------------------------------------------------------------
// Goi moi vong loop(). Tu quyet dinh bat/tat coi theo trang thai.
//
// Nguyen ly: dung millis() dem thoi gian thay vi delay().
//   - Bao dong  : keu 200ms -> nghi 300ms -> lap (nghe nhu coi hut)
//   - Het bao dong: mot tieng tram 900ms roi im
//
// VI SAO KHONG DUNG delay():
//   delay(300) se chan loop() 300ms -> trong 300ms do:
//     - khong nhan duoc goi RF (co the mat tin)
//     - khong quet duoc ban phim
//     - LCD khong nhaps nhay
//   Voi SF12, gioi han nay nghiem trong: gói bao dong chiem 1.3s tren khong.
// ---------------------------------------------------------------------
void buzzerTick() {
#if !BUZZER_ENABLED
  return;
#else
  bool anyAlarm = alarmActive || alarmRxActive;
  unsigned long now = millis();

  // =====================================================================
  //  UU TIEN 1: BAO DONG - cao nhat, cat ngang moi thu khac
  // =====================================================================
  if (anyAlarm) {
    // Dang phat chuoi ket qua ma bao dong den -> huy ngay chuoi ket qua
    if (buzzerState == 3) {
      resBeepLeft = 0;
    }
    if (buzzerState != 1) {
      buzzerState = 1;
      buzzerOn    = true;
      buzzerAt    = now;
      tone(BUZZER_PIN, BUZZER_FREQ_ALARM);   // keu ngay, khong cho
      return;
    }
    // Dang o state 1 -> tao nhip ON/OFF lien tuc
    if (buzzerOn) {
      if (now - buzzerAt >= BUZZER_ON_MS) {   // het pha keu -> tat
        noTone(BUZZER_PIN);
        buzzerOn = false;
        buzzerAt = now;
      }
    } else {
      if (now - buzzerAt >= BUZZER_OFF_MS) {  // het pha nghi -> keu lai
        tone(BUZZER_PIN, BUZZER_FREQ_ALARM);
        buzzerOn = true;
        buzzerAt = now;
      }
    }
    return;
  }

  // =====================================================================
  //  UU TIEN 2: CHUOI TIENG BAO KET QUA GUI TIN (state 3)
  //  Phat lan luot: keu -> nghi -> keu -> ... cho den khi het so tieng.
  // =====================================================================
  if (buzzerState == 3) {
    if (buzzerOn) {
      // Dang trong pha keu -> het thoi luong thi tat
      if (now - buzzerAt >= resBeepOnMs) {
        noTone(BUZZER_PIN);
        buzzerOn = false;
        buzzerAt = now;
        resBeepLeft--;                 // vua xong 1 tieng
      }
    } else {
      // Dang trong pha nghi
      if (now - buzzerAt >= resBeepGapMs) {
        if (resBeepLeft == 0) {
          // Het tieng -> ket thuc chuoi, ve trang thai im
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

  // =====================================================================
  //  UU TIEN 3: TIENG HUY BAO DONG (state 2) - 1 tieng tram roi im
  // =====================================================================
  if (buzzerState == 2) {
    if (now - buzzerAt >= BUZZER_CLEAR_MS) {
      noTone(BUZZER_PIN);
      buzzerState = 0;
      buzzerOn    = false;
      buzzerAt    = now;
    }
    return;
  }

  // =====================================================================
  //  Khong co gi -> dam bao im
  // =====================================================================
  if (buzzerState == 1) {
    // Vua het bao dong (khong con anyAlarm) -> phat tieng tram bao hieu
    noTone(BUZZER_PIN);
    tone(BUZZER_PIN, BUZZER_FREQ_CLEAR);
    buzzerState = 2;
    buzzerAt    = now;
    buzzerOn    = true;
    if (Serial) Serial.println("[BUZZER] Bao dong ket thuc -> tieng tram");
  }
#endif
}

// ---------------------------------------------------------------------
//  BAO KET QUA GUI TIN bang am thanh (chi 1 lan, khong lap)
// ---------------------------------------------------------------------
// ok = true  : 2 tieng CAO  "tit-tit"     (1800 Hz)
// ok = false : 3 tieng TRAM "bup-bup-bup" (600 Hz)
//
// KHONG lam gi neu dang bao dong -> bao dong luon thang (uu tien 1).
// ---------------------------------------------------------------------
void buzzerSendResult(bool ok) {
#if BUZZER_ENABLED && BUZZER_BEEP_RESULT
  // Bao dong dang chay -> khong cat ngang tieng coi hut
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

// ---------------------------------------------------------------------
//  BAT/TAT BUZZER NGAY LAP TUC (dung khi khoi tao / loi)
// ---------------------------------------------------------------------
void buzzerSilence() {
  noTone(BUZZER_PIN);
  buzzerState = 0;
  buzzerOn    = false;
  resBeepLeft = 0;      // huy luon chuoi tieng ket qua neu dang phat
}

// ---------------------------------------------------------------------
//  Nhap nhay den nen + tu dong het han canh bao nhan duoc.
//  Dung millis(), KHONG dung delay().
// ---------------------------------------------------------------------
void alarmBlinkTick() {
  bool anyAlarm = alarmActive || alarmRxActive;

  if (!anyAlarm) {
    // Dam bao den nen bat lai sau khi bao dong ket thuc
    if (lcdOk && !blinkOn) { lcd->backlight(); blinkOn = true; }
    return;
  }

  // Canh bao NHAN DUOC tu het han neu khong con nghe goi lap lai nua.
  // Moi lan nhan duoc goi, handleAlarmPacket() da cap nhat alarmRxAt.
  // => Chung nao ben kia con phat thi ben nay con hien canh bao.
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

// =====================================================================
//  XU LY PHIM BAM
// =====================================================================
void handleKey(char key) {
  // =================================================================
  //  UU TIEN 0: PHIM SOS. Xu ly TRUOC MOI THU.
  // =================================================================
  //  '#' = KICH HOAT bao dong chay
  //  '*' = HUY bao dong
  //
  //  PHAI nam o dau ham, truoc khoi "bam phim bat ky -> ve man hinh
  //  soan" ben duoi. Neu de sau, khoi do se keo screenMode ve
  //  SCREEN_EDIT va man hinh bao dong bi tat ngay khi bam phim.
  //
  //  GIA PHAI TRA: 2 phim '#' va '*' KHONG con dung de go ky tu trong
  //  tin nhan thuong nua. Do la danh doi chap nhan duoc vi ban phim
  //  khong du phim de vua nhap lieu vua co nut khan cap.
  // =================================================================
  if (key == '#') {
    if (alarmActive) {
      // Node nay DANG phat bao dong -> bam nua chi la bam nham.
      // LCD phai noi ra ly do: node chay pin ngoai hien truong khong co
      // Serial Monitor, neu chi in Serial thi nguoi dung tuong phim hong.
      alarmNotice      = "Dang bao dong";
      alarmNoticeUntil = millis() + ALARM_NOTICE_MS;
      if (Serial) Serial.println("[SOS] Bao dong dang chay. Bam '*' de huy.");
      needRedraw = true;
    } else {
      // -----------------------------------------------------------------
      //  !!! DA SUA LOI NGHIEP VU !!!
      //  Ban cu: if (alarmActive || alarmRxActive) -> chan, chi bao
      //  "Dang bao dong". Nghia la khi node B NGHE thay bao chay tu node A,
      //  nguoi o node B KHONG THE phat bao dong cua chinh minh.
      //
      //  Do la hanh vi SAI ve mat nghiep vu: tinh huong chay lan ra,
      //  nguoi o node B cung can bao cho nguoi khac biet.
      //  Nen: chi chan khi node nay DA la nguon phat. Neu moi chi la
      //  nguoi nghe thi cho phep phat bao dong cua rieng minh.
      //
      //  alarmStart() se set alarmActive=true; luu y no KHONG xoa
      //  alarmRxActive, nen dau '!' canh bao cu van con hien.
      // -----------------------------------------------------------------
      if (alarmRxActive && Serial) {
        Serial.println("[SOS] Node nay cung phat bao dong (truoc do dang nghe tu node khac).");
      }
      alarmStart();
    }
    return;
  }

  if (key == '*') {
    if (alarmActive) {
      // Node nay LA nguon phat -> huy that su, broadcast goi 'C'
      alarmCancel();
    } else if (alarmRxActive) {
      // -----------------------------------------------------------------
      //  Node nay chi la NGHE bao dong tu node khac.
      //
      //  '*' KHONG the tat bao dong (chi nguon phat moi huy duoc), nhung
      //  no XAC NHAN: nguoi dung da thay canh bao, muon quay ve go tin
      //  phan hoi. Danh dau alarmRxDismissed de cac goi lap ke tiep
      //  (4 giay/lan) khong keo man hinh ve lai SCREEN_ALARM nua.
      //
      //  Canh bao VAN CHUA tat: dau '!' con hien tren man hinh soan tin,
      //  den nen van nhap nhay. Khi nguon phat huy that (goi 'C') hoac
      //  het han hold thi moi tat han.
      // -----------------------------------------------------------------
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
      // Khong co bao dong nao -> nhac nguoi dung phim nay dung de lam gi.
      // Truong hop nay transient hien duoc vi screenMode != SCREEN_ALARM.
      if (Serial) Serial.println("[SOS] Khong co bao dong nao de huy.");
      if (lcdOk) showTransient("Ko co bao dong", "'#'=SOS '*'=huy", 1500);
    }
    return;
  }

  // -----------------------------------------------------------------
  //  DANG BAO DONG -> HANH VI PHU THUOC VAI TRO CUA NODE
  // -----------------------------------------------------------------
  //  !!! DA SUA LOI THAT SU !!!
  //  Ban cu: guard chan moi phim khi (alarmActive || alarmRxActive).
  //  Hau qua tren node chi NGHE bao dong: nguoi dung bi khoa ban phim,
  //  khong go duoc tin phan hoi (VD "da so tan", "lua o huong dong") -
  //  dung vao luc can nhat. Va cam giac luc do la "ban phim bi hong".
  //
  //  SUA: chi khoa khi node nay LA NGUON PHAT (alarmActive). Vi khi do
  //  moi luot loop() phai uu tien phat goi bao dong, khong nen de nguoi
  //  dung chen tin thuong vao.
  //  Neu chi la NGHE (alarmRxActive) -> cho phep dung binh thuong, chi
  //  danh dau canh bao tren man hinh soan tin.
  // -----------------------------------------------------------------
  if (alarmActive) {
    if (key == 'D') {
// Vong 4 man hinh khi DANG BAO DONG:
      //   ALARM -> MSG -> STATS -> LINK -> quay ve ALARM
      // Dam bao nguoi dung LUON tro lai duoc manh canh bao.
      // PHAI cap nhat khi them man hinh moi - neu khong, man LINK se
      // bi bo qua hoac khong quay ve duoc ALARM.
      switch (screenMode) {
        case SCREEN_MSG:   screenMode = SCREEN_STATS; break;
        case SCREEN_STATS: screenMode = SCREEN_LINK;  break;
        case SCREEN_LINK:  screenMode = SCREEN_ALARM; break;
        default:           screenMode = SCREEN_MSG;   break;  // tu ALARM
      }

      modeSwitchAt = millis();
      // Reset cuon ve dau khi quay lai man hinh tin nhan. Neu khong, tin da
      // cuon het o luot truoc se hien ra dung o KY TU CUOI -> tuong tin bi cat.
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
    needRedraw = true;   // ve lai man hinh bao dong
    return;
  }

  // -----------------------------------------------------------------
  //  NGUOI DUNG BAM PHIM = uu tien cao nhat.
  //  Huy man hinh tam thoi de ky tu vua bam hien ra NGAY LAP TUC.
  //
  //  Vi sao can: redraw() uu tien transientOn, nen neu khong huy o day
  //  thi khi ban bam phim trong luc "Tin qua dai" dang hien, ky tu se
  //  KHONG len man hinh cho toi khi transient het gio (~900ms).
  //  Cam giac khi do la "ban phim bi tre/hong" - rat kho hieu.
  //  Ngoai le: phim 'E' duoc xu ly rieng ben duoi vi no co the tao
  //  transient moi (bao "Chua co noi dung").
  // -----------------------------------------------------------------
  if (key != 'E' && transientOn) {
    transientOn = false;
  }

// -----------------------------------------------------------------
  //  PHIM 'D': VONG QUA 4 MAN HINH.
  //  PHAI xu ly TRUOC khoi "bam phim bat ky -> ve man hinh soan" ben
  //  duoi, vi khoi do keo screenMode ve 0 va 'D' se khong bao gio
  //  chuyen duoc sang man hinh thong so.
  //
  //  Vong: SOAN -> TIN NHAN -> THONG SO -> LIEN KET -> SOAN -> ...
  //    SOAN     : go tin (tin chiem ca 2 dong)
  //    TIN NHAN : doc tin cuoi (tin chiem ca 2 dong)
  //    THONG SO : RSSI/SNR + bo dem TX/RX
  //    LIEN KET : hop/seq (chan doan repeater) + ACK ok/fail
  //
  //  Ly do tach THONG SO va LIEN KET lam 2 man: 4 bo dem x 3 chu so
  //  = 12 chu so + 4 nhan = 16 ky tu -> tran cot neu gop chung
  //  (da mo phong va phat hien truong hop tran that).
  // -----------------------------------------------------------------
  if (key == 'D') {
    switch (screenMode) {
      case SCREEN_EDIT:  screenMode = SCREEN_MSG;   break;
      case SCREEN_MSG:   screenMode = SCREEN_STATS; break;
      case SCREEN_STATS: screenMode = SCREEN_LINK;  break;
      case SCREEN_LINK:  screenMode = SCREEN_EDIT;  break;
      default:           screenMode = SCREEN_MSG;   break;   // tu ALARM
    }

    modeSwitchAt = millis();
    // Reset cuon ve dau khi quay lai man hinh tin nhan. Neu khong, tin da
    // cuon het o luot truoc se hien ra dung o KY TU CUOI -> tuong tin bi cat.
    msgScrollPos = 0;
    msgScrollAt  = millis();
    needRedraw = true;
    return;
  }

  // Bam phim bat ky (khac 'D') khi dang xem tin -> ve man hinh soan.
  if (screenMode != SCREEN_EDIT) {
    // -----------------------------------------------------------------
    //  Neu dang o man hinh bao dong do NHAN tu node khac, thi viec nguoi
    //  dung bam phim = ho da thay va muon lam viec khac.
    //  Danh dau alarmRxDismissed de cac goi lap lai (4-20 giay/lan) khong
    //  keo man hinh ve SCREEN_ALARM nua.
    //
    //  !!! NEU THIEU DONG NAY !!!
    //  Nguoi dung bam phim -> ve man soan tin -> 4 giay sau goi lap cua
    //  node A toi -> man hinh bi giat lai ve "!! BAO CHAY !!" -> khong the
    //  go duoc tin nao. Day chinh la nguyen nhan khien node B "bam khong an".
    // -----------------------------------------------------------------
    if (screenMode == SCREEN_ALARM && alarmRxActive && !alarmActive) {
      alarmRxDismissed = true;
    }
    screenMode = SCREEN_EDIT;
    needRedraw = true;
  }

  if (key == 'E') {               // ENTER = GUI
    if (inputBuffer.length() == 0) {
      // KHONG dung delay(900) o day: delay se chan loop(), khien
      // handleIncoming() khong chay nen radio "diec" suot 900ms.
      showTransient("Chua co noi dung", "", 900);
      return;
    }
    String outgoing = inputBuffer;
    inputBuffer = "";
    sendMessage(outgoing);
    return;
  }

  if (key == 'U') {               // xoa lui 1 ky tu
    if (inputBuffer.length() > 0) inputBuffer.remove(inputBuffer.length() - 1);
    needRedraw = true;
    return;
  }

  if (key == 'X') {               // xoa het
    inputBuffer = "";
    needRedraw = true;
    return;
  }

  // -----------------------------------------------------------------
  //  MOI KY TU CON LAI DEU LA PHIM NHAP LIEU
  // -----------------------------------------------------------------
  // Khong duoc "nuot" phim o day. Ban cu cua code co mot khoi:
  //     if (key=='A'||key=='B'||key=='L'||key=='R'||key=='#') return;
  // va no lam HONG ban phim: hang 1 (A B # *) khong ra chu nao,
  // hang 5 mat nut dau (L) va nut thu ba (R).
  //
  // Quy uoc hien tai - rat de nho khi nhin ban phim:
  //     COT 4       = 4 phim chuc nang: U(xoa lui) D(xem) X(xoa het) E(gui)
  //     16 phim con lai = nhap chu, gom A B # * L R va 0-9
  // -----------------------------------------------------------------
  if (inputBuffer.length() < MAX_MSG_LEN) {
    inputBuffer += key;
    needRedraw = true;
  } else {
    // KHONG delay(900): giu loop() chay de radio van nhan duoc tin.
    showTransient("Tin qua dai", String(MAX_MSG_LEN) + " ky tu max", 900);
  }
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) delay(10);
  delay(200);

  Serial.println();
  Serial.println("==========================================");
  Serial.print("  LORA TERMINAL  ID='"); Serial.print(NODE_ID);
  Serial.println("'  (v6 co SOS)");
  // DAU VAN TAY PHIEN BAN. In ra de biet chac chan board dang chay
  // ban code nao. Neu 2 board in 2 thoi diem khac nhau -> chung dang
  // chay 2 ban khac nhau (mot board chua duoc nap lai).
  Serial.print("  Firmware build: ");
  Serial.print(BUILD_DATE); Serial.print(" "); Serial.println(BUILD_TIME);
  Serial.println("==========================================");

  // -----------------------------------------------------------------
  //  LCD I2C
  //  BAC 1: setPins() TRUOC. Bat buoc. Wire mac dinh SCL = index 23 =
  //  P0.11 = DIO1 cua LoRa. Neu de Wire.begin() chay truoc, ngat nhan
  //  tin chet va ban se tuong la loi khoang cach.
  // -----------------------------------------------------------------
  Wire.setPins(LCD_SDA_IDX, LCD_SCL_IDX);   // (SDA, SCL) - SDA truoc
  Wire.begin();

  Serial.print("[I2C] setPins SDA=idx"); Serial.print(LCD_SDA_IDX);
  Serial.print("(P0.31/D21) SCL=idx");   Serial.print(LCD_SCL_IDX);
  Serial.println("(P0.29/D20)");

  // Quet tim dia chi LCD (thuong la 0x27 hoac 0x3F)
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
    lcd->init();          // ben trong goi Wire.begin() lai - vo hai
    lcd->backlight();
    lcdOk = true;

    lcdStatus(String("Node '") + NODE_ID + "' dang khoi dong", "SX1262...");
  } else {
    Serial.println("KHONG thay LCD!");
    Serial.println("  -> Kiem tra: SDA=D21, SCL=D20, VCC=3.3V, GND chung");
    Serial.println("  -> Van chay binh thuong, chi khong co man hinh.");
  }

  // -----------------------------------------------------------------
  //  BAN PHIM
  // -----------------------------------------------------------------
  // Mo khoa NFC TRUOC khi cau hinh chan hang, vi hang 3 (P0.10) va
  // hang 4 (P0.09) nam tren 2 chan NFC. Ham nay co the lam may reset
  // 1 lan o lan chay dau tien.
  ensureNfcPinsAsGpio();

initKeypad();
  Serial.println("[OK] Phim 5x4: Cot D6/D9/D19/D18");
  Serial.println("                Hang D15/D14/D8/D10/D0  (hang1 -> hang5)");
  Serial.print("                Hang 3 o "); Serial.println(ROW3_LABEL);

#if BUZZER_ENABLED
  // -----------------------------------------------------------------
  //  BUZZER tren P0.10 (Feather index 2)
  //  !!! DUNG index 2, KHONG phai 16 !!!
  //  tone(2) -> g_ADigitalPinMap[2] = 10 = P0.10  (SuperMini D16) DUNG
  //  tone(16)-> g_ADigitalPinMap[16]= 30 = P0.30  (chan khac)     SAI
  // -----------------------------------------------------------------
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerSilence();
  Serial.print("[OK] Buzzer: P0.10 (SuperMini D16), index ");
  Serial.print(BUZZER_PIN);
  Serial.print(", bao dong ");
  Serial.print(BUZZER_FREQ_ALARM);
  Serial.print(" Hz / het ");
  Serial.print(BUZZER_FREQ_CLEAR);
  Serial.println(" Hz");

  // Beep ngan bao khoi dong xong - biet ngay buzzer hoat dong
  tone(BUZZER_PIN, 2000, 150);
  delay(250);
#else
  Serial.println("[--] Buzzer: TAT (BUZZER_ENABLED = 0)");
#endif

#if KEYPAD_SELFTEST
  // Do dien tung chan HANG + chan D16 cu de xac nhan chan nao chet.
  testRowPinsElectrical();
  Serial.println();

  // Do dien tung chan COT. BUOC NAY RAT QUAN TRONG voi loi cua node B:
  // 3 phim F2/#/* chet cung luc la dau hieu nghi van COT (D9/D19/D18).
  // testRowPinsElectrical() khong the phat hien duoc dieu nay.
  testColPinsElectrical();
  Serial.println();

  // DUNG LAI O DAY de chan doan ban phim (tu thoat sau 30 giay).
  // Chua khoi tao LoRa nen khong co song vo tuyen nao bi phat ra.
  keypadSelfTest();
#endif

  // -----------------------------------------------------------------
  //  LORA
  // -----------------------------------------------------------------
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
    // -----------------------------------------------------------------
    //  Hien DAU VAN TAY PHIEN BAN ngay tren LCD, khong can laptop.
    //  __TIME__ dang "HH:MM:SS" = 8 ky tu; "Node 'B'" = 8 ky tu.
    //  Ca hai vua khop 16 cot.
    //
    //  CACH DUNG: sau khi nap, nhin LCD node B. Neu gio build KHAC voi
    //  node A (hoac khac voi luc ban vua bien dich) => node B dang chay
    //  FIRMWARE CU, chua duoc nap lai. Do chinh la ly do 3 phim van chet.
    // -----------------------------------------------------------------
    lcdStatus(String("Node '") + NODE_ID + "'", String(BUILD_TIME));
    delay(3000);
    lcdStatus(String("Node '") + NODE_ID + "' san sang", "RF OK  SF" + String(LORA_SF));
    delay(1500);
  }

  radio.setDio1Action(setFlag);   // gan ngat SAU CUNG, tranh glitch luc reset
  if (enterRx()) Serial.println("[READY] Dang lang nghe...");

  needRedraw = true;
  redraw();
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  // 0. KIEM TRA PHIM KHAN CAP TRUC TIEP - KHONG DOI scanKeypad
  //    Day la sua doan triet de cho loi "bam * khong an khi dang bao dong".
  //
  //    Van de: bam # xong, alarmTick phat goi lap moi 8 giay, moi lan
  //    phat BLOCK loop ~4.4 giay (CAD 3s + transmit 1.3s). Bam * vao
  //    khoang do -> scanKeypad khong chay -> phim mat -> "khong tat duoc".
  //    Fix repeat 600ms chi giup khi GIU phim; bam nhanh van co the mat.
  //
  //    Giai phap nay doc 2 phim khuj cap (*) truc tiep tu GPIO o dau
  //    moi vong loop - ke ca vua quay ra tu khoang block.
  //    Chi kich hoat KHI co bao dong dang chay.
  if (alarmActive || alarmRxActive) {
    // Doc truc tiep trang thai GPIO cua phim * va #
    // * = HANG 1 COT 4  (D18/P1.15); # = HANG 1 COT 3 (D19/P0.02)
    // Kiem tra: keo cot tuong ung xuong LOW, doc hang 1 (D15/P1.13)
    nrf_gpio_pin_clear(colPins[3]);            // cot 4 = phim *
    delayMicroseconds(20);
    bool starPressed = (nrf_gpio_pin_read(rowPins[0]) == 0);
    nrf_gpio_pin_set(colPins[3]);

    nrf_gpio_pin_clear(colPins[2]);            // cot 3 = phim #
    delayMicroseconds(20);
    bool hashPressed = (nrf_gpio_pin_read(rowPins[0]) == 0);
    nrf_gpio_pin_set(colPins[2]);

    // Chi xu ly khi phim vua duoc nhan (canh xuong), tranh lap
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

  // 1. Nhan song RF
  if (radioReady) handleIncoming();

  // 1b. BAO DONG: phat goi lap lai theo kieu TICH (khong chan loop).
  //     Dat SAU handleIncoming() de tin nhan duoc uu tien xu ly truoc,
  //     va TRUOC khi quet ban phim de phim '*' luon duoc phuc vu.
alarmTick();
  alarmBlinkTick();
  buzzerTick();   // 1c. BUZZER: tao nhip beep, khong chan loop

  // 2. Quet ban phim
  //    !!! DA SUA LOI "BAM KHONG AN" KHI LOOP BI CHAN ~5 GIAY !!!
  //
  //    Cach quet cu: phim chi duoc nhan khi (key != lastKey).
  //    Van de: khi loop() bi chan boi alarmCancel()/txPacket() (~5 giay),
  //    phim bam trong khoang do KHONG BAO GIO duoc quet.
  //
  //    Fix: giu phim duoc phep LAP LAI khi giu du lau hon debounce.
  //    Chi '*' va '#' duoc lap (tranh go chu lap vo tan).
  //
  //    !!! DA SUA LOI HOI QUI "KHONG GUI DUOC TIN" !!!
  //    Ban truoc co lenh return ngay tai day -> loop() bi THOAT SOM,
  //    bo qua buoc 3 (Serial), 4 (cuon), 5, 6 (ve LCD), 7.
  //    Khi nguoi dung GIU phim so (go tin), loop bi cat lien tuc
  //    -> tin go xong bam E cung KHONG GUI (Serial/doc tin khong chay).
  //    Thay return bang: chi bo qua handleKey, khong thoat loop.
  char key = scanKeypad();
  if (key != '\0' && (key != lastKey || (millis() - lastDebounceTime > 600))) {
    bool isRepeatHold = (key == lastKey);   // giu phim (khong phai phim moi)

    if (key != lastKey) {
      lastDebounceTime = millis();
    } else if (key == '*' || key == '#') {
      // Giu phim chuc nang: cho phep lap sau 600ms
      lastDebounceTime = millis();
    } else {
      // Giu phim chu/so: KHONG lap (tranh go "111111")
      // Chi cap nhat lastKey, KHONG goi handleKey,
      // va KHONG return (loop phai chay tiep cac buoc duoi!)
      lastKey = key;
    }

    if (!isRepeatHold || key == '*' || key == '#') {
      lastKey = key;
      handleKey(key);
    }
  }
  if (key == '\0') lastKey = '\0';

  // 3. Go tu Serial Monitor USB de test ma khong can bam phim
  //    'sos' = kich hoat bao dong, 'huy' = tat bao dong.
  //    Chuyen nay rat huu ich khi test bang laptop ma khong co ban phim.
  //    Luu y khi bam '*' bi "khong an": see alarmCancel() - phat goi
  //    'C' ton toi da ~5 giay (CAD cho 3s + transmit 2.1s). Trong 5 giay
  //    do loop() bi chan -> phim bam se bi MAT (khong duoc doc).
  //    Bam * roi PHAI DOI 5-6 giay truoc khi bam lai.
  if (Serial.available() > 0) {
    // !!! QUAN TRONG: giam timeout cua readStringUntil !!!
    // Mac dinh Arduino = 1000ms cho moi lan doc.
    // Neu web_server lien tuc gui du lieu rac (hoac DTR lam buffer co
    // du lieu le), moi lan loop bi chan them 1000ms -> phim mat nhay.
    // Dat 50ms: du doc nhanh, khong anh huong ban phim.
    Serial.setTimeout(50);
    String m = Serial.readStringUntil('\n');
    m.trim();
    if (m.length() > MAX_MSG_LEN) m = m.substring(0, MAX_MSG_LEN);
    if (m.length() == 0) {
      // chuoi rong -> bo qua
    } else if (m.equalsIgnoreCase("sos")) {
      if (alarmActive) {
        if (Serial) Serial.println("[SOS] Bao dong dang chay. Go 'huy' de tat.");
      } else {
        alarmStart();
      }
    } else if (m.equalsIgnoreCase("huy") || m.equalsIgnoreCase("clear")) {
      alarmCancel();
    } else if (alarmActive) {
      // -----------------------------------------------------------------
      //  CHAN GUI TIN THUONG khi NODE NAY DANG PHAT BAO DONG.
      //  sendMessage() chan loop() ~18 giay. Neu cho no chay trong luc
      //  bao dong thi alarmTick() bi tre ~18 giay -> bao dong mat tac
      //  dung lap lai. handleKey() da chan duong ban phim, day la chan
      //  duong Serial Monitor cho dong bo.
      // -----------------------------------------------------------------
      if (Serial) {
        Serial.println("[SOS] Dang bao dong - tu choi gui tin thuong.");
        Serial.println("      Go 'huy' de tat bao dong roi moi gui tin.");
      }
    } else {
      sendMessage(m);
    }
  }

// 4. CUON TIN NHAN DAI (tinh truoc khi ve, de ve ngay trong luot nay)
  //    !!! DA SUA CHO CUON 2 DONG !!!
  //    Chi chay khi dang o man hinh tin nhan VA tin dai hon suc chua.
  //    KHONG chay khi man hinh tam thoi dang hien: neu khong, msgScrollPos
  //    se tien am am sau lung transient, va khi transient het gio thi tin
  //    nhan hien ra GIUA CHUNG thay vi tu dau -> ban doc thieu phan dau.
  //
  //    Cong thuc da MO PHONG truoc khi viet (khong tran o moi do dai 1..32):
  //      CAP    = MSG_VISIBLE - 3 = 30 - 3 = 27  (3 cot dem khoang trang)
  //      maxPos = 3 + L - 30
  //    msgNeedScroll duoc dat = (L > CAP) trong deliverToUser().
  if (screenMode == SCREEN_MSG && msgNeedScroll && lcdOk && !transientOn) {
    if (millis() - msgScrollAt >= SCROLL_STEP_MS) {
      msgScrollAt = millis();
      // Vi tri cuon toi da: sao cho MSG_VISIBLE_RX ky tu CUOI cua tin vua khop.
      uint8_t maxPos = (uint8_t)(MSG_SCROLL_PAD + lastMsg.length()
                                 - MSG_VISIBLE_RX);
      // msgNeedScroll chi true khi L > 24, va MSG_VISIBLE_RX = 27,
      // nen bieu thuc luon duong -> khong lo tran so unsigned.
// msgNeedScroll chi true khi L > 27, va MSG_VISIBLE = 30,
      // nen bieu thuc luon duong -> khong lo tran so unsigned.
      if (msgScrollPos < maxPos) {
        msgScrollPos++;
        needRedraw = true;
      }
    }
  }

  // 5. MAN HINH TAM THOI: tu tat khi het gio.
  //    PHAI co buoc nay. redraw() chi chay khi needRedraw == true,
  //    nen neu khong kiem tra o day thi man hinh "OK da gui" se
  //    TREO MAI MAI vi khong co gi dat lai needRedraw.
  if (transientOn && lcdOk && (long)(millis() - transientUntil) > 0) {
    transientOn = false;
    needRedraw  = true;
  }

  // 6. Ve lai man hinh
  if (needRedraw) redraw();

  // 7. TU DONG QUAY VE MAN HINH SOAN - DA TAT (AUTO_BACK_MS = 0)
  //    Theo yeu cau: tin nhan va thong so O LAI MAI MAI tren LCD,
  //    chi bien mat khi ban bam phim.
  //
  //    !!! CANH BAO KHI SUA LAI CHO NAY !!!
  //    Dung gio AUTO_BACK_MS ve mot so khac 0 ma khong sua logic duoi.
  //    Voi AUTO_BACK_MS = 0, dieu kien "millis() - modeSwitchAt > backAfter"
  //    se DUNG NGAY LAP TUC (vi backAfter = 0) -> man hinh tin nhan bi
  //    tat tuc thi, con te hon ca khi chua sua. Nen phan nay duoc
  //    bao boc bang #if de no hoan toan khong duoc bien dich khi = 0.
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



