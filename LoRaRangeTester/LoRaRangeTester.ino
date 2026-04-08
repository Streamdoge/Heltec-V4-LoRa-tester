// ============================================================================
//  LoRaRangeTester.ino
//  Прошивка для тестирования дальности LoRa
//  Плата: Heltec WiFi LoRa 32 V4 (ESP32-S3 + SX1262)
//
//  Одна прошивка — две роли: SENDER / RECEIVER
//  Роль выбирается через веб-интерфейс и сохраняется в NVS.
//
//  В Arduino IDE выбирать плату: "WiFi LoRa 32(V3)"
//  Upload Mode: USB-OTG-CDC, USB CDC On Boot: Enabled
// ============================================================================

#include <RadioLib.h>
#include "Arduino.h"
#include <SSD1306Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// ─── Версия прошивки ─────────────────────────────────────────────────────────
#define FW_VERSION "1.0.0"

// ─── Конфигурация устройства ─────────────────────────────────────────────────
// Менять DEVICE_NAME для каждого устройства (становится SSID точки доступа)
#define DEVICE_NAME   "LoRa-02"
#define WIFI_PASSWORD "1223334444"

// ─── Роли ────────────────────────────────────────────────────────────────────
#define ROLE_SENDER   0
#define ROLE_RECEIVER 1

// ─── Параметры LoRa по умолчанию ─────────────────────────────────────────────
#define DEFAULT_FREQUENCY  868000000UL   // 868 МГц (EU)
#define DEFAULT_SF         9             // SF9
#define DEFAULT_BW         0             // 0=125kHz, 1=250kHz, 2=500kHz
#define DEFAULT_CR         1             // 1=4/5, 2=4/6, 3=4/7, 4=4/8
#define DEFAULT_PREAMBLE   8
#define DEFAULT_TX_POWER   20            // dBm (макс 22 для V4)
#define DEFAULT_INTERVAL   5000UL        // мс

#define BUFFER_SIZE                50

// ─── GPS пины — Quectel L76K, GNSS-разъём P3 (SH1.25-8pin) ──────────────────
// Источник: Meshtastic firmware для Heltec V4 + схема платы.
// VGNSS_Ctrl → Q7 (AO3401A, P-channel): LOW = GPS запитан, HIGH = GPS обесточен.
// GPIO21 (RST_GPS) совпадает с RST_OLED — не трогаем, держим HIGH (не в сбросе).
#define GPS_RX_PIN     39    // ESP32 UART1 RX ← GPS TX (подтверждено: Meshtastic V4)
#define GPS_TX_PIN     38    // ESP32 UART1 TX → GPS RX (подтверждено: Meshtastic V4)
#define GPS_RST_PIN    21    // GPS Reset (= RST_OLED, не используем активно)
#define VGNSS_CTRL_PIN 34    // VGNSS_Ctrl P-MOS: LOW = GPS запитан (подтверждено: Meshtastic V4)

// ─── Батарея ─────────────────────────────────────────────────────────────────
#define BAT_ADC_PIN    1     // GPIO1 (ADC1_CH0): батарея через делитель 100кΩ+390кΩ (ratio 4.9)
                             // Heltec V4: Vbat = (raw/4095) × 3.3В × 4.9. Диапазон Li-Ion: 3.0-4.2В.
#define BAT_ADC_CTRL   37    // GPIO37: управление делителем батареи (HIGH = включён, LOW = выкл)

// ─── Лимиты буферов ──────────────────────────────────────────────────────────
#define MAX_SERIAL_LOG_LINES 500
#define MAX_CSV_LOG_LINES    1000

// ════════════════════════════════════════════════════════════════════════════
//  ГЛОБАЛЬНОЕ СОСТОЯНИЕ
// ════════════════════════════════════════════════════════════════════════════

// Настройки (загружаются из NVS, namespace "lora-cfg")
uint8_t  currentRole    = ROLE_SENDER;
uint32_t loraFrequency  = DEFAULT_FREQUENCY;
uint8_t  loraSF         = DEFAULT_SF;
uint8_t  loraBW         = DEFAULT_BW;
uint8_t  loraCR         = DEFAULT_CR;
uint32_t loraPreamble   = DEFAULT_PREAMBLE;
uint8_t  txPower        = DEFAULT_TX_POWER;
uint32_t txInterval     = DEFAULT_INTERVAL;

// Рабочее состояние
bool     active         = false;   // передача или приём запущены
bool     lora_idle      = true;    // радио свободно (для Sender)
int16_t  lastRssi       = 0;
int8_t   lastSnr        = 0;
uint32_t packetCount    = 0;
uint32_t lastPacketNum  = 0;  // номер последнего пакета из содержимого (для расчёта потерь)
char     rxpacket[BUFFER_SIZE];
char     txpacket[BUFFER_SIZE];
unsigned long lastTxTime = 0;

// GPS
HardwareSerial gpsSerial(1);
TinyGPSPlus    gps;
unsigned long  lastGpsLogTime = 0;
bool           gpsEnabled     = true;   // false = GPS обесточен (P-MOS HIGH)

// Буферы логов
String serialLog = "";
String csvLog    = "";   // заголовок устанавливается в loadSettings()
uint16_t csvRowCount = 0; // кол-во строк данных в csvLog (без заголовка)

unsigned long gpsInitTime = 0;  // момент инициализации GPS UART (для детекции Error)
bool btnSleepFired  = false;    // true после пробуждения из deep sleep — подавляет лишние действия кнопки

// Кэш батареи — обновляется раз в 30с, не нагружает АЦП при каждом updateDisplay()
float         cachedBatVoltage = 0.0f;
unsigned long lastBatReadTime  = 0;

// ─── Объекты ─────────────────────────────────────────────────────────────────
// SX1262: NSS=SS(8), DIO1=DIO0(14), RST=RST_LoRa(12), BUSY=BUSY_LoRa(13)
SX1262 radio = new Module(SS, DIO0, RST_LoRa, BUSY_LoRa);
volatile bool radioFlag = false;
bool txInProgress = false;

void IRAM_ATTR setRadioFlag() { radioFlag = true; }

WebServer   server(80);
Preferences preferences;

// Дисплей 128×64 (GEOMETRY_128_64 — отличие от Wireless Stick V3 с 64×32)
static SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED, GEOMETRY_128_64);

// ─── Прототипы ───────────────────────────────────────────────────────────────
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void updateDisplay();
void updateLoRaSettings();
void reinitLoRaForRole();
void stopActive();
String getGPSString();
void trimCSVLog();
void resetSession();


// ════════════════════════════════════════════════════════════════════════════
//  TASK 3 — СИСТЕМА ЛОГИРОВАНИЯ
// ════════════════════════════════════════════════════════════════════════════

// Добавить строку в serialLog с меткой времени [Xs].
// Лимит MAX_SERIAL_LOG_LINES — старые строки вытесняются.
void addToSerialLog(const String& message) {
  String line = "[" + String(millis() / 1000) + "s] " + message;

  serialLog += (serialLog.length() > 0 ? "\n" : "") + line;
  Serial.println(line);

  // Обрезаем до MAX_SERIAL_LOG_LINES
  int count = 0, cutIdx = 0;
  for (int i = serialLog.length() - 1; i >= 0; i--) {
    if (serialLog.charAt(i) == '\n') {
      count++;
      if (count >= MAX_SERIAL_LOG_LINES) { cutIdx = i + 1; break; }
    }
  }
  if (cutIdx > 0) serialLog = serialLog.substring(cutIdx);
}

// Добавить строку в CSV (Sender): Time,Packet,RSSI,Data,Lat,Lon,Satellites
// gps — строка вида "55.7558,37.6176,8" или "0.000000,0.000000,0"
void addToCSVLogSender(uint32_t pkt, int16_t rssi, const char* data, const String& gps) {
  csvLog += String(millis() / 1000) + "," + String(pkt) + "," +
            String(rssi) + "," + String(data) + "," + gps + "\n";
  csvRowCount++;
  trimCSVLog();
}

// Добавить строку в CSV (Receiver): Time,Packet,RSSI,SNR,Data,Lat,Lon,Satellites
void addToCSVLogReceiver(uint32_t pkt, int16_t rssi, int8_t snr, const char* data, const String& gps) {
  csvLog += String(millis() / 1000) + "," + String(pkt) + "," +
            String(rssi) + "," + String(snr) + "," + String(data) + "," + gps + "\n";
  csvRowCount++;
  trimCSVLog();
}

// Обрезаем CSV: O(1) проверка по счётчику, O(n) только при реальном переполнении
void trimCSVLog() {
  if (csvRowCount <= MAX_CSV_LOG_LINES) return;

  // Находим конец заголовка
  int headerEnd = csvLog.indexOf('\n');
  if (headerEnd < 0) return;
  String header = csvLog.substring(0, headerEnd + 1);

  // Убираем самую старую строку данных
  int firstNl = csvLog.indexOf('\n', headerEnd + 1);
  if (firstNl >= 0) {
    csvLog = header + csvLog.substring(firstNl + 1);
    csvRowCount--;
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  TASK 4 — ЗАГРУЗКА И СОХРАНЕНИЕ НАСТРОЕК (NVS)
// ════════════════════════════════════════════════════════════════════════════

// Сброс счётчиков пакетов и CSV-лога под текущую роль.
// Вызывается при смене роли, очистке лога и старте.
void resetSession() {
  packetCount = 0; lastPacketNum = 0; lastRssi = 0; lastSnr = 0;
  csvRowCount = 0;
  csvLog = (currentRole == ROLE_SENDER)
    ? "Time,Packet,RSSI,Data,Lat,Lon,Satellites\n"
    : "Time,Packet,RSSI,SNR,Data,Lat,Lon,Satellites\n";
}

void loadSettings() {
  preferences.begin("lora-cfg", true);
  currentRole   = preferences.getUChar("role",      ROLE_SENDER);
  loraFrequency = preferences.getUInt("frequency",  DEFAULT_FREQUENCY);
  loraSF        = preferences.getUChar("sf",        DEFAULT_SF);
  loraBW        = preferences.getUChar("bw",        DEFAULT_BW);
  loraCR        = preferences.getUChar("cr",        DEFAULT_CR);
  loraPreamble  = preferences.getUInt("preamble",   DEFAULT_PREAMBLE);
  txPower       = preferences.getUChar("power",     DEFAULT_TX_POWER);
  txInterval    = preferences.getUInt("interval",   DEFAULT_INTERVAL);
  gpsEnabled    = preferences.getBool("gps_en",     true);
  preferences.end();

  resetSession();  // устанавливает CSV-заголовок и обнуляет счётчики

  // Лог загруженных настроек
  String bwStr = (loraBW == 0) ? "125" : (loraBW == 1) ? "250" : "500";
  String msg = "FW v" + String(FW_VERSION) +
               " | Frequency: " + String(loraFrequency / 1000000.0, 3) + " MHz" +
               " | SF:" + loraSF +
               " | BW:" + bwStr + "kHz" +
               " | CR:4/" + (loraCR + 4) +
               " | Preamble:" + loraPreamble;
  if (currentRole == ROLE_SENDER)
    msg += " | Power:" + String(txPower) + "dBm | Interval:" + String(txInterval / 1000.0, 1) + "s";
  addToSerialLog(msg);
  addToSerialLog("Role: " + String(currentRole == ROLE_SENDER ? "SENDER" : "RECEIVER"));
}

void saveSettings() {
  preferences.begin("lora-cfg", false);
  preferences.putUChar("role",     currentRole);
  preferences.putUInt("frequency", loraFrequency);
  preferences.putUChar("sf",       loraSF);
  preferences.putUChar("bw",       loraBW);
  preferences.putUChar("cr",       loraCR);
  preferences.putUInt("preamble",  loraPreamble);
  preferences.putUChar("power",    txPower);
  preferences.putUInt("interval",  txInterval);
  preferences.putBool("gps_en",    gpsEnabled);
  preferences.end();
}

// ════════════════════════════════════════════════════════════════════════════
//  TASK 2 — ДИСПЛЕЙ OLED 128×64
// ════════════════════════════════════════════════════════════════════════════

void initDisplay() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);  // Включить питание дисплея
  delay(100);
  // Импульс сброса OLED (ThingPulse не делает это автоматически)
  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, LOW);
  delay(20);
  digitalWrite(RST_OLED, HIGH);
  delay(50);
  display.init();
  display.flipScreenVertically();  // поворот 180°
  display.clear();
  display.setContrast(255);
  display.display();
}

// Напряжение батареи в вольтах. Кэш 30с — не нагружать АЦП при каждом updateDisplay().
// Делитель 100кΩ + 390кΩ (ratio 4.9): Vbat = raw/4095 × 3.3В × 4.9
float getBatteryVoltage() {
  if (lastBatReadTime > 0 && millis() - lastBatReadTime < 30000) return cachedBatVoltage;
  lastBatReadTime = millis();
  pinMode(BAT_ADC_CTRL, OUTPUT);
  digitalWrite(BAT_ADC_CTRL, HIGH);
  delay(5);
  long raw = 0;
  for (int i = 0; i < 20; i++) raw += analogRead(BAT_ADC_PIN);
  raw /= 20;
  digitalWrite(BAT_ADC_CTRL, LOW);
  cachedBatVoltage = (raw / 4095.0f) * 3.3f * 4.9f;
  Serial.printf("[BAT] raw=%ld V=%.2f\n", raw, cachedBatVoltage);
  return cachedBatVoltage;
}

// Отрисовка экрана:
// Строка 1: [SENDER/RECEIVER] ............. [SEND/LISTEN/STOP]
// Строка 2: P:X  R:X
// Строка 3: GPS: ...
void updateDisplay() {
  String role   = (currentRole == ROLE_SENDER) ? "SENDER" : "RECEIVER";
  String status;
  if (!active)                          status = "STOP";
  else if (currentRole == ROLE_SENDER)  status = "SEND";
  else                                  status = "LISTEN";

  // Строка 3: P(принято) R(rssi) N(номер пакета отправителя)
  uint32_t N = (currentRole == ROLE_SENDER) ? packetCount : lastPacketNum;
  String pkt = "P:" + String(packetCount) +
               " R:" + (lastRssi == 0 ? String("--") : String(lastRssi)) +
               " N:" + String(N);

  // Строка 3: receiver — Loss% + SNR, sender — SF + BW + мощность
  String line3;
  if (currentRole == ROLE_RECEIVER) {
    uint8_t loss = (N > 0 && N >= packetCount) ? (uint8_t)((N - packetCount) * 100 / N) : 0;
    line3 = "Loss:" + String(loss) + "% SNR:" + String(lastSnr);
  } else {
    line3 = String(txInterval / 1000.0, 1) + "s " + String(txPower) + "dBm";
  }

  display.clear();
  display.setFont(ArialMT_Plain_10);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, role);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 0, status);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 16, pkt);
  display.drawString(0, 32, line3);
  display.drawString(0, 48, getGPSDisplayString());
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 48, String(getBatteryVoltage(), 1) + "V");

  display.display();
}

// ════════════════════════════════════════════════════════════════════════════
//  TASK 6 — GPS (Quectel L76K, TinyGPS++, UART1)
// ════════════════════════════════════════════════════════════════════════════

// Возвращает состояние GPS:
// 0=нет данных (Error), 1=поиск (Search), 2=фикс (Find), 3=выключен (OFF)
uint8_t getGPSState() {
  if (!gpsEnabled) return 3;
  if (gps.charsProcessed() == 0)
    return (gpsInitTime > 0 && millis() - gpsInitTime > 30000) ? 0 : 1;
  if (!gps.location.isValid()) return 1;
  return 2;
}

// Возвращает "lat,lon,satellites" для CSV.
// Если GPS выключен или нет фикса — "0.000000,0.000000,0".
String getGPSString() {
  if (!gpsEnabled || !gps.location.isValid())
    return "0.000000,0.000000,0";
  return String(gps.location.lat(), 6) + "," +
         String(gps.location.lng(), 6) + "," +
         String(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
}

// Строка для дисплея — использует ту же логику что и веб
String getGPSDisplayString() {
  uint8_t state = getGPSState();
  if (state == 3) return "GPS: OFF";
  if (state == 0) return "GPS: Error";
  if (state == 1) return "GPS: Search";
  int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
  return "GPS: Find " + String(sats) + " sat";
}

// Включить GPS: подать питание, запустить UART
void enableGPS() {
  gpsEnabled = true;
  gps = TinyGPSPlus();                 // сбросить устаревшие данные предыдущего сеанса
  digitalWrite(VGNSS_CTRL_PIN, LOW);   // P-MOS: LOW = GPS запитан
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gpsInitTime = millis();
  saveSettings();
  addToSerialLog("GPS ON");
  updateDisplay();
}

// Выключить GPS: остановить UART, снять питание
void disableGPS() {
  gpsEnabled = false;
  gpsSerial.end();
  digitalWrite(VGNSS_CTRL_PIN, HIGH);  // P-MOS: HIGH = GPS обесточен
  saveSettings();
  addToSerialLog("GPS OFF");
  updateDisplay();
}

// Опрос GPS — вызывать в каждом цикле loop()
void pollGPS() {
  if (!gpsEnabled) return;

  while (gpsSerial.available())
    gps.encode(gpsSerial.read());

  // Лог координат каждые 10 секунд при наличии фикса
  if (gps.location.isValid() && millis() - lastGpsLogTime >= 10000) {
    lastGpsLogTime = millis();
    addToSerialLog("GPS: " +
                   String(gps.location.lat(), 4) + ", " +
                   String(gps.location.lng(), 4) + " (" +
                   String(gps.satellites.isValid() ? (int)gps.satellites.value() : 0) +
                   " sat)");
    updateDisplay();
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  TASK 5 — ИНИЦИАЛИЗАЦИЯ LoRa (SX1262)
// ════════════════════════════════════════════════════════════════════════════

// Применить текущие параметры (SF/BW/CR/Preamble/Power/Frequency) к радио.
// Останавливает радио, реинициализирует, восстанавливает состояние.
void updateLoRaSettings() {
  bool wasActive = active;
  if (wasActive) {
    stopActive();
  }
  delay(200);

  reinitLoRaForRole();

  if (wasActive) {
    active = true;
    lora_idle = true;
    if (currentRole == ROLE_RECEIVER) radio.startReceive();
  }

  String bwStr = (loraBW == 0) ? "125" : (loraBW == 1) ? "250" : "500";
  String msg = "Frequency: " + String(loraFrequency / 1000000.0, 3) + " MHz" +
               " | SF:" + loraSF +
               " | BW:" + bwStr + "kHz" +
               " | CR:4/" + (loraCR + 4) +
               " | Preamble:" + loraPreamble;
  if (currentRole == ROLE_SENDER)
    msg += " | Power:" + String(txPower) + "dBm";
  addToSerialLog(msg);
}

// Переключить конфигурацию радио под текущую роль (TX или RX)
void reinitLoRaForRole() {
  txInProgress = false;
  radioFlag    = false;

  // BW: 0=125kHz, 1=250kHz, 2=500kHz
  const float bwTable[] = {125.0f, 250.0f, 500.0f};
  float bwKHz = bwTable[loraBW < 3 ? loraBW : 0];
  // CR: 1→5(4/5), 2→6(4/6), 3→7(4/7), 4→8(4/8)
  uint8_t crCode = loraCR + 4;

  int16_t state = radio.begin(
    loraFrequency / 1e6f,               // частота в МГц
    bwKHz,                              // ширина полосы кГц
    loraSF,                             // SF 7-12
    crCode,                             // CR 5-8
    RADIOLIB_SX126X_SYNC_WORD_PRIVATE,  // private network
    (int8_t)txPower,                    // мощность dBm
    (uint16_t)loraPreamble,             // преамбула
    1.8f                                // TCXO voltage для Heltec V4
  );

  if (state != RADIOLIB_ERR_NONE) {
    addToSerialLog("LoRa init failed: " + String(state));
    return;
  }

  radio.setDio2AsRfSwitch(true);  // DIO2 управляет RF-переключателем на V4
  radio.setDio1Action(setRadioFlag);
  // Не вызываем radio.startReceive() здесь — только через handleStart() / updateLoRaSettings()
  // когда active=true. Иначе: phantom receive при active=false, и двойной startReceive() в updateLoRaSettings().
}

// ─── LoRa колбэки ────────────────────────────────────────────────────────────

// Таблица приблизительного времени передачи пакета ~15 байт, BW=125kHz, CR=4/5
unsigned long calcPacketTimeMs() {
  const unsigned long t[] = {800, 1200, 1800, 2600, 3700, 5300}; // SF7..SF12
  uint8_t idx = loraSF < 7 ? 0 : loraSF > 12 ? 5 : loraSF - 7;
  return t[idx];
}

void OnTxDone(void) {
  lora_idle = true;
  digitalWrite(LED, LOW);   // LED off после передачи
  String gps = getGPSString();
  addToSerialLog("P:" + String(packetCount) + " D:" + String(txpacket));
  addToCSVLogSender(packetCount, 0, txpacket, gps);
  updateDisplay();
}


void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  if (!active || currentRole != ROLE_RECEIVER) return;

  digitalWrite(LED, HIGH);  // LED on при приёме пакета
  lastRssi = rssi;
  lastSnr  = snr;
  packetCount++;
  // Извлекаем номер пакета из содержимого "Packet #N"
  uint32_t n = 0;
  if (sscanf((char*)payload, "Packet #%lu", &n) == 1) lastPacketNum = n;

  memset(rxpacket, 0, BUFFER_SIZE);
  memcpy(rxpacket, payload, size < BUFFER_SIZE - 1 ? size : BUFFER_SIZE - 1);

  String gps = getGPSString();
  addToSerialLog("P:" + String(packetCount) +
                 " R:" + rssi +
                 " S:" + snr +
                 " D:" + String(rxpacket));
  addToCSVLogReceiver(packetCount, rssi, snr, rxpacket, gps);
  updateDisplay();
  digitalWrite(LED, LOW);   // LED off после обработки пакета
}

// ════════════════════════════════════════════════════════════════════════════
//  TASK 1 + 9/10 — ВЕБ-СЕРВЕР
// ════════════════════════════════════════════════════════════════════════════

// --- Вспомогательные функции для HTML ---

// Генерирует HTML <option> для <select>
String selectOption(const String& val, const String& label, bool selected) {
  return "<option value=\"" + val + "\"" + (selected ? " selected" : "") + ">" + label + "</option>";
}

// --- Главная страница ---
void handleRoot() {
  String roleLabel  = (currentRole == ROLE_SENDER) ? "SENDER" : "RECEIVER";
  String startLabel = active ? "STOP" : "START";

  // Опции для select-полей
  String sfOpts = selectOption("7","SF7",loraSF==7) + selectOption("8","SF8",loraSF==8) +
                  selectOption("9","SF9",loraSF==9) + selectOption("10","SF10",loraSF==10) +
                  selectOption("11","SF11",loraSF==11) + selectOption("12","SF12",loraSF==12);
  String bwOpts = selectOption("0","125 kHz",loraBW==0) + selectOption("1","250 kHz",loraBW==1) +
                  selectOption("2","500 kHz",loraBW==2);
  String crOpts = selectOption("1","4/5",loraCR==1) + selectOption("2","4/6",loraCR==2) +
                  selectOption("3","4/7",loraCR==3) + selectOption("4","4/8",loraCR==4);

  String html = R"rawliteral(<!DOCTYPE html>
<html lang="ru"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>)rawliteral" + String(DEVICE_NAME) + R"rawliteral( — LoRa Tester</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,sans-serif;background:#f0f4f8;color:#1a202c;padding:12px}
.card{background:#fff;border-radius:16px;box-shadow:0 2px 12px rgba(0,0,0,.08);margin-bottom:12px;overflow:hidden}
.card-header{padding:14px 16px;display:flex;align-items:center;justify-content:space-between}
.card-header h1{font-size:17px;font-weight:700}
.badge{display:inline-flex;align-items:center;gap:6px;padding:4px 10px;border-radius:20px;font-size:12px;font-weight:700}
.badge.stopped{background:#fce4e4;color:#c62828}
.badge.active{background:#e6f4ea;color:#1e7e34}
.badge .dot{width:7px;height:7px;border-radius:50%}
.badge.stopped .dot{background:#c62828}
.badge.active .dot{background:#28a745;animation:blink 1s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.3}}
.btns{display:grid;grid-template-columns:1fr 1fr;gap:8px;padding:12px 16px}
.btn{border:none;border-radius:12px;padding:11px 8px;font-size:14px;font-weight:700;cursor:pointer;transition:opacity .15s}
.btn:active{opacity:.7}
.btn-role{background:#e3f2fd;color:#1565c0}
.btn-start{background:#e6f4ea;color:#1e7e34}
.btn-stop{background:#fce4e4;color:#c62828}
.btn-clear{background:#fff8e1;color:#f57f17}
.btn-save{background:#ede7f6;color:#4527a0}
.console{padding:0 16px 16px}
.console-box{background:#0d1117;color:#7ee787;font-family:ui-monospace,monospace;font-size:12px;line-height:1.5;padding:12px;border-radius:12px;height:240px;overflow-y:auto;white-space:pre-wrap;word-break:break-all}
.settings{padding:0 16px 16px;display:flex;flex-direction:column;gap:8px}
.gps-ok{color:#1e7e34}.gps-search{color:#856404}.gps-off{color:#718096}
.row{display:flex;align-items:center;justify-content:space-between;background:#f7fafc;border-radius:10px;padding:8px 12px;border:1px solid #edf2f7}
.row label{font-size:13px;color:#4a5568;font-weight:600}
.row input,.row select{font-size:13px;border:none;background:transparent;color:#1a202c;text-align:right;width:110px;font-family:ui-monospace,monospace;padding:2px 4px;border-radius:6px}
.row input:focus,.row select:focus{outline:2px solid #4299e1;background:#ebf8ff}
.sender-only{display:)rawliteral" + String(currentRole == ROLE_SENDER ? "flex" : "none") + R"rawliteral(}
</style></head><body>
<div class="card">
  <div class="card-header">
    <h1>)rawliteral" + String(DEVICE_NAME) + R"rawliteral(</h1>
    <span class="badge )rawliteral" + String(active ? "active" : "stopped") + R"rawliteral(" id="badge">
      <span class="dot"></span>
      <span id="badgeText">)rawliteral" + String(active ? (currentRole == ROLE_SENDER ? "SEND" : "LISTEN") : "STOP") + R"rawliteral(</span>
    </span>
  </div>
  <div class="btns">
    <button class="btn btn-role" id="btnRole" onclick="toggleRole()">)rawliteral" + (currentRole == ROLE_SENDER ? "SENDER →" : "← RECEIVER") + R"rawliteral(</button>
    <button class="btn )rawliteral" + String(active ? "btn-stop" : "btn-start") + R"rawliteral(" id="btnStart" onclick="toggleActive()">)rawliteral" + startLabel + R"rawliteral(</button>
    <button class="btn btn-clear" onclick="clearLog()">CLEAR LOG</button>
    <button class="btn btn-save" onclick="saveCSV()">SAVE CSV</button>
  </div>
</div>
<div class="card">
  <div class="card-header"><h1>Console</h1><span id="pktBadge" style="font-size:12px;color:#718096">P:0</span></div>
  <div class="console"><div class="console-box" id="console">Connecting...</div></div>
</div>
<div class="card">
  <div class="card-header">
    <h1>GPS</h1>
    <span id="gpsStatus" class="gps-search" style="font-size:13px;font-family:ui-monospace,monospace">Search</span>
  </div>
</div>
<div class="card">
  <div class="card-header"><h1>LoRa Settings</h1></div>
  <div class="settings">
    <div class="row">
      <label>Frequency (Hz)</label>
      <input type="number" id="freq" min="860000000" max="870000000" step="100000" value=")rawliteral" + String(loraFrequency) + R"rawliteral(" onchange="setSetting('/frequency',this.value,860000000,870000000)">
    </div>
    <div class="row">
      <label>Spreading Factor</label>
      <select id="sf" onchange="setSetting('/sf',this.value,7,12)">)rawliteral" + sfOpts + R"rawliteral(</select>
    </div>
    <div class="row">
      <label>Bandwidth</label>
      <select id="bw" onchange="setSetting('/bw',this.value,0,2)">)rawliteral" + bwOpts + R"rawliteral(</select>
    </div>
    <div class="row">
      <label>Coding Rate</label>
      <select id="cr" onchange="setSetting('/cr',this.value,1,4)">)rawliteral" + crOpts + R"rawliteral(</select>
    </div>
    <div class="row">
      <label>Preamble</label>
      <input type="number" id="preamble" min="6" max="65535" value=")rawliteral" + String(loraPreamble) + R"rawliteral(" onchange="setSetting('/preamble',this.value,6,65535)">
    </div>
    <div class="row sender-only" id="rowPower">
      <label>TX Power (dBm)</label>
      <input type="number" id="power" min="0" max="22" value=")rawliteral" + String(txPower) + R"rawliteral(" onchange="setSetting('/power',this.value,0,22)">
    </div>
    <div class="row sender-only" id="rowInterval">
      <label>Interval (s)</label>
      <input type="number" id="interval" min="1" max="60" step="0.5" value=")rawliteral" + String(txInterval / 1000.0, 1) + R"rawliteral(" onchange="setSetting('/interval',this.value,1,60)">
    </div>
  </div>
</div>
<script>
let _role=)rawliteral" + String(currentRole) + R"rawliteral(,_active=)rawliteral" + String(active ? "true" : "false") + R"rawliteral(,_editing=false;
const q=id=>document.getElementById(id);

// Обновление лога
function pollLog(){
  fetch('/log').then(r=>r.text()).then(t=>{
    const el=q('console');
    const atBottom=el.scrollTop+el.clientHeight>=el.scrollHeight-20;
    el.textContent=t||'(empty)';
    if(atBottom)el.scrollTop=el.scrollHeight;
  }).catch(()=>{});
}

// Обновление статуса
function pollStatus(){
  if(_editing)return;
  fetch('/status').then(r=>r.json()).then(d=>{
    _role=d.role; _active=d.active;
    // badge
    const badge=q('badge'), btxt=q('badgeText');
    if(d.active){
      badge.className='badge active';
      btxt.textContent=d.role===0?'SEND':'LISTEN';
    } else {
      badge.className='badge stopped';
      btxt.textContent='STOP';
    }
    // кнопки
    q('btnRole').textContent=d.role===0?'SENDER \u2192':'\u2190 RECEIVER';
    q('btnStart').textContent=d.active?'STOP':'START';
    q('btnStart').className='btn '+(d.active?'btn-stop':'btn-start');
    q('pktBadge').textContent='P:'+d.packets+'  R:'+(d.rssi||'--');
    // sender-only поля
    const so=document.querySelectorAll('.sender-only');
    so.forEach(el=>el.style.display=d.role===0?'flex':'none');
    // поля настроек
    q('freq').value=d.frequency;
    q('sf').value=d.sf;
    q('bw').value=d.bw;
    q('cr').value=d.cr;
    q('preamble').value=d.preamble;
    q('power').value=d.power;
    q('interval').value=(d.interval/1000).toFixed(1);
    // GPS статус
    const gs=q('gpsStatus');
    if(!d.gps_enabled){gs.textContent='OFF';gs.className='gps-off';}
    else if(d.gps_state===2){gs.textContent='Find '+d.gps_sats+' sat';gs.className='gps-ok';}
    else if(d.gps_state===1){gs.textContent='Search';gs.className='gps-search';}
    else{gs.textContent='Error';gs.className='gps-off';}
  }).catch(()=>{});
}

// Переключить роль
function toggleRole(){
  const newRole=_role===0?1:0;
  fetch('/role?value='+newRole).then(()=>pollStatus()).catch(()=>{});
}

// Старт/Стоп
function toggleActive(){
  fetch(_active?'/stop':'/start').then(()=>pollStatus()).catch(()=>{});
}

// Очистить лог
function clearLog(){
  if(!confirm('Очистить лог?'))return;
  fetch('/clear').then(()=>pollLog()).catch(()=>{});
}

// Сохранить CSV
function saveCSV(){
  fetch('/csv').then(r=>r.blob()).then(b=>{
    const url=URL.createObjectURL(b);
    const a=document.createElement('a');
    a.href=url;
    a.download='lora_log_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';
    a.click(); URL.revokeObjectURL(url);
  }).catch(()=>{});
}

// Изменить настройку LoRa
function setSetting(endpoint,val,min,max){
  const v=parseFloat(val);
  if(isNaN(v)||v<min||v>max){pollStatus();return;}
  fetch(endpoint+'?value='+v).then(()=>pollStatus()).catch(()=>{});
}

// Отслеживаем фокус на полях — не перезаписывать во время редактирования
document.querySelectorAll('input,select').forEach(el=>{
  el.addEventListener('focus',()=>_editing=true);
  el.addEventListener('blur',()=>{_editing=false;setTimeout(pollStatus,300);});
});

setInterval(pollLog,2000);
setInterval(pollStatus,2000);
pollLog();
setTimeout(pollStatus,1000); // сдвиг 1с — не отправлять два запроса одновременно
</script>
</body></html>
)rawliteral";

  server.send(200, "text/html; charset=utf-8", html);
}

// --- REST эндпоинты ---

void handleLog() {
  server.send(200, "text/plain; charset=utf-8", serialLog);
}

void handleCSV() {
  server.sendHeader("Content-Disposition", "attachment; filename=\"lora_log.csv\"");
  server.send(200, "text/csv; charset=utf-8", csvLog);
}

void handleClear() {
  resetSession();
  serialLog = "";
  addToSerialLog("Log cleared");
  updateDisplay();
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  if (!active) {
    active = true;
    lora_idle = true;
    if (currentRole == ROLE_RECEIVER) radio.startReceive();
    addToSerialLog(currentRole == ROLE_SENDER ? "Transmission STARTED" : "Reception STARTED");
    updateDisplay();
  }
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  stopActive();
  addToSerialLog(currentRole == ROLE_SENDER ? "Transmission STOPPED" : "Reception STOPPED");
  updateDisplay();
  server.send(200, "text/plain", "OK");
}

void handleRole() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  int v = server.arg("value").toInt();
  if (v != ROLE_SENDER && v != ROLE_RECEIVER) { server.send(400, "text/plain", "Invalid role"); return; }

  if (active) { stopActive(); }
  currentRole = v;
  saveSettings();
  resetSession();
  reinitLoRaForRole();
  addToSerialLog("Role changed: " + String(currentRole == ROLE_SENDER ? "SENDER" : "RECEIVER"));
  updateDisplay();
  server.send(200, "text/plain", "OK");
}

void handleSF() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  int v = server.arg("value").toInt();
  if (v < 7 || v > 12) { server.send(400, "text/plain", "SF: 7-12"); return; }
  loraSF = v; updateLoRaSettings(); saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleBW() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  int v = server.arg("value").toInt();
  if (v < 0 || v > 2) { server.send(400, "text/plain", "BW: 0-2"); return; }
  loraBW = v; updateLoRaSettings(); saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleCR() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  int v = server.arg("value").toInt();
  if (v < 1 || v > 4) { server.send(400, "text/plain", "CR: 1-4"); return; }
  loraCR = v; updateLoRaSettings(); saveSettings();
  server.send(200, "text/plain", "OK");
}

void handlePreamble() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  long v = server.arg("value").toInt();
  if (v < 6 || v > 65535) { server.send(400, "text/plain", "Preamble: 6-65535"); return; }
  loraPreamble = v; updateLoRaSettings(); saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleFrequency() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  long v = server.arg("value").toInt();
  if (v < 860000000L || v > 870000000L) { server.send(400, "text/plain", "Freq: 860-870 MHz"); return; }
  loraFrequency = v; updateLoRaSettings(); saveSettings();
  server.send(200, "text/plain", "OK");
}

void handlePower() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  int v = server.arg("value").toInt();
  if (v < 0 || v > 22) { server.send(400, "text/plain", "Power: 0-22 dBm"); return; }
  txPower = v; updateLoRaSettings(); saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleInterval() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  float v = server.arg("value").toFloat();
  if (v < 1.0f || v > 60.0f) { server.send(400, "text/plain", "Interval: 1-60s"); return; }
  txInterval = (uint32_t)(v * 1000.0f);
  saveSettings();
  addToSerialLog("Interval: " + String(v, 1) + "s");
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  String json = "{";
  json += "\"role\":"      + String(currentRole)              + ",";
  json += "\"active\":"    + String(active ? "true" : "false") + ",";
  json += "\"packets\":"   + String(packetCount)              + ",";
  json += "\"rssi\":"      + String(lastRssi)                 + ",";
  json += "\"snr\":"       + String(lastSnr)                  + ",";
  json += "\"frequency\":" + String(loraFrequency)            + ",";
  json += "\"sf\":"        + String(loraSF)                   + ",";
  json += "\"bw\":"        + String(loraBW)                   + ",";
  json += "\"cr\":"        + String(loraCR)                   + ",";
  json += "\"preamble\":"  + String(loraPreamble)             + ",";
  json += "\"power\":"     + String(txPower)                  + ",";
  json += "\"interval\":"  + String(txInterval)               + ",";
  json += "\"gps_state\":"   + String(getGPSState())            + ",";
  json += "\"gps_sats\":"   + String(gps.satellites.isValid() ? (int)gps.satellites.value() : 0) + ",";
  json += "\"gps_enabled\":" + String(gpsEnabled ? "true" : "false") + ",";
  json += "\"vbat\":"        + String(cachedBatVoltage, 2);
  json += "}";
  server.send(200, "application/json", json);
}

// ════════════════════════════════════════════════════════════════════════════
//  КНОПКА USER (GPIO 0)
//  1 клик (нажать+отжать):       старт/стоп
//  2 быстрых клика:               смена роли Sender ↔ Receiver
//  Удержание 5с (без отжима):     глубокий сон / пробуждение
// ════════════════════════════════════════════════════════════════════════════

// Корректная остановка — сбрасывает TX-состояние и гасит LED
void stopActive() {
  active       = false;
  txInProgress = false;
  lora_idle    = true;
  radioFlag    = false;
  digitalWrite(LED, LOW);
  radio.sleep();
}

void enterDeepSleep() {
  addToSerialLog("Going to sleep...");
  stopActive();
  digitalWrite(VGNSS_CTRL_PIN, HIGH);  // выключить GPS
  display.displayOff();
  digitalWrite(Vext, HIGH);            // выключить Vext (OLED)
  delay(100);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  // пробуждение по GPIO0 LOW
  esp_deep_sleep_start();
}

void handleUserButtonShort() {
  if (active) {
    stopActive();
    addToSerialLog(currentRole == ROLE_SENDER ? "Transmission STOPPED" : "Reception STOPPED");
  } else {
    active = true;
    lora_idle = true;
    if (currentRole == ROLE_RECEIVER) radio.startReceive();
    addToSerialLog(currentRole == ROLE_SENDER ? "Transmission STARTED" : "Reception STARTED");
  }
  updateDisplay();
}

void handleUserButtonDoubleClick() {
  if (active) { stopActive(); }
  currentRole = (currentRole == ROLE_SENDER) ? ROLE_RECEIVER : ROLE_SENDER;
  saveSettings();
  resetSession();
  reinitLoRaForRole();
  addToSerialLog("Role → " + String(currentRole == ROLE_SENDER ? "SENDER" : "RECEIVER"));
  updateDisplay();
}

void handleUserButtonTripleClick() {
  if (gpsEnabled) disableGPS();
  else            enableGPS();
}

// ════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  // После пробуждения из deep sleep — требуем удержать кнопку 5с
  // Если отпустили раньше — уходим обратно спать
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    pinMode(0, INPUT_PULLUP);
    unsigned long t = millis();
    while (digitalRead(0) == LOW) {
      if (millis() - t >= 5000) break;  // держат 5с — продолжаем загрузку
      delay(10);
    }
    if (digitalRead(0) != LOW) {        // отпустили раньше — спать обратно
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
      esp_deep_sleep_start();
    }
    // Кнопка ещё зажата после пробуждения — сигнализируем state machine игнорировать это удержание
    btnSleepFired = true;
  }

  // Task 4: настройки из NVS (первым делом — addToSerialLog уже работает)
  loadSettings();

  // Task 2: дисплей
  initDisplay();
  updateDisplay();

  // Кнопка USER
  pinMode(0, INPUT_PULLUP);

  // LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Task 1: WiFi AP
  addToSerialLog("WiFi AP: " + String(DEVICE_NAME));
  WiFi.softAP(DEVICE_NAME, WIFI_PASSWORD);
  addToSerialLog("IP: " + WiFi.softAPIP().toString());

  // Task 1: веб-сервер
  server.on("/",          handleRoot);
  server.on("/log",       handleLog);
  server.on("/csv",       handleCSV);
  server.on("/clear",     handleClear);
  server.on("/start",     handleStart);
  server.on("/stop",      handleStop);
  server.on("/role",      handleRole);
  server.on("/sf",        handleSF);
  server.on("/bw",        handleBW);
  server.on("/cr",        handleCR);
  server.on("/preamble",  handlePreamble);
  server.on("/frequency", handleFrequency);
  server.on("/power",     handlePower);
  server.on("/interval",  handleInterval);
  server.on("/status",    handleStatus);
  server.begin();
  addToSerialLog("Web server started");

  // Task 6: GPS (Quectel L76K, UART1)
  pinMode(GPS_RST_PIN, OUTPUT);
  digitalWrite(GPS_RST_PIN, HIGH);     // HIGH = GPS не в сбросе (разделяет с RST_OLED)
  pinMode(VGNSS_CTRL_PIN, OUTPUT);
  if (gpsEnabled) {
    digitalWrite(VGNSS_CTRL_PIN, LOW); // AO3401A P-MOS: LOW = GPS запитан
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    gpsInitTime = millis();
    addToSerialLog("GPS UART started (RX:" + String(GPS_RX_PIN) + " TX:" + String(GPS_TX_PIN) + ")");
  } else {
    digitalWrite(VGNSS_CTRL_PIN, HIGH); // P-MOS: HIGH = GPS обесточен
    addToSerialLog("GPS disabled (saved state)");
  }

  // Task 5: LoRa SX1262 (RadioLib)
  reinitLoRaForRole();
  addToSerialLog("LoRa initialized | " +
                 String(loraFrequency / 1000000.0, 3) + " MHz | SF" + loraSF);

  updateDisplay();
}

// ════════════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════════════

void loop() {
  server.handleClient();

  // Обработка прерывания от SX1262 (TxDone или RxDone)
  if (radioFlag) {
    radioFlag = false;
    if (txInProgress) {
      txInProgress = false;
      OnTxDone();
    } else if (active && currentRole == ROLE_RECEIVER) {
      size_t pktLen = radio.getPacketLength();
      if (pktLen > 0) {
        if (pktLen >= BUFFER_SIZE) pktLen = BUFFER_SIZE - 1;
        uint8_t buf[BUFFER_SIZE];
        int16_t state = radio.readData(buf, pktLen);
        buf[pktLen] = 0;
        if (state == RADIOLIB_ERR_NONE) {
          OnRxDone(buf, (uint16_t)pktLen,
                   (int16_t)radio.getRSSI(), (int8_t)radio.getSNR());
        }
      }
      if (active) radio.startReceive();
    }
  }

  pollGPS();  // Task 6: чтение NMEA из GPS

  // Кнопка USER (GPIO 0): state machine
  //   1 клик          → старт/стоп
  //   2 быстрых клика → смена роли
  //   3 быстрых клика → вкл/выкл GPS
  //   Удержание 5с    → глубокий сон
  {
    enum BtnState : uint8_t {
      BTN_IDLE, BTN_PRESSED_1, BTN_WAIT_2,
      BTN_PRESSED_2, BTN_WAIT_3, BTN_PRESSED_3
    };
    static BtnState      btnState       = BTN_IDLE;
    static unsigned long btnPressTime   = 0;
    static unsigned long btnReleaseTime = 0;
    static const unsigned long DOUBLE_MS = 400;   // окно ожидания следующего клика, мс
    static const unsigned long SLEEP_MS  = 5000;  // порог удержания для сна, мс

    bool btnNow = (digitalRead(0) == LOW);
    switch (btnState) {

      case BTN_IDLE:
        if (btnNow) { btnState = BTN_PRESSED_1; btnPressTime = millis(); }
        break;

      case BTN_PRESSED_1:
        if (!btnNow) {
          if (btnSleepFired) {          // удержание при пробуждении — игнорируем
            btnSleepFired = false;
            btnState = BTN_IDLE;
          } else {
            btnState = BTN_WAIT_2;
            btnReleaseTime = millis();
          }
        } else if (!btnSleepFired && millis() - btnPressTime >= SLEEP_MS) {
          enterDeepSleep();             // уход в глубокий сон
          btnState = BTN_IDLE;
        }
        break;

      case BTN_WAIT_2:
        if (btnNow) {
          btnState = BTN_PRESSED_2;    // второй клик — ждём третьего или отпускания
        } else if (millis() - btnReleaseTime >= DOUBLE_MS) {
          handleUserButtonShort();      // одиночный клик → старт/стоп
          btnState = BTN_IDLE;
        }
        break;

      case BTN_PRESSED_2:
        if (!btnNow) {
          btnState = BTN_WAIT_3;
          btnReleaseTime = millis();
        }
        break;

      case BTN_WAIT_3:
        if (btnNow) {
          handleUserButtonTripleClick(); // третий клик → вкл/выкл GPS
          btnState = BTN_PRESSED_3;
        } else if (millis() - btnReleaseTime >= DOUBLE_MS) {
          handleUserButtonDoubleClick(); // двойной клик → смена роли
          btnState = BTN_IDLE;
        }
        break;

      case BTN_PRESSED_3:
        if (!btnNow) btnState = BTN_IDLE;
        break;
    }
  }

  // Task 7: Sender TX loop
  // Тайм-аут TX: если TxDone-прерывание не пришло — сброс зависания
  if (txInProgress && millis() - lastTxTime > calcPacketTimeMs() + 2000UL) {
    radio.standby();
    txInProgress = false;
    lora_idle    = true;
    radioFlag    = false;
    digitalWrite(LED, LOW);
    addToSerialLog("TX timeout - radio reset");
  }

  if (active && currentRole == ROLE_SENDER && lora_idle) {
    if (millis() - lastTxTime >= txInterval) {
      lastTxTime = millis();
      packetCount++;
      snprintf(txpacket, sizeof(txpacket), "Packet #%lu", (unsigned long)packetCount);

      // Предупреждение если интервал меньше времени передачи
      unsigned long pktTime = calcPacketTimeMs();
      if (txInterval < pktTime + 500) {
        addToSerialLog("WARN: interval " + String(txInterval/1000.0,1) +
                       "s < min " + String((pktTime+500)/1000.0,1) + "s for SF" + loraSF);
      }

      lora_idle = false;
      txInProgress = true;
      digitalWrite(LED, HIGH);  // LED on во время передачи
      int16_t txState = radio.startTransmit((uint8_t*)txpacket, strlen(txpacket));
      if (txState != RADIOLIB_ERR_NONE) {
        txInProgress = false;
        lora_idle = true;
        digitalWrite(LED, LOW);
        addToSerialLog("TX error: " + String(txState));
      }
    }
  }
}
