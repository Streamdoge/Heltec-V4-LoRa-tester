#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// Настройки WiFi
const char* ssid = "Sender01";
const char* password = "1223334444";

// Веб-сервер на порту 80
WebServer server(80);

// Для сохранения настроек в EEPROM
Preferences preferences;

// НАСТРОЙКИ ДЛЯ МАКСИМАЛЬНОЙ ДАЛЬНОСТИ 868 МГц
#define DEFAULT_RF_FREQUENCY                        868000000
#define LORA_BANDWIDTH                              0         // 125 kHz (лучшая чувствительность)
#define LORA_SPREADING_FACTOR                       12        // SF12 (максимальная дальность)
#define LORA_CODINGRATE                             4         // CR 4/8 (лучшая помехоустойчивость)
#define LORA_PREAMBLE_FACTOR                        16        // Удлиненный preamble
#define LORA_SYMBOL_TIMEOUT                         16        // Увеличенный timeout для SF12
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define BUFFER_SIZE                                 50

// Radio events - ПЕРЕМЕСТИЛИ ВЫШЕ!
static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

char txpacket[BUFFER_SIZE];
uint32_t txNumber = 0;  // Начинаем с 0
int16_t rssi = 0;
bool lora_idle = true;
bool transmissionEnabled = false; // По умолчанию выключено

// Параметры LoRa с возможностью настройки
unsigned long transmissionInterval = 5000; // По умолчанию 5 секунд
uint8_t txPower = 20;                      // Мощность передачи (0-20 dBm)
uint8_t loraSF = LORA_SPREADING_FACTOR;    // Spreading Factor (7-12)
uint8_t loraBW = LORA_BANDWIDTH;           // Bandwidth (0=125kHz, 1=250kHz, 2=500kHz)
uint8_t loraCR = LORA_CODINGRATE;          // Coding Rate (1=4/5, 2=4/6, 3=4/7, 4=4/8)
uint8_t loraPreamble = LORA_PREAMBLE_FACTOR; // Preamble length (6-65535)
uint32_t loraFrequency = DEFAULT_RF_FREQUENCY; // Частота в Гц

// Настройки максимального размера логов
const int MAX_SERIAL_LOG_LINES = 500;   // 500 строк, потом перезапись
const int MAX_CSV_LOG_LINES = 500;      // 500 строк, потом перезапись
bool memoryLimitReached = false;        // Теперь всегда false - нет паузы

// Буферы для логов
String csvLog = "Time,Packet,RSSI,Data\n";
String serialLog = "";

// Дисплей
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_64_32, RST_OLED);

// Функция расчета времени передачи для разных SF (в миллисекундах)
unsigned long calculatePacketTime() {
  // РЕАЛЬНЫЕ значения для пакета ~15 байт, BW=125kHz, CR=4/8
  const unsigned long sfTiming[] = {
    800,    // SF7  (~0.8 сек)
    1200,   // SF8  (~1.2 сек)  
    1800,   // SF9  (~1.8 сек)
    2600,   // SF10 (~2.6 сек)
    3700,   // SF11 (~3.7 сек)
    5300    // SF12 (~5.3 сек)
  };
  
  // Гарантируем, что SF в диапазоне 7-12
  uint8_t safeSF = loraSF;
  if (safeSF < 7) safeSF = 7;
  if (safeSF > 12) safeSF = 12;
  
  int sfIndex = safeSF - 7;
  return sfTiming[sfIndex];
}

// Функции для работы с настройками
void saveSettings() {
  preferences.begin("lora-settings", false);
  preferences.putUInt("interval", transmissionInterval);
  preferences.putUChar("power", txPower);
  preferences.putUChar("sf", loraSF);
  preferences.putUChar("bw", loraBW);
  preferences.putUChar("cr", loraCR);
  preferences.putUInt("preamble", loraPreamble);
  preferences.putUInt("frequency", loraFrequency);
  preferences.end();
  addToSerialLog("Settings saved");
}

void loadSettings() {
  preferences.begin("lora-settings", true);
  transmissionInterval = preferences.getUInt("interval", 5000);
  txPower = preferences.getUChar("power", 20);
  loraSF = preferences.getUChar("sf", 12);
  loraBW = preferences.getUChar("bw", 0);
  loraCR = preferences.getUChar("cr", 4);
  loraPreamble = preferences.getUInt("preamble", 16);
  loraFrequency = preferences.getUInt("frequency", DEFAULT_RF_FREQUENCY);
  preferences.end();
  
  // Краткий лог загруженных настроек
  String settingsLog = "Frequency: " + String(loraFrequency/1000000.0, 3) + " MHz | " +
                      "SF: " + String(loraSF) + " | BW: " + 
                      String(loraBW == 0 ? "125" : loraBW == 1 ? "250" : "500") + "kHz | CR: 4/" + 
                      String(loraCR + 4) + " | Preamble: " + String(loraPreamble) + " | " +
                      "Power: " + String(txPower) + "dBm | Interval: " + String(transmissionInterval/1000.0, 1) + "s";
  addToSerialLog(settingsLog);
}

// Функции дисплея
void initDisplay() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);
  
  display.init();
  display.clear();
  display.setContrast(255);
  display.screenRotate(ANGLE_0_DEGREE);
  display.display();
}

void displayScreen(const String& line1, const String& line2, const String& line3) {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, line1);
  display.drawString(0, 10, line2);
  display.drawString(0, 20, line3);
  display.display();
}

void updateDisplay() {
  if (memoryLimitReached) {
    displayScreen("P:" + String(txNumber), "R:" + String(rssi), "END");
  } else if (transmissionEnabled) {
    displayScreen("P:" + String(txNumber), "R:" + String(rssi), "SEND");
  } else {
    displayScreen("P:" + String(txNumber), "R:" + String(rssi), "STOP");
  }
}

void displayTimeout() {
  displayScreen("P:" + String(txNumber), "R:" + String(rssi), "Timeout!");
}

// Функция для добавления в Serial лог (веб-интерфейс)
void addToSerialLog(const String& message) {
  String timestamp = "[" + String(millis()/1000) + "s] ";
  String fullMessage = timestamp + message;
  
  // Добавляем новое сообщение В КОНЕЦ (а не в начало)
  if (serialLog.length() > 0) {
    serialLog += "\n" + fullMessage;
  } else {
    serialLog = fullMessage;
  }
  
  // ВСЕГДА выводим в Serial Monitor
  Serial.println(fullMessage);
  
  // Ограничиваем размер лога (перезапись старых записей)
  int newlineCount = 0;
  int startIndex = 0;
  for (int i = serialLog.length() - 1; i >= 0; i--) {
    if (serialLog.charAt(i) == '\n') {
      newlineCount++;
      if (newlineCount >= MAX_SERIAL_LOG_LINES) {
        startIndex = i + 1;
        break;
      }
    }
  }
  
  if (startIndex > 0) {
    serialLog = serialLog.substring(startIndex);
  }
}

// Функция для добавления в CSV лог (только отправки пакетов)
void addToCSVLog(uint32_t packetNum, int16_t rssiVal, const char* data) {
  String csvLine = String(millis()/1000) + "," + 
                   String(packetNum) + "," + 
                   String(rssiVal) + "," + 
                   String(data);
  
  csvLog = csvLine + "\n" + csvLog;
  
  // Ограничиваем размер лога (перезапись старых записей)
  int newlineCount = 0;
  int maxIndex = csvLog.length();
  for (int i = 0; i < csvLog.length(); i++) {
    if (csvLog.charAt(i) == '\n') {
      newlineCount++;
      if (newlineCount >= MAX_CSV_LOG_LINES) {
        maxIndex = i;
        break;
      }
    }
  }
  
  if (maxIndex < csvLog.length()) {
    csvLog = csvLog.substring(0, maxIndex);
  }
}

// Функция автоматической очистки логов при возобновлении работы
void autoClearLogs() {
  String clearMsg = "Logs cleared AUTOMATICALLY - starting fresh session";
  serialLog = clearMsg;
  csvLog = "Time,Packet,RSSI,Data\n";
  memoryLimitReached = false;
  txNumber = 0;
  rssi = 0;
  Serial.println(clearMsg);
}

// Обновление настроек LoRa с полной реинициализацией
void updateLoRaSettings() {
  // Сохраняем текущее состояние
  bool wasTransmitting = transmissionEnabled;
  
  // Останавливаем все радиодеятельности
  if (wasTransmitting) {
    transmissionEnabled = false;
    Radio.Sleep();
  }
  
  // Даем время на завершение операций
  delay(200);
  
  // Полная реинициализация радио модуля
  Radio.Init(&RadioEvents);
  Radio.SetChannel(loraFrequency);
  Radio.SetTxConfig(MODEM_LORA, txPower, 0, loraBW,
                   loraSF, loraCR, loraPreamble, 
                   LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, 
                   LORA_IQ_INVERSION_ON, 5000);
  
  // Восстанавливаем состояние
  if (wasTransmitting) {
    delay(100);
    transmissionEnabled = true;
    lora_idle = true; // Сбрасываем флаг занятости
  }
  
  // Краткий лог обновленных настроек
  String settingsLog = "Frequency: " + String(loraFrequency/1000000.0, 3) + " MHz | " +
                      "SF: " + String(loraSF) + " | BW: " + 
                      String(loraBW == 0 ? "125" : loraBW == 1 ? "250" : "500") + "kHz | CR: 4/" + 
                      String(loraCR + 4) + " | Preamble: " + String(loraPreamble) + " | " +
                      "Power: " + String(txPower) + "dBm | Interval: " + String(transmissionInterval/1000.0, 1) + "s";
  addToSerialLog(settingsLog);
}

// Обработчик кнопки User
void handleUserButton() {
  static unsigned long lastButtonPress = 0;
  unsigned long currentTime = millis();
  
  // Защита от дребезга кнопки
  if (currentTime - lastButtonPress > 500) {
    lastButtonPress = currentTime;
    
    transmissionEnabled = !transmissionEnabled;
    
    if (transmissionEnabled) {
      addToSerialLog("Transmission STARTED");
    } else {
      addToSerialLog("Transmission STOPPED");
    }
    
    updateDisplay();
  }
}

// Веб-обработчики
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html><html lang="ru"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Sender 01 Monitor</title><style>
:root{--bg:#f6f8fb;--card:#fff;--text:#0f172a;--muted:#6b7280;--ring:rgba(59,130,246,.25);--r:18px;--shadow:0 6px 22px rgba(15,23,42,.06);--g:#22c55e;--gb:#ecfdf5;--gbr:#86efac;--r:#ef4444;--rb:#fef2f2;--rbr:#fca5a5;--o:#f59e0b;--ob:#fffbeb;--obr:#fcd34d;--b:#3b82f6;--bb:#eff6ff;--bbr:#93c5fd;--mono:ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,monospace;--sans:ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial}
*{box-sizing:border-box}
html,body{height:100%}
body{margin:0;padding:0;font-family:var(--sans);color:var(--text);background:var(--bg);overflow-x:hidden}
.wrap{max-width:1200px;margin:0 auto}
.card{background:var(--card);border-radius:var(--r);box-shadow:var(--shadow);border:1px solid #eef2f7}
/* выравниваем отступы шапки с остальными блоками */
.header{display:flex;align-items:center;justify-content:space-between;padding:18px 18px}
h1{margin:0;font-size:24px}
/* предотвращаем перенос/переполнение статуса */
.chip{display:inline-flex;align-items:center;gap:8px;padding:8px 12px;border-radius:999px;font-weight:800;font-size:12px;border:1px solid #e5eaf1;background:#f8fafc;color:#1f2937;white-space:nowrap}
.chip.send{background:#ecfdf5;border-color:#86efac;color:#22c55e}
.chip.stop{background:#fef2f2;border-color:#fca5a5;color:#ef4444}
.chip.end{background:#fffbeb;border-color:#fcd34d;color:#f59e0b}
.chip .dot{width:8px;height:8px;border-radius:50%}
.chip.send .dot{background:#22c55e}
.chip.stop .dot{background:#ef4444}
.chip.end .dot{background:#f59e0b}
.toolbar{display:flex;flex-direction:column;gap:10px;padding:0 18px;margin:0}
/* не даём ряду и кнопкам расталкивать контейнер */
.row{display:flex;gap:10px;flex-wrap:nowrap;min-width:0}
.btn{appearance:none;cursor:pointer;border-radius:14px;padding:12px 16px;font-weight:800;font-size:14px;border:1px solid transparent;flex:1;min-width:0}
.btn:focus{outline:none;box-shadow:0 0 0 4px var(--ring)}
.btn.start{background:var(--gb);color:var(--g);border-color:var(--gbr)}
.btn.stop{background:var(--rb);color:var(--r);border-color:var(--rbr)}
.btn.clear{background:var(--ob);color:var(--o);border-color:var(--obr)}
.btn.save{background:var(--bb);color:var(--b);border-color:var(--bbr)}
.input-group{display:flex;flex-direction:column;gap:10px;padding:0 18px 18px;margin:0}
.control-group{display:flex;align-items:center;justify-content:space-between;gap:8px;padding:8px 12px;background:#f8fafc;border-radius:14px;border:1px solid #e5eaf1;width:100%}
/* лейбл ужимается и не ломает строку */
.control-group label{font-weight:600;font-size:13px;color:var(--muted);white-space:nowrap;flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis}
.control-group input,.control-group select{width:80px;border:none;background:none;font-family:var(--mono);font-size:13px;padding:4px 8px;border-radius:8px;text-align:center;flex:0 0 auto}
.control-group input:focus,.control-group select:focus{outline:none;background:#fff;box-shadow:0 0 0 2px var(--ring)}
.panel{padding:18px 18px 18px}
.panel h3{margin:0 0 12px 0;font-size:13px;color:var(--muted);font-weight:800}
.log{background:#0b0f14;color:#d2f7d2;border:1px solid #0b0f14;border-radius:16px;padding:12px;font-family:var(--mono);font-size:13px;line-height:1.4;white-space:pre-wrap;word-break:break-word;max-height:460px;overflow:auto;box-shadow:inset 0 8px 22px rgba(0,0,0,.45)}
@media(max-width:640px){
  .header{padding:18px}
  .row{flex-wrap:wrap;gap:8px}
  .btn{flex:1 1 48%;min-width:0}
  .control-group{flex-direction:row;align-items:center;justify-content:space-between}
  .control-group input,.control-group select{width:60px}
}
</style></head><body>
<div class="wrap"><div class="card">
  <div class="header">
    <h1>Sender 01 Monitor</h1>
    <div id="statusChip" class="chip" role="status" aria-live="polite"><span class="dot"></span> OFF</div>
  </div>
  <div class="toolbar" role="toolbar" aria-label="Controls">
    <div class="row">
      <button class="btn start" onclick="startTX()">Запуск</button>
      <button class="btn stop" onclick="stopTX()">Остановка</button>
    </div>
    <div class="row">
      <button class="btn clear" onclick="clearLog()">Очистить лог</button>
      <button class="btn save" onclick="saveLog()">Сохранить CSV</button>
    </div>
  </div>
  <div class="panel">
    <h3>Серийный монитор</h3>
    <div class="log" id="logDiv">Подключение…</div>
  </div>
  <div class="input-group">
    <div class="control-group">
      <label for="frequencyInput">Частота (Гц):</label>
      <input type="number" id="frequencyInput" min="860000000" max="870000000" step="1000" value=")rawliteral" + String(loraFrequency) + R"rawliteral(" onchange="updateFrequency()">
    </div>
    <div class="control-group">
      <label for="powerInput">Мощность (dBm):</label>
      <input type="number" id="powerInput" min="0" max="20" step="1" value=")rawliteral" + String(txPower) + R"rawliteral(" onchange="updatePower()">
    </div>
    <div class="control-group">
      <label for="intervalInput">Интервал (сек):</label>
      <input type="number" id="intervalInput" min="0.1" max="60.0" step="0.1" value=")rawliteral" + String(transmissionInterval/1000.0, 1) + R"rawliteral(" onchange="updateInterval()">
    </div>
    <div class="control-group">
      <label for="sfSelect">Spreading Factor:</label>
      <select id="sfSelect" onchange="updateSF()">
        <option value="7")rawliteral" + String(loraSF == 7 ? " selected" : "") + R"rawliteral(>SF7 (быстро)</option>
        <option value="8")rawliteral" + String(loraSF == 8 ? " selected" : "") + R"rawliteral(>SF8</option>
        <option value="9")rawliteral" + String(loraSF == 9 ? " selected" : "") + R"rawliteral(>SF9</option>
        <option value="10")rawliteral" + String(loraSF == 10 ? " selected" : "") + R"rawliteral(>SF10</option>
        <option value="11")rawliteral" + String(loraSF == 11 ? " selected" : "") + R"rawliteral(>SF11</option>
        <option value="12")rawliteral" + String(loraSF == 12 ? " selected" : "") + R"rawliteral(>SF12 (далеко)</option>
      </select>
    </div>
    <div class="control-group">
      <label for="bwSelect">Bandwidth:</label>
      <select id="bwSelect" onchange="updateBW()">
        <option value="0")rawliteral" + String(loraBW == 0 ? " selected" : "") + R"rawliteral(>125 kHz (далеко)</option>
        <option value="1")rawliteral" + String(loraBW == 1 ? " selected" : "") + R"rawliteral(>250 kHz</option>
        <option value="2")rawliteral" + String(loraBW == 2 ? " selected" : "") + R"rawliteral(>500 kHz (быстро)</option>
      </select>
    </div>
    <div class="control-group">
      <label for="crSelect">Coding Rate:</label>
      <select id="crSelect" onchange="updateCR()">
        <option value="1")rawliteral" + String(loraCR == 1 ? " selected" : "") + R"rawliteral(>4/5 (быстро)</option>
        <option value="2")rawliteral" + String(loraCR == 2 ? " selected" : "") + R"rawliteral(>4/6</option>
        <option value="3")rawliteral" + String(loraCR == 3 ? " selected" : "") + R"rawliteral(>4/7</option>
        <option value="4")rawliteral" + String(loraCR == 4 ? " selected" : "") + R"rawliteral(>4/8 (надежно)</option>
      </select>
    </div>
    <div class="control-group">
      <label for="preambleInput">Preamble:</label>
      <input type="number" id="preambleInput" min="6" max="65535" step="1" value=")rawliteral" + String(loraPreamble) + R"rawliteral(" onchange="updatePreamble()">
    </div>
  </div>
</div></div>
<script>
let currentInterval=)rawliteral" + String(transmissionInterval/1000.0, 1) + R"rawliteral(,currentPower=)rawliteral" + String(txPower) + R"rawliteral(,currentSF=)rawliteral" + String(loraSF) + R"rawliteral(,currentBW=)rawliteral" + String(loraBW) + R"rawliteral(,currentCR=)rawliteral" + String(loraCR) + R"rawliteral(,currentPreamble=)rawliteral" + String(loraPreamble) + R"rawliteral(,currentFrequency=)rawliteral" + String(loraFrequency) + R"rawliteral(;let isEditing=false;let activeInput=null;

function updateLog(){fetch('/log').then(r=>r.ok?r.text():Promise.reject()).then(t=>{const el=document.getElementById('logDiv');const atBottom=el.scrollTop+el.clientHeight>=el.scrollHeight-8;el.textContent=t||'–';if(atBottom)el.scrollTop=el.scrollHeight;}).catch(()=>{});}
function updateStatus(){if(isEditing)return;fetch('/status').then(r=>r.ok?r.json():Promise.reject()).then(d=>{const on=!!d.transmissionEnabled;const memoryLimit=!!d.memoryLimit;const chip=document.getElementById('statusChip');if(memoryLimit){chip.className='chip end';chip.innerHTML='<span class="dot"></span> END';}else if(on){chip.className='chip send';chip.innerHTML='<span class="dot"></span> SEND';}else{chip.className='chip stop';chip.innerHTML='<span class="dot"></span> STOP';}currentInterval=d.interval?d.interval/1000:5.0;currentPower=d.power||)rawliteral" + String(txPower) + R"rawliteral(;currentSF=d.sf||)rawliteral" + String(loraSF) + R"rawliteral(;currentBW=d.bw||)rawliteral" + String(loraBW) + R"rawliteral(;currentCR=d.cr||)rawliteral" + String(loraCR) + R"rawliteral(;currentPreamble=d.preamble||)rawliteral" + String(loraPreamble) + R"rawliteral(;currentFrequency=d.frequency||)rawliteral" + String(loraFrequency) + R"rawliteral(;if(!isEditing){document.getElementById('powerInput').value=currentPower;document.getElementById('intervalInput').value=currentInterval.toFixed(1);document.getElementById('sfSelect').value=currentSF;document.getElementById('bwSelect').value=currentBW;document.getElementById('crSelect').value=currentCR;document.getElementById('preambleInput').value=currentPreamble;document.getElementById('frequencyInput').value=currentFrequency;}}).catch(()=>{});}
function startTX(){fetch('/start').then(updateStatus).catch(()=>{});}
function stopTX(){fetch('/stop').then(updateStatus).catch(()=>{});}
function clearLog(){if(!confirm('Очистить лог?'))return;fetch('/clear').then(updateLog).catch(()=>{});}
function saveLog(){fetch('/csv').then(r=>r.ok?r.text():Promise.reject()).then(data=>{const url=URL.createObjectURL(new Blob([data],{type:'text/csv'}));const a=document.createElement('a');a.href=url;a.download='lora_tx_log_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';document.body.appendChild(a);a.click();a.remove();URL.revokeObjectURL(url);}).catch(()=>{});}
function updateInterval(){const newInterval=parseFloat(document.getElementById('intervalInput').value);if(newInterval>=0.1&&newInterval<=60.0){fetch('/interval?value='+newInterval).then(()=>{currentInterval=newInterval;updateStatus();}).catch(()=>{document.getElementById('intervalInput').value=currentInterval.toFixed(1);});}else{alert('Интервал должен быть от 0.1 до 60.0 секунд');document.getElementById('intervalInput').value=currentInterval.toFixed(1);}}
function updatePower(){const newPower=parseInt(document.getElementById('powerInput').value);if(newPower>=0&&newPower<=20){fetch('/power?value='+newPower).then(()=>{currentPower=newPower;updateStatus();}).catch(()=>{document.getElementById('powerInput').value=currentPower;});}else{alert('Мощность должна быть от 0 до 20 dBm');document.getElementById('powerInput').value=currentPower;}}
function updateSF(){const newSF=parseInt(document.getElementById('sfSelect').value);if(newSF>=7&&newSF<=12){fetch('/sf?value='+newSF).then(()=>{currentSF=newSF;updateStatus();}).catch(()=>{document.getElementById('sfSelect').value=currentSF;});}else{alert('Spreading Factor должен быть от 7 до 12');document.getElementById('sfSelect').value=currentSF;}}
function updateBW(){const newBW=parseInt(document.getElementById('bwSelect').value);if(newBW>=0&&newBW<=2){fetch('/bw?value='+newBW).then(()=>{currentBW=newBW;updateStatus();}).catch(()=>{document.getElementById('bwSelect').value=currentBW;});}else{alert('Bandwidth должен быть от 0 до 2');document.getElementById('bwSelect').value=currentBW;}}
function updateCR(){const newCR=parseInt(document.getElementById('crSelect').value);if(newCR>=1&&newCR<=4){fetch('/cr?value='+newCR).then(()=>{currentCR=newCR;updateStatus();}).catch(()=>{document.getElementById('crSelect').value=currentCR;});}else{alert('Coding Rate должен быть от 1 до 4');document.getElementById('crSelect').value=currentCR;}}
function updatePreamble(){const newPreamble=parseInt(document.getElementById('preambleInput').value);if(newPreamble>=6&&newPreamble<=65535){fetch('/preamble?value='+newPreamble).then(()=>{currentPreamble=newPreamble;updateStatus();}).catch(()=>{document.getElementById('preambleInput').value=currentPreamble;});}else{alert('Preamble должен быть от 6 до 65535');document.getElementById('preambleInput').value=currentPreamble;}}
function updateFrequency(){const newFrequency=parseInt(document.getElementById('frequencyInput').value);if(newFrequency>=860000000&&newFrequency<=870000000){fetch('/frequency?value='+newFrequency).then(()=>{currentFrequency=newFrequency;updateStatus();}).catch(()=>{document.getElementById('frequencyInput').value=currentFrequency;});}else{alert('Частота должна быть от 860000000 до 870000000 Гц');document.getElementById('frequencyInput').value=currentFrequency;}}

// Отслеживаем фокус на полях ввода
document.addEventListener('DOMContentLoaded', function() {
  const inputs = document.querySelectorAll('input, select');
  inputs.forEach(input => {
    input.addEventListener('focus', function() {
      isEditing = true;
      activeInput = this.id;
    });
    input.addEventListener('blur', function() {
      isEditing = false;
      activeInput = null;
      setTimeout(updateStatus, 100);
    });
  });
});

setInterval(updateLog,2000);
setInterval(updateStatus,2000);
updateLog();
updateStatus();
</script></body></html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleLog() {
  server.send(200, "text/plain", serialLog);
}

void handleCSV() {
  server.send(200, "text/csv", csvLog);
}

void handleClear() {
  String clearMsg = "Log cleared via web interface";
  serialLog = clearMsg;
  csvLog = "Time,Packet,RSSI,Data\n";
  memoryLimitReached = false;
  txNumber = 0;
  rssi = 0;
  Serial.println(clearMsg);
  updateDisplay();
  server.send(200, "text/plain", "OK");
}

void handleStart() {
  transmissionEnabled = true;
  addToSerialLog("Transmission STARTED");
  updateDisplay();
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  transmissionEnabled = false;
  addToSerialLog("Transmission STOPPED");
  updateDisplay();
  server.send(200, "text/plain", "OK");
}

// Обработчик интервала
void handleInterval() {
  if (server.hasArg("value")) {
    float newInterval = server.arg("value").toFloat();
    if (newInterval >= 0.1 && newInterval <= 60.0) {
      transmissionInterval = (unsigned long)(newInterval * 1000);
      
      // Проверяем, не слишком ли мал интервал для текущего SF
      unsigned long minInterval = calculatePacketTime() + 500;
      if (transmissionInterval < minInterval) {
        String warning = "Interval " + String(newInterval, 1) + 
                        "s may be too short for SF" + String(loraSF) + 
                        " (min: " + String(minInterval/1000.0, 1) + "s)";
        addToSerialLog(warning);
      }
      
      addToSerialLog("Interval: " + String(newInterval, 1) + "s");
      saveSettings();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid interval. Must be between 0.1 and 60.0 seconds");
    }
  } else {
    server.send(400, "text/plain", "Missing interval value");
  }
}

void handlePower() {
  if (server.hasArg("value")) {
    int newPower = server.arg("value").toInt();
    if (newPower >= 0 && newPower <= 20) {
      txPower = newPower;
      updateLoRaSettings();
      saveSettings();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid power. Must be between 0 and 20 dBm");
    }
  } else {
    server.send(400, "text/plain", "Missing power value");
  }
}

void handleSF() {
  if (server.hasArg("value")) {
    int newSF = server.arg("value").toInt();
    if (newSF >= 7 && newSF <= 12) {
      loraSF = newSF;
      updateLoRaSettings();
      saveSettings();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid SF. Must be between 7 and 12");
    }
  } else {
    server.send(400, "text/plain", "Missing SF value");
  }
}

void handleBW() {
  if (server.hasArg("value")) {
    int newBW = server.arg("value").toInt();
    if (newBW >= 0 && newBW <= 2) {
      loraBW = newBW;
      updateLoRaSettings();
      saveSettings();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid BW. Must be between 0 and 2");
    }
  } else {
    server.send(400, "text/plain", "Missing BW value");
  }
}

void handleCR() {
  if (server.hasArg("value")) {
    int newCR = server.arg("value").toInt();
    if (newCR >= 1 && newCR <= 4) {
      loraCR = newCR;
      updateLoRaSettings();
      saveSettings();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid CR. Must be between 1 and 4");
    }
  } else {
    server.send(400, "text/plain", "Missing CR value");
  }
}

void handlePreamble() {
  if (server.hasArg("value")) {
    int newPreamble = server.arg("value").toInt();
    if (newPreamble >= 6 && newPreamble <= 65535) {
      loraPreamble = newPreamble;
      updateLoRaSettings();
      saveSettings();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid preamble. Must be between 6 and 65535");
    }
  } else {
    server.send(400, "text/plain", "Missing preamble value");
  }
}

void handleFrequency() {
  if (server.hasArg("value")) {
    unsigned long newFrequency = server.arg("value").toInt();
    if (newFrequency >= 860000000 && newFrequency <= 870000000) {
      loraFrequency = newFrequency;
      updateLoRaSettings();
      saveSettings();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid frequency. Must be between 860000000 and 870000000 Hz");
    }
  } else {
    server.send(400, "text/plain", "Missing frequency value");
  }
}

void handleStatus() {
  String json = "{";
  json += "\"transmissionEnabled\":" + String(transmissionEnabled ? "true" : "false");
  json += ",\"packetCount\":" + String(txNumber);
  json += ",\"rssi\":" + String(rssi);
  json += ",\"interval\":" + String(transmissionInterval);
  json += ",\"power\":" + String(txPower);
  json += ",\"sf\":" + String(loraSF);
  json += ",\"bw\":" + String(loraBW);
  json += ",\"cr\":" + String(loraCR);
  json += ",\"preamble\":" + String(loraPreamble);
  json += ",\"frequency\":" + String(loraFrequency);
  json += ",\"memoryLimit\":" + String(memoryLimitReached ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  
  // Загружаем настройки из EEPROM
  loadSettings();
  
  // Инициализация дисплея
  initDisplay();
  
  transmissionEnabled = false;
  displayScreen("P:0", "R:--", "STOP");

  // Настройка кнопки User
  pinMode(0, INPUT_PULLUP);

  addToSerialLog("WiFi AP: " + String(ssid));
  
  WiFi.softAP(ssid, password);
  
  addToSerialLog("IP: " + WiFi.softAPIP().toString());
  
  // Настройка веб-сервера
  server.on("/", handleRoot);
  server.on("/log", handleLog);
  server.on("/csv", handleCSV);
  server.on("/clear", handleClear);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/interval", handleInterval);
  server.on("/power", handlePower);
  server.on("/sf", handleSF);
  server.on("/bw", handleBW);
  server.on("/cr", handleCR);
  server.on("/preamble", handlePreamble);
  server.on("/frequency", handleFrequency);
  server.on("/status", handleStatus);
  server.begin();

  addToSerialLog("LoRa transmitter ready");
  
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  
  Radio.Init(&RadioEvents);
  Radio.SetChannel(loraFrequency);
  
  // Инициализация LoRa без лишнего лога
  Radio.SetTxConfig(MODEM_LORA, txPower, 0, loraBW,
                   loraSF, loraCR, loraPreamble, 
                   LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, 
                   LORA_IQ_INVERSION_ON, 5000);
  
  addToSerialLog("Web: http://" + WiFi.softAPIP().toString());
  
  updateDisplay();
}

void loop() {
  server.handleClient();
  
  // Проверка кнопки User
  if (digitalRead(0) == LOW) {
    handleUserButton();
    delay(300);
  }
  
  if (lora_idle && transmissionEnabled) {
    static unsigned long lastSendTime = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastSendTime >= transmissionInterval) {
      lastSendTime = currentTime;
      
      rssi = Radio.Rssi(MODEM_LORA);
      sprintf(txpacket, "%d Sended packet", txNumber + 1);
      
      updateDisplay();
      Radio.Send((uint8_t *)txpacket, strlen(txpacket));
      lora_idle = false;
    }
  }
  
  Radio.IrqProcess();
}

void OnTxDone(void) {
  txNumber++;
  
  String serialMessage = "P:" + String(txNumber) + " R:" + String(rssi) + " D:" + String(txpacket);
  addToSerialLog(serialMessage);
  addToCSVLog(txNumber, rssi, txpacket);
  
  updateDisplay();
  lora_idle = true;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  
  String serialMessage = "P:" + String(txNumber) + " R:" + String(rssi) + " TIMEOUT";
  addToSerialLog(serialMessage);
  
  displayTimeout();
  lora_idle = true;
}