# LoRa Range Tester

Firmware for LoRa range testing on the **Heltec WiFi LoRa 32 V4** board (ESP32-S3 + SX1262).

One firmware — two devices. The **Sender** or **Receiver** role is selected via the web interface and saved to non-volatile memory. Two boards with identical firmware are enough for a range test.

---

## Features

- Two operating modes: **Sender** and **Receiver** — switch without reflashing
- Wi-Fi access point with mobile-friendly web interface
- Real-time LoRa parameter tuning: SF, BW, CR, frequency, TX power, interval
- GPS coordinates in every log entry (Quectel L76K module)
- CSV log export — ready for Excel / Google Sheets
- OLED display 128×64: packet counter, RSSI, SNR, packet loss, GPS status, battery level
- Hardware button: start/stop, role switch, deep sleep
- Li-Ion battery support with charge level display
- Deep sleep with button wake-up

---

## Hardware

| Component | Description |
|-----------|-------------|
| **MCU** | Heltec WiFi LoRa 32 V4 (ESP32-S3R2) |
| **LoRa** | SX1262, 868 MHz (EU) |
| **Display** | OLED 128×64, SSD1306, I2C |
| **GPS** | Quectel L76K, NMEA 0183, 9600 baud — connects to the onboard SH1.25-8Pin connector |
| **Power** | USB Type-C or Li-Ion battery (JST 1.25mm) |

---

## Quick Start

### 1. Dependencies (Arduino IDE)

Install via Library Manager:
- **RadioLib** — SX1262 radio driver
- **ESP8266 and ESP32 OLED driver for SSD1306** (ThingPulse) — display
- **TinyGPS++** — NMEA parser

Install via Boards Manager:
- **Heltec ESP32 Dev-Boards** (by Heltec Automation)

### 2. Board settings in Arduino IDE

| Setting | Value |
|---------|-------|
| Board | **WiFi LoRa 32(V3)** *(V4 is compatible)* |
| Upload Mode | **USB-OTG-CDC (TinyUSB)** |
| USB CDC On Boot | **Enabled** |

### 3. Firmware configuration

Before flashing each device, edit at the top of the file:

```cpp
#define DEVICE_NAME   "DLRP-01"   // Wi-Fi SSID — must be unique per device
#define WIFI_PASSWORD "1223334444" // Wi-Fi password
```

### 4. Flashing

Connect the board via USB, select the port, click Upload. Flashing with a battery connected is safe.

---

## Usage

### Connecting

1. After flashing, the board creates a Wi-Fi access point named `DEVICE_NAME`
2. Connect to it using `WIFI_PASSWORD`
3. Open a browser: **http://192.168.4.1**

### Range test

1. Flash two boards — assign one as **Sender** and the other as **Receiver** (or switch roles via the web UI)
2. Press **START** on both
3. The Sender transmits packets at the configured interval; the Receiver logs them with RSSI, SNR and GPS coordinates
4. After the test, press **SAVE CSV** to download the log

---

## Web Interface

```
┌─────────────────────────────────────────┐
│  [SENDER →]        [START / STOP]       │
│  [CLEAR LOG]       [SAVE CSV]           │
├─────────────────────────────────────────┤
│  Console                        P:0 R:--│
│  [0s] FW v1.0.0 | 868.000 MHz | SF9... │
│  [5s] Transmission STARTED             │
│  [10s] P:1 R:-87 D:Packet #1          │
├─────────────────────────────────────────┤
│  GPS                         Search     │
├─────────────────────────────────────────┤
│  LoRa Settings                          │
│  Frequency (Hz)         [868000000]     │
│  Spreading Factor            [SF9 ▼]   │
│  Bandwidth                [125 kHz ▼]  │
│  Coding Rate                  [4/5 ▼]  │
│  Preamble                         [8]  │
│  TX Power (dBm)                  [20]  │  ← Sender only
│  Interval (s)                   [5.0]  │  ← Sender only
└─────────────────────────────────────────┘
```

The console updates every 2 seconds. All LoRa setting changes are applied immediately and persisted to NVS.

---

## LoRa Parameters

| Parameter | Default | Range |
|-----------|---------|-------|
| Frequency | 868 MHz | 860–870 MHz |
| Spreading Factor | SF9 | SF7–SF12 |
| Bandwidth | 125 kHz | 125 / 250 / 500 kHz |
| Coding Rate | 4/5 | 4/5 – 4/8 |
| Preamble | 8 | 6–65535 |
| TX Power | 20 dBm | 0–22 dBm |
| Interval | 5 s | 1–60 s |

---

## Hardware Button (PRG / GPIO0)

| Action | Function |
|--------|----------|
| **1 click** | Start / Stop transmission or reception |
| **2 quick clicks** | Switch role Sender ↔ Receiver |
| **Hold 5 s** | Enter deep sleep |
| **Hold 5 s** (from sleep) | Wake up |

---

## OLED Display 128×64

```
SENDER                       SEND
P:42  R:-87  N:44
Loss:4%  SNR:9
GPS: Find 8 sat            87%
```

| Line | Content |
|------|---------|
| 1 | Role (SENDER / RECEIVER) · Status (SEND / LISTEN / STOP) |
| 2 | P: packets received · R: RSSI · N: last sender packet number |
| 3 | **Receiver:** packet loss % and SNR · **Sender:** interval and TX power |
| 4 | GPS status (Error / Search / Find N sat) · Battery voltage (V) |

---

## CSV Log Format

### Sender
```
Time,Packet,RSSI,Data,Lat,Lon,Satellites
17,1,-87,Packet #1,55.7558,37.6176,8
```

### Receiver
```
Time,Packet,RSSI,SNR,Data,Lat,Lon,Satellites
22,1,-91,9,Packet #1,55.7560,37.6180,7
```

`Time` is uptime in seconds. If GPS has no fix: `Lat=0.000000`, `Lon=0.000000`, `Satellites=0`. Log limit is 1000 rows; oldest entries are dropped when full.

---

## GPS

The **Quectel L76K** module connects to the onboard 8-pin SH1.25 connector on the V4 board (included with the board). Pin assignments confirmed from Meshtastic firmware for Heltec V4:

| Signal | GPIO |
|--------|------|
| ESP32 RX ← GPS TX | GPIO39 |
| ESP32 TX → GPS RX | GPIO38 |
| GPS power (P-MOS) | GPIO34 — `LOW` = GPS on |

GPS runs independently of start/stop state. Coordinates are recorded with every packet. Web UI status: **Error** (no NMEA data for >30 s) / **Search** (no fix) / **Find N sat** (fix acquired).

---

## Battery

The board supports Li-Ion / Li-Po batteries via the JST 1.25mm connector. Charging starts automatically when USB Type-C is connected (onboard TP4054, red LED = charging). Battery level is estimated from voltage via the onboard divider (GPIO1) and shown on the display.

> **Note:** For batteries with capacity ≤500 mAh, verify the board's charge current (default may be 200–500 mA). Recommended charge rate is ≤1C.

---

## Persistent Settings (NVS)

All settings are stored in non-volatile memory (namespace `lora-cfg`) and restored after reboot: role, frequency, SF, BW, CR, preamble, TX power, interval.

---

## Project Structure

```
LoRaRangeTester/
└── LoRaRangeTester.ino    — single firmware file
```
