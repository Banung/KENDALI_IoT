# 🌱 KENDALI — IoT Soil Monitoring Node

> **K**ondisi **E**kosistem **N**irkabel **D**ata **A**nalitik **L**ahan **I**ntelijen

Sistem monitoring kesehatan tanah berbasis IoT yang memungkinkan petani memantau kondisi lahan secara **real-time** langsung dari smartphone, **tanpa koneksi internet**, dan **tanpa listrik PLN**.

Dibuat untuk **Kompetisi SFT 2026 — Universitas Padjadjaran** 🏆

---

## ✨ Fitur Utama

| Fitur                           | Keterangan                                                         |
| ------------------------------- | ------------------------------------------------------------------ |
| 🌡️ **Suhu Tanah**               | DS18B20 via protokol OneWire, akurasi ±0.5°C                       |
| 💧 **Kelembapan Tanah**         | Sensor kapasitif (bukan resistif), tahan korosi                    |
| 🧪 **pH Tanah**                 | pH-4502C + probe BNC, range pH 0–14                                |
| ⚗️ **TDS & Konduktivitas (EC)** | Kandungan mineral (ppm) & konduktivitas (µS/cm)                    |
| 💡 **LED Peringatan Kritis**    | Menyala otomatis jika tanah terlalu kering                         |
| 📡 **BLE 5.0**                  | Kirim data JSON ke HP petani setiap 1 detik                        |
| 💾 **MicroSD Offline Buffer**   | Simpan riwayat saat HP terputus, auto-sync & flush saat tersambung |
| ☀️ **Off-Grid**                 | Bertenaga surya + baterai 18650, tidak butuh PLN                   |
| 🔬 **Simulasi Wokwi**           | Tersedia `diagram.json` untuk simulasi sebelum rakit hardware      |

---

## 🏗️ Arsitektur Sistem

```
☀️ Panel Surya 5V
      ↓
🔋 TP4056 Charger Module
      ↓
🔋 2× Baterai 18650 Li-Ion (paralel)
      ↓
🔄 Buck-Boost Converter → 5.0V stabil
      ↓
┌──────────────────────────────────────────────┐
│            🧠 ESP32-S3 DevKitC-1             │
│                                              │
│  GPIO 6  ← 💧 Soil Moisture Sensor (ADC)     │
│  GPIO 7  ← 🧪 pH-4502C Sensor (ADC)          │
│  GPIO 3  ← ⚗️  TDS & EC Sensor (ADC)          │
│  GPIO 4  ← 🌡️  DS18B20 Suhu (OneWire)        │
│  GPIO 5  → 💡 LED Alert Merah                │
│  SPI Bus ↔ 💾 Modul MicroSD (Offline Buffer) │
│            (CS:10, MOSI:11, SCK:12, MISO:13) │
│  Antena BLE 5.0 ↗️                           │
└──────────────────────────────────────────────┘
                ↓ BLE (Bluetooth Low Energy)
        📱 HP Petani — Aplikasi KENDALI
           (Dashboard real-time, auto-sync data)
```

---

## 🔧 Hardware & Wiring

| Komponen                      | Pin ESP32-S3      | Keterangan                      |
| ----------------------------- | ----------------- | ------------------------------- |
| DS18B20 (Suhu)                | GPIO 4            | OneWire + pull-up 4.7kΩ ke 3.3V |
| Capacitive Soil Moisture v1.2 | GPIO 6 (ADC1_CH5) | Tegangan 3.3V                   |
| pH-4502C (Pin Po)             | GPIO 7 (ADC1_CH6) | Board sensor pakai 5V           |
| Sensor TDS/EC                 | GPIO 3 (ADC1_CH2) | Tegangan 3.3V                   |
| LED Merah Alert               | GPIO 5            | Seri resistor 220Ω              |
| MicroSD Module (CS)           | GPIO 10           | Chip Select (SPI)               |
| MicroSD Module (MOSI)         | GPIO 11           | Master Out Slave In (SPI)       |
| MicroSD Module (SCK)          | GPIO 12           | Clock (SPI)                     |
| MicroSD Module (MISO)         | GPIO 13           | Master In Slave Out (SPI)       |
| MicroSD Module (VCC/GND)      | 5V / 3.3V & GND   | Sesuai spesifikasi modul SD     |

---

## 📡 Data BLE

Data dikirim sebagai JSON setiap **1 detik** via BLE NOTIFY:

```json
{
  "temp": 28.5,
  "soil": 45.2,
  "soilRaw": 1851,
  "ph": 6.8,
  "tds": 420.5,
  "ec": 675.3,
  "critical": false
}
```

| Parameter BLE            | Nilai                                  |
| ------------------------ | -------------------------------------- |
| Nama Perangkat           | `KENDALI_IoT`                          |
| Service UUID             | `4fafc201-1fb5-459e-8fcc-c5c9c331914b` |
| Characteristic UUID (TX) | `beb5483e-36e1-4688-b7f5-ea07361b26a8` |
| Mode                     | READ + NOTIFY                          |

Kompatibel dengan aplikasi **nRF Connect** atau **Serial Bluetooth Terminal**.

---

## 🚀 Cara Pakai

### Prasyarat

- [PlatformIO IDE](https://platformio.org/) (ekstensi VS Code) atau PlatformIO CLI
- Board: **ESP32-S3 DevKitC-1**

### 1. Clone repositori

```bash
git clone https://github.com/<username>/KENDALI.git
cd KENDALI
```

### 2. Build & Upload

```bash
# Build firmware
pio run

# Upload ke ESP32-S3 (pastikan board sudah terhubung via USB)
pio run --target upload

# Buka Serial Monitor untuk melihat data sensor
pio device monitor
```

> **Catatan:** Port USB akan terdeteksi otomatis. Jika tidak, tambahkan `upload_port = COMx` (Windows) atau `upload_port = /dev/ttyUSBx` (Linux/Mac) ke `platformio.ini` sesuai port kamu.

### 3. Simulasi Wokwi (tanpa hardware)

Buka proyek ini di [Wokwi](https://wokwi.com/) menggunakan `diagram.json` dan `wokwi.toml` yang sudah tersedia. Putar potentiometer untuk mensimulasikan nilai sensor.

---

## 📁 Struktur Folder

```
KENDALI/
├── src/
│   └── main.cpp                          # Firmware utama ESP32-S3
├── diagram.json                          # Skema simulasi Wokwi
├── wokwi.toml                            # Konfigurasi simulator Wokwi
├── platformio.ini                        # Konfigurasi build PlatformIO
└── README.md
```

---

## 🔬 Tentang Sensor & Formula

### Kelembapan Tanah

```
Kelembapan (%) = (ADC / 4095) × 100

ADC < 1200    → KERING  🔴 (LED alert menyala)
ADC 1200–2800 → NORMAL  🟢
ADC > 2800    → BASAH   🔵
```

### pH Tanah

```
pH = (ADC × 14.0) / 4095

pH < 6.5    → ASAM   (perlu pengapuran)
pH 6.5–7.5  → NETRAL ✅ (ideal)
pH > 7.5    → BASA   (perlu penyesuaian)
```

### TDS & Konduktivitas (dari regresi linear kalibrasi empiris)

```
EC  (µS/cm) = (0.2142 × avgADC) + 494.93
TDS (ppm)   = (0.3417 × avgADC) + 281.08
```

> Nilai TDS diambil dari **moving average 20 sampel** setiap 50ms untuk mengurangi noise sensor analog.

---

## 📄 Lisensi

MIT License — bebas digunakan dan dimodifikasi dengan atribusi.

---

_Dibuat dengan ❤️ untuk petani Indonesia — SFT 2026, hampirgalulusgaragarastempel Universitas Padjadjaran_
