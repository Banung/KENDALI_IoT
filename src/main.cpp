#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include "driver/gpio.h"

// PIN DEFINITIONS - SENSORS & ACTUATORS
#define PIN_DS18B20 4   // OneWire Digital Pin (DS18B20 Temperature)
#define PIN_SOIL 6      // Analog ADC1_CH5 (Capacitive Soil Moisture)
#define PIN_PH 7        // Analog ADC1_CH6 (pH Tanah - Probe Buram)
#define PIN_DMS_PH 14   // Digital Output (Kontrol DMS modul pengkondisi pH tanah)
#define PIN_TDS 3       // Analog ADC1_CH2 (TDS & Conductivity)
#define PIN_LED_ALERT 5 // Digital Output (Lampu Peringatan)

// PIN DEFINITIONS - SPI MICROSD CARD (ESP32-S3)
#define PIN_SD_CS 10   // Chip Select MicroSD
#define PIN_SD_MOSI 11 // Master Out Slave In
#define PIN_SD_SCK 12  // Serial Clock
#define PIN_SD_MISO 13 // Master In Slave Out

// pH TANAH (Probe Buram + modul pengkondisi DMS) KALIBRASI
// Regresi linier dari vendor, dikalibrasi untuk ADC 10-bit (0–1023).
#define PH_SLOPE (-0.0333f)      // Slope regresi ADC -> pH
#define PH_INTERCEPT 9.8340f     // Intercept regresi ADC -> pH
#define PH_ADC12_TO_10 4.0f      // Konversi ADC 12-bit -> ekuivalen 10-bit
#define PH_DMS_WARMUP_MS 10000UL // DMS ON -> tunggu stabil sebelum baca
#define PH_DMS_REST_MS 3000UL    // DMS OFF -> istirahat sebelum siklus berikut

// BLE DEFINITIONS
#define BLE_DEVICE_NAME "KENDALI_IoT"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristicTx = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// MICROSD OFFLINE BUFFER DEFINITIONS
const char *OFFLINE_DATA_FILE = "/offline_data.txt";
bool sdAvailable = false;

// pH TANAH STATE (siklus non-blocking via modul DMS)
enum PhCycleState
{
  PH_RESTING, // DMS OFF, jeda istirahat
  PH_WARMING  // DMS ON, menunggu sensor stabil
};
PhCycleState phCycleState = PH_RESTING;
unsigned long phCycleSince = 0;
int phRaw = 0;
float phValue = 0.0f;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("[BLE] HP Terhubung!");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("[BLE] HP Terputus.");
  }
};

// OBJECTS & VARIABLES
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensorDS18B20(&oneWire);

// Moving Average Buffer for TDS / Conductivity
const int NUM_TDS_SAMPLES = 20;
int tdsSamples[NUM_TDS_SAMPLES];
int tdsSampleIdx = 0;
long tdsSampleSum = 0;

// Timing intervals
unsigned long lastDashboardTime = 0;
unsigned long lastTdsSampleTime = 0;
unsigned long lastOfflineLogTime = 0;
const unsigned long DASHBOARD_INTERVAL = 1000; // Pembacaan sensor & refresh Serial setiap 1 detik
const unsigned long TDS_SAMPLE_INTERVAL = 50;  // Sample TDS setiap 50ms

// Interval pencatatan ke MicroSD saat HP terputus
// Saat ini diatur 1000ms (1 detik) untuk tahap debugging.
// Untuk produksi jangka panjang di lahan, ubah ke: 600000UL (10 menit)
const unsigned long OFFLINE_LOG_INTERVAL = 1000;

// FUNCTION DECLARATIONS
void initBLE();
bool initSD();
bool sdSpiProbe();
void saveOfflineData(const char *payload);
void syncOfflineData();
void updateTdsBuffer();
float getAverageTdsADC();
void updatePhSensor();
void buildTelemetryJson(char *buffer, size_t maxLen, float temp, float soilPct, int soilRaw, float phVal, float tdsPpm, float ec, bool isCritical);
void printDashboard(float temp, int soilRaw, float soilPct, const char *soilStatus, bool isCritical,
                    int phRaw, float phVal, const char *phStatus,
                    float avgTdsADC, float conductivity, float tdsPpm);
void sendBleTelemetry(const char *payload);

void setup()
{
  // For Terminal Output / Debugging
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("   KENDALI IoT - ESP32-S3 SENSOR NODE   ");
  Serial.println("========================================");

  // Warning LED
  pinMode(PIN_LED_ALERT, OUTPUT);
  digitalWrite(PIN_LED_ALERT, LOW);
  Serial.println("[OK] Lampu Peringatan Siap (GPIO 5 - Seri 220 Ohm)");

  // DS18B20 Temperature Sensor
  sensorDS18B20.begin();
  Serial.println("[OK] DS18B20 Suhu Siap (GPIO 4 - Pull-Up 4.7k)");

  // ADC Resolution
  analogReadResolution(12);
  Serial.println("[OK] ADC Analog Siap (Soil: GPIO 6, pH: GPIO 7, TDS: GPIO 3)");

  // Soil pH Sensor (Probe Buram + DMS conditioning module)
  pinMode(PIN_DMS_PH, OUTPUT);
  digitalWrite(PIN_DMS_PH, HIGH); // DMS OFF saat boot
  Serial.println("[OK] pH Tanah Siap (ADC GPIO 7 - DMS GPIO 14)");

  // TDS buffer
  for (int i = 0; i < NUM_TDS_SAMPLES; i++)
  {
    tdsSamples[i] = 0;
  }

  // MicroSD Card Init
  sdAvailable = initSD();

  // Bluetooth Low Energy
  initBLE();
  Serial.println("----------------------------------------\n");
}

void loop()
{
  unsigned long currentMillis = millis();

  // Update siklus pH tanah (non-blocking)
  updatePhSensor();

  // Handle BLE reconnection when disconnected
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // Berikan jeda stack Bluetooth
    pServer->startAdvertising(); // Mulai broadcast ulang
    Serial.println("[BLE] Memulai advertising ulang, menunggu koneksi...");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
    // HP baru saja terhubung: sinkronisasi data offline dari MicroSD ke HP
    syncOfflineData();
  }

  // Periodically sample TDS to maintain smooth moving average
  if (currentMillis - lastTdsSampleTime >= TDS_SAMPLE_INTERVAL)
  {
    lastTdsSampleTime = currentMillis;
    updateTdsBuffer();
  }

  // Read Sensors & Send Telemetry every 1 second
  if (currentMillis - lastDashboardTime >= DASHBOARD_INTERVAL)
  {
    lastDashboardTime = currentMillis;

    // Read DS18B20
    sensorDS18B20.requestTemperatures();
    float tempC = sensorDS18B20.getTempCByIndex(0);

    // Read Capacitive Soil Moisture & Alert Logic
    int soilRaw = analogRead(PIN_SOIL);
    float soilPercent = ((float)soilRaw / 4095.0) * 100.0;

    // Kondisi Kritis jika tanah KERING atau SANGAT BASAH
    bool isCritical = (soilRaw < 1200);
    const char *soilStatus = (soilRaw < 1200) ? "KERING" : ((soilRaw > 2800) ? "BASAH" : "NORMAL");

    // Kontrol Lampu Peringatan
    digitalWrite(PIN_LED_ALERT, isCritical ? HIGH : LOW);

    // pH Tanah — nilai terakhir hasil siklus DMS (diperbarui di updatePhSensor())
    const char *phStatus;
    if (phValue < 6.5)
    {
      phStatus = "ASAM";
    }
    else if (phValue <= 7.5)
    {
      phStatus = "NETRAL";
    }
    else
    {
      phStatus = "BASA";
    }

    // Read TDS & Conductivity
    float avgTdsADC = getAverageTdsADC();
    float conductivity = (0.2142 * avgTdsADC) + 494.93; // uS/cm
    float tdsPpm = (0.3417 * avgTdsADC) + 281.08;       // ppm

    // Output ke Serial Monitor USB
    printDashboard(tempC, soilRaw, soilPercent, soilStatus, isCritical,
                   phRaw, phValue, phStatus,
                   avgTdsADC, conductivity, tdsPpm);

    // Format payload JSON telemetri
    char payload[160];
    buildTelemetryJson(payload, sizeof(payload), tempC, soilPercent, soilRaw, phValue, tdsPpm, conductivity, isCritical);

    // Jika HP terhubung, kirim secara real-time via BLE
    if (deviceConnected)
    {
      sendBleTelemetry(payload);
    }
    // Jika HP tidak terhubung, simpan ke MicroSD Card sesuai interval offline
    else
    {
      if (currentMillis - lastOfflineLogTime >= OFFLINE_LOG_INTERVAL)
      {
        lastOfflineLogTime = currentMillis;
        saveOfflineData(payload);
      }
    }
  }
}

// HELPER FUNCTIONS
bool initSD()
{
  Serial.println("[SD] Init MicroSD (SPI SCK=GPIO12, MISO=GPIO13, MOSI=GPIO11, CS=GPIO10)...");

  // CS idle HIGH sebelum init. ESP32-S3-DevKitC-1 TIDAK menyediakan pull-up
  // eksternal pada bus SPI; internal pull-up (~45k) hanya membantu, belum tentu cukup.
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  // SPI.begin() memasang MISO ke periferal sehingga pull-up dari pinMode() bisa hilang.
  // Aktifkan pull-up via GPIO pad API (tak mengubah fungsi periferal).
  // ESP32-S3-DevKitC-1 tidak punya pull-up eksternal di bus SPI; internal (~45k) sering
  // belum cukup -> MISO floating -> "Select Failed". Idealnya tetap tambah 10k ke 3.3V.
  gpio_set_pull_mode((gpio_num_t)PIN_SD_MISO, GPIO_PULLUP_ONLY);

  // Clock rendah saat probing. Catatan: SD lib tetap pakai 400 kHz ketika init.
  if (!SD.begin(PIN_SD_CS, SPI, 400000))
  {
    Serial.println("[WARN] SD.begin() gagal.");
    if (sdSpiProbe())
    {
      Serial.println("[SD] Komunikasi SPI OK (kartu menjawab CMD0).");
      Serial.println("[SD] -> Kecurigaan FILESYSTEM: format kartu FAT32? (exFAT/NTFS tidak didukung)");
    }
    else
    {
      Serial.println("[SD] Tidak ada respons SPI dari kartu.");
      Serial.println("[SD] -> Cek catu daya VCC/GND, MISO/MOSI/SCK/CS, pull-up 10k, level tegangan modul.");
    }
    Serial.println("[WARN] Sistem tetap berjalan normal tanpa buffer offline.");
    return false;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("[WARN] Slot modul terdeteksi tapi kartu MicroSD belum dimasukkan.");
    return false;
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("[OK] MicroSD Card Siap! Kapasitas: %llu MB\r\n", cardSize);
  return true;
}

// Diagnostik: kirim perintah CMD0 mentah. Balasan 0x01 = kartu hidup di mode SPI.
bool sdSpiProbe()
{
  digitalWrite(PIN_SD_CS, HIGH);
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  for (int i = 0; i < 10; i++)
  {
    SPI.transfer(0xFF); // >74 clock dengan CS HIGH -> kartu masuk mode SPI
  }

  digitalWrite(PIN_SD_CS, LOW);
  const uint8_t cmd0[6] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
  for (uint8_t b : cmd0)
  {
    SPI.transfer(b);
  }

  uint8_t r = 0xFF;
  for (int i = 0; i < 8 && (r & 0x80); i++)
  {
    r = SPI.transfer(0xFF);
  }

  digitalWrite(PIN_SD_CS, HIGH);
  SPI.transfer(0xFF);
  SPI.endTransaction();

  return r == 0x01;
}

void saveOfflineData(const char *payload)
{
  if (!sdAvailable)
  {
    return;
  }

  File file = SD.open(OFFLINE_DATA_FILE, FILE_APPEND);
  if (file)
  {
    file.println(payload);
    file.close();
    Serial.println("[SD] Data telemetri offline tersimpan ke MicroSD.");
  }
  else
  {
    Serial.println("[SD ERROR] Gagal membuka file untuk menyimpan data offline!");
  }
}

void syncOfflineData()
{
  if (!sdAvailable)
  {
    return;
  }

  if (!SD.exists(OFFLINE_DATA_FILE))
  {
    Serial.println("[SD] Tidak ada data riwayat offline yang perlu disinkronkan.");
    return;
  }

  File file = SD.open(OFFLINE_DATA_FILE, FILE_READ);
  if (!file)
  {
    Serial.println("[SD ERROR] Gagal membuka file data offline!");
    return;
  }

  Serial.println("\n[SD -> BLE] Memulai sinkronisasi data riwayat offline ke HP...");
  int recordCount = 0;

  while (file.available() && deviceConnected)
  {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
    {
      pCharacteristicTx->setValue((uint8_t *)line.c_str(), line.length());
      pCharacteristicTx->notify();
      recordCount++;
      delay(35); // Jeda singkat agar buffer BLE stack stabil & tidak drop
    }
  }
  file.close();

  // Bersihkan data yang sudah dikirim jika HP masih terhubung
  if (deviceConnected)
  {
    SD.remove(OFFLINE_DATA_FILE);
    Serial.printf("[SD -> BLE] Sinkronisasi selesai! %d baris data terkirim, file buffer dibersihkan.\n\n", recordCount);
  }
  else
  {
    Serial.println("[SD -> BLE] Koneksi HP terputus di tengah sinkronisasi! File disimpan untuk dicoba lagi.\n");
  }
}

void initBLE()
{
  Serial.println("[BLE] Menginisialisasi Bluetooth Low Energy...");
  BLEDevice::init(BLE_DEVICE_NAME);

  // Buat BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Buat BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Buat BLE Characteristic untuk transmisi telemetri
  pCharacteristicTx = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);

  // BLE2902 descriptor agar aplikasi Android/iOS bisa enable notification
  pCharacteristicTx->addDescriptor(new BLE2902());

  // Mulai Service
  pService->start();

  // Mulai Advertising agar HP Petani bisa menemukan perangkat
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.print("[OK] BLE Siap! Nama Perangkat: ");
  Serial.println(BLE_DEVICE_NAME);
}

void updateTdsBuffer()
{
  int newReading = analogRead(PIN_TDS);
  tdsSampleSum -= tdsSamples[tdsSampleIdx];
  tdsSamples[tdsSampleIdx] = newReading;
  tdsSampleSum += newReading;
  tdsSampleIdx = (tdsSampleIdx + 1) % NUM_TDS_SAMPLES;
}

float getAverageTdsADC()
{
  return (float)tdsSampleSum / (float)NUM_TDS_SAMPLES;
}

// Siklus pembacaan pH tanah non-blocking sesuai workflow vendor:
// DMS ON -> tunggu 10s stabil -> baca ADC -> konversi regresi -> DMS OFF -> tunggu 3s.
void updatePhSensor()
{
  unsigned long now = millis();

  if (phCycleState == PH_RESTING)
  {
    if (now - phCycleSince >= PH_DMS_REST_MS)
    {
      digitalWrite(PIN_DMS_PH, LOW); // DMS ON (active LOW)
      phCycleState = PH_WARMING;
      phCycleSince = now;
    }
    return;
  }

  // PH_WARMING
  if (now - phCycleSince >= PH_DMS_WARMUP_MS)
  {
    phRaw = analogRead(PIN_PH);

    // Formula vendor dikalibrasi untuk ADC 10-bit; konversi dari 12-bit.
    float adc10 = (float)phRaw / PH_ADC12_TO_10;
    float value = (PH_SLOPE * adc10) + PH_INTERCEPT;

    // Di luar rentang valid pH (0–14) -> dianggap tidak valid.
    if (value < 0.0f || value > 14.0f)
    {
      value = 0.0f;
    }
    phValue = value;

    digitalWrite(PIN_DMS_PH, HIGH); // DMS OFF
    phCycleState = PH_RESTING;
    phCycleSince = now;
  }
}

void buildTelemetryJson(char *buffer, size_t maxLen, float temp, float soilPct, int soilRaw, float phVal, float tdsPpm, float ec, bool isCritical)
{
  snprintf(buffer, maxLen,
           "{\"temp\":%.2f,\"soil\":%.1f,\"soilRaw\":%d,\"ph\":%.2f,\"tds\":%.1f,\"ec\":%.1f,\"critical\":%s}",
           temp, soilPct, soilRaw, phVal, tdsPpm, ec, isCritical ? "true" : "false");
}

void printDashboard(float temp, int soilRaw, float soilPct, const char *soilStatus, bool isCritical,
                    int phRaw, float phVal, const char *phStatus,
                    float avgTdsADC, float conductivity, float tdsPpm)
{
  Serial.println("--- DATA SENSOR (KENDALI) ---");
  Serial.printf("Suhu              : %.2f °C\r\n", temp);
  Serial.printf("Soil Moisture     : %.1f %% (ADC: %d) [%s]\r\n", soilPct, soilRaw, soilStatus);
  Serial.printf("Status Peringatan : %s\r\n", isCritical ? "KRITIS (LAMPU ON!)" : "AMAN (LAMPU OFF)");
  Serial.printf("pH                : %.2f (ADC: %d) [%s]\r\n", phVal, phRaw, phStatus);
  Serial.printf("TDS               : %.1f ppm (Avg ADC: %.1f)\r\n", tdsPpm, avgTdsADC);
  Serial.printf("Conductivity      : %.1f uS/cm\r\n", conductivity);
  Serial.printf("Status MicroSD    : %s\r\n", sdAvailable ? "Aktif & Siap" : "Tidak Terdeteksi / Nonaktif");
  Serial.printf("Status BLE        : %s\r\n", deviceConnected ? "Terhubung ke HP" : "Menunggu Koneksi");
  Serial.println();
}

void sendBleTelemetry(const char *payload)
{
  if (!deviceConnected)
  {
    return;
  }

  pCharacteristicTx->setValue((uint8_t *)payload, strlen(payload));
  pCharacteristicTx->notify();
}