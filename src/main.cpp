#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// PIN DEFINITIONS
#define PIN_DS18B20 4   // OneWire Digital Pin (DS18B20 Temperature)
#define PIN_SOIL 1      // Analog ADC1_CH0 (Capacitive Soil Moisture)
#define PIN_PH 2        // Analog ADC1_CH1 (pH-4502C Po Pin)
#define PIN_TDS 3       // Analog ADC1_CH2 (TDS & Conductivity)
#define PIN_LED_ALERT 5 // Digital Output (Lampu Peringatan)

// BLE DEFINITIONS
#define BLE_DEVICE_NAME "KENDALI_IoT"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristicTx = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

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
const unsigned long DASHBOARD_INTERVAL = 1000; // Kirim data setiap 1 detik
const unsigned long TDS_SAMPLE_INTERVAL = 50;  // Sample TDS setiap 50ms

// FUNCTION DECLARATIONS
void initBLE();
void updateTdsBuffer();
float getAverageTdsADC();
void printDashboard(float temp, int soilRaw, float soilPct, const char *soilStatus, bool isCritical,
                    int phRaw, float phVal, const char *phStatus,
                    float avgTdsADC, float conductivity, float tdsPpm);
void sendBleTelemetry(float temp, float soilPct, int soilRaw, float phVal, float tdsPpm, float ec, bool isCritical);

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
  Serial.println("[OK] ADC Analog Siap (Soil: GPIO 1, pH: GPIO 2, TDS: GPIO 3)");

  // TDS buffer
  for (int i = 0; i < NUM_TDS_SAMPLES; i++)
  {
    tdsSamples[i] = 0;
  }

  // Bluetooth Low Energy
  initBLE();
  Serial.println("----------------------------------------\n");
}

void loop()
{
  unsigned long currentMillis = millis();

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

    // Read pH Sensor
    int phRaw = analogRead(PIN_PH);
    // Formula pendekatan pH (skala 0 - 14 dari tegangan ADC 12-bit)
    float phValue = ((float)phRaw * 14.0) / 4095.0;
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

    // Kirim Data via BLE ke Aplikasi KENDALI di HP Petani
    sendBleTelemetry(tempC, soilPercent, soilRaw, phValue, tdsPpm, conductivity, isCritical);
  }
}

// HELPER FUNCTIONS
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
  Serial.printf("Status BLE        : %s\r\n", deviceConnected ? "Terhubung ke HP" : "Menunggu Koneksi");
  Serial.println();
}

void sendBleTelemetry(float temp, float soilPct, int soilRaw, float phVal, float tdsPpm, float ec, bool isCritical)
{
  if (!deviceConnected)
  {
    return; // Tidak ada HP yang tersambung, hemat siklus pengiriman
  }

  // Format JSON
  char payload[160];
  snprintf(payload, sizeof(payload),
           "{\"temp\":%.2f,\"soil\":%.1f,\"soilRaw\":%d,\"ph\":%.2f,\"tds\":%.1f,\"ec\":%.1f,\"critical\":%s}",
           temp, soilPct, soilRaw, phVal, tdsPpm, ec, isCritical ? "true" : "false");

  pCharacteristicTx->setValue((uint8_t *)payload, strlen(payload));
  pCharacteristicTx->notify();
}