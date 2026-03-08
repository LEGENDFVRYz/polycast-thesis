// ===============================
// POLYCAST SENDER - NANO ESP32
// IMU: 100 Hz
// UWB: ~20 Hz
// 5 IMU : 1 UWB Batching
// ===============================

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ---------- CONFIG ----------
const uint8_t RECEIVER_MAC[] = {0x80, 0xF3, 0xDA, 0x55, 0x9F, 0x6C};
#define WIFI_CHANNEL 1
#define FORCE_PIN A0
#define UWB_RX 9
#define UWB_TX 8
#define BATCH_SIZE 5

// ---------- TIMING ----------
const unsigned long INTERVAL_US = 10000; // 10ms (100 Hz)
unsigned long lastTime = 0;

// ---------- STRUCTS ----------
struct ImuSample {
  float qx, qy, qz, qw;
  float ax, ay, az;
  float force;
  uint32_t ts;
};

struct Packet {
  uint32_t seq;
  uint32_t batch_ts;
  uint32_t uwb_ts;

  float dist0;
  float dist1;
  float dist2;

  ImuSample samples[BATCH_SIZE];
};

Packet myPacket;

// ---------- GLOBALS ----------
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float latest_qx, latest_qy, latest_qz, latest_qw;
float latest_ax, latest_ay, latest_az;
float latest_d0 = 0.0, latest_d1 = 0.0, latest_d2 = 0.0;

uint32_t latest_uwb_ts = 0;
uint32_t packetSeq = 0;
uint32_t batchStartTs = 0;
int sampleCounter = 0;

// ---------- UWB DECODER ----------
void pollUWB() {
  static uint8_t buf[256];
  static int idx = 0;
  static bool started = false;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (!started) {
      if (b == 0xAA) {
        started = true;
        idx = 0;
        buf[idx++] = b;
      }
    } else {
      buf[idx++] = b;

      if (idx >= 64) {
        started = false;
        idx = 0;
      }

      if (idx >= 35) {
        if (buf[1] == 0x25 && buf[2] == 0x01) {

          uint32_t r0 = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) |
                        ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 24);

          uint32_t r1 = (uint32_t)buf[7] | ((uint32_t)buf[8] << 8) |
                        ((uint32_t)buf[9] << 16) | ((uint32_t)buf[10] << 24);

          uint32_t r2 = (uint32_t)buf[11] | ((uint32_t)buf[12] << 8) |
                        ((uint32_t)buf[13] << 16) | ((uint32_t)buf[14] << 24);

          if (r0 > 0) latest_d0 = r0 / 1000.0f;
          if (r1 > 0) latest_d1 = r1 / 1000.0f;
          if (r2 > 0) latest_d2 = r2 / 1000.0f;

          latest_uwb_ts = micros();  // Timestamp UWB update
        }

        started = false;
        idx = 0;
      }
    }
  }
}

// ---------- IMU ----------
void pollIMU() {

  while (bno08x.wasReset()) {
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
  }

  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      latest_qx = sensorValue.un.rotationVector.i;
      latest_qy = sensorValue.un.rotationVector.j;
      latest_qz = sensorValue.un.rotationVector.k;
      latest_qw = sensorValue.un.rotationVector.real;
    }
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      latest_ax = sensorValue.un.linearAcceleration.x;
      latest_ay = sensorValue.un.linearAcceleration.y;
      latest_az = sensorValue.un.linearAcceleration.z;
    }
  }
}

// ---------- SETUP ----------
void setup() {

  Serial.begin(115200);

  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, UWB_RX, UWB_TX);

  Wire.begin();
  pinMode(FORCE_PIN, INPUT);

  while (!bno08x.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO085 not detected. Retrying...");
    delay(100);
  }

  bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    delay(1000);
    ESP.restart();
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

// ---------- LOOP ----------
void loop() {

  pollIMU();
  pollUWB();

  unsigned long now = micros();

  if (now - lastTime >= INTERVAL_US) {

    lastTime = now;

    if (sampleCounter == 0) {
      batchStartTs = now;
    }

    int rawForce = analogRead(FORCE_PIN);
    float forceVal = (rawForce / 4095.0f) * 100.0f;

    myPacket.samples[sampleCounter] = {
      latest_qx, latest_qy, latest_qz, latest_qw,
      latest_ax, latest_ay, latest_az,
      forceVal,
      now
    };

    sampleCounter++;

    if (sampleCounter >= BATCH_SIZE) {

      myPacket.seq = packetSeq++;
      myPacket.batch_ts = batchStartTs;
      myPacket.uwb_ts = latest_uwb_ts;

      myPacket.dist0 = latest_d0;
      myPacket.dist1 = latest_d1;
      myPacket.dist2 = latest_d2;

      esp_now_send(RECEIVER_MAC, (uint8_t *)&myPacket, sizeof(myPacket));

      sampleCounter = 0;
    }
  }
}
