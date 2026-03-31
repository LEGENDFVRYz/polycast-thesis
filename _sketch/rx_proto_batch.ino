// ===============================
// POLYCAST RECEIVER - ESP32 WROOM
// ===============================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
#define SERIAL_BAUD_RATE 115200
#define BATCH_SIZE 5

// ---------- STRUCTS (MUST MATCH SENDER) ----------
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
  float dist3;

  ImuSample samples[BATCH_SIZE];
};

Packet myPacket;
volatile bool newPacket = false;

uint32_t lastSeq = 0;
bool firstPacket = true;

// ---------- CALLBACK ----------
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData,
                int len) {

  if (len != sizeof(Packet)) return;

  memcpy(&myPacket, incomingData, sizeof(Packet));
  newPacket = true;
}

// ---------- SETUP ----------
void setup() {

  Serial.begin(SERIAL_BAUD_RATE);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    delay(1000);
    ESP.restart();
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Receiver Ready.");
}

// ---------- LOOP ----------
void loop() {

  if (newPacket) {

    newPacket = false;

    // Packet loss detection
    if (!firstPacket) {
      if (myPacket.seq != lastSeq + 1) {
        Serial.print("Packet loss detected! Missed: ");
        Serial.println(myPacket.seq - lastSeq - 1);
      }
    }

    firstPacket = false;
    lastSeq = myPacket.seq;

    // ---- CSV OUTPUT ----
    Serial.print(myPacket.seq); Serial.print(",");
    Serial.print(myPacket.batch_ts); Serial.print(",");
    Serial.print(myPacket.uwb_ts); Serial.print(",");

    Serial.print(myPacket.dist0, 3); Serial.print(",");
    Serial.print(myPacket.dist1, 3); Serial.print(",");
    Serial.print(myPacket.dist2, 3); Serial.print(",");
    Serial.print(myPacket.dist3, 3);

    for (int i = 0; i < BATCH_SIZE; i++) {
      Serial.print(",");

      Serial.print(myPacket.samples[i].qx, 4); Serial.print(",");
      Serial.print(myPacket.samples[i].qy, 4); Serial.print(",");
      Serial.print(myPacket.samples[i].qz, 4); Serial.print(",");
      Serial.print(myPacket.samples[i].qw, 4); Serial.print(",");

      Serial.print(myPacket.samples[i].ax, 2); Serial.print(",");
      Serial.print(myPacket.samples[i].ay, 2); Serial.print(",");
      Serial.print(myPacket.samples[i].az, 2); Serial.print(",");

      Serial.print(myPacket.samples[i].force); Serial.print(",");
      Serial.print(myPacket.samples[i].ts);
    }

    Serial.println();
  }
}
