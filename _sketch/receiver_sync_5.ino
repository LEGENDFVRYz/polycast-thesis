// SENDER CODE - ARDUINO NANO ESP32
// IMU - 100 Hz
// UWB - 20 Hz

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1

// --- STRUCTS (Must Match) ---
struct ImuSample {
  float qx, qy, qz, qw; 
  float ax, ay, az;     
  float force;          
  uint32_t ts;          
}; 

struct Packet {
  float dist0; 
  float dist1; 
  float dist2; 
  ImuSample samples[5]; 
}; 
Packet myPacket;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(Packet)) {
    Serial.println("Size Mismatch");
    return;
  }

  memcpy(&myPacket, incomingData, sizeof(Packet));

  // CSV Output: PKT_START, Dist0, Dist1, Dist2, [Sample 1...]
  // Serial.print("PKT_START,");
  Serial.print(myPacket.dist0); Serial.print(",");
  Serial.print(myPacket.dist1); Serial.print(",");
  Serial.print(myPacket.dist2);

  for (int i = 0; i < 5; i++) {
    Serial.print(",");
    Serial.print(myPacket.samples[i].qx); Serial.print(",");
    Serial.print(myPacket.samples[i].qy); Serial.print(",");
    Serial.print(myPacket.samples[i].qz); Serial.print(",");
    Serial.print(myPacket.samples[i].qw); Serial.print(",");
    Serial.print(myPacket.samples[i].ax); Serial.print(",");
    Serial.print(myPacket.samples[i].ay); Serial.print(",");
    Serial.print(myPacket.samples[i].az); Serial.print(",");
    Serial.print(myPacket.samples[i].force); Serial.print(",");
    Serial.print(myPacket.samples[i].ts);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK)
    delay(1000);
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Receiver Ready (3 Anchors).");
}

void loop() { 
    delay(100);
}