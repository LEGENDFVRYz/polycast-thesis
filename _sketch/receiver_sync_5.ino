// RECEIVER CODE - ESP32 WROOM
// IMU - 100 Hz | UWB - 20 Hz

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
#define SERIAL_BAUD_RATE 115200

// --- STRUCTS (Must Match Sender) ---
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
volatile bool newPacket = false;

// Callback function that runs when data is received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(Packet)) {
    // Determine if we should warn about size mismatch (optional)
    return;
  }
  memcpy(&myPacket, incomingData, sizeof(Packet));
  newPacket = true; // Signal to loop() that data is ready
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE); // Disable power saving for max performance
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // --- FIX APPLIED HERE: Added { } braces ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    delay(1000);
    ESP.restart();
  }
  // ------------------------------------------

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Receiver Ready. Waiting for packets...");
}

void loop() { 
  if (newPacket) {
    newPacket = false; // Reset flag so we don't print twice

    // --- CSV FORMAT OUTPUT ---
    // Header: DIST0, DIST1, DIST2
    Serial.print(myPacket.dist0, 3); Serial.print(",");
    Serial.print(myPacket.dist1, 3); Serial.print(",");
    Serial.print(myPacket.dist2, 3);

    // Loop through the 5 batched IMU samples
    for (int i = 0; i < 5; i++) {
      Serial.print(",");
      // Quaternions (4)
      Serial.print(myPacket.samples[i].qx, 4); Serial.print(",");
      Serial.print(myPacket.samples[i].qy, 4); Serial.print(",");
      Serial.print(myPacket.samples[i].qz, 4); Serial.print(",");
      Serial.print(myPacket.samples[i].qw, 4); Serial.print(",");
      // Acceleration (3)
      Serial.print(myPacket.samples[i].ax, 2); Serial.print(",");
      Serial.print(myPacket.samples[i].ay, 2); Serial.print(",");
      Serial.print(myPacket.samples[i].az, 2); Serial.print(",");
      // Force & Time
      Serial.print(myPacket.samples[i].force, 1); Serial.print(",");
      Serial.print(myPacket.samples[i].ts);
    }
    Serial.println(); // End the line
  }
}