// Receiver: ESP32 WROOM (Serial Bridge)
// ---------------------------------------------------------
// This code receives ESP-NOW packets and wraps them in a 
// binary frame compatible with your 'SerialStreamer' Python script.
//
// FRAME FORMAT:
// [0xAA] [0x55] [LEN] [TS_0] [TS_1] [TS_2] [TS_3] [DATA...] [0xFF]
// ---------------------------------------------------------

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1

// --- CALLBACK: DATA RECEIVED ---
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  
  // 1. Write Header (2 Bytes)
  // Matches Python: if self.buffer[0] != 0xAA or self.buffer[1] != 0x55:
  Serial.write(0xAA);
  Serial.write(0x55);

  // 2. Write Length (1 Byte)
  // The Python script expects the length to cover the Data + 4 bytes of Receiver Timestamp
  // Python logic: payload_len = self.buffer[2]
  uint8_t totalPayloadLen = len + 4; 
  Serial.write(totalPayloadLen);

  // 3. Write Receiver Timestamp (4 Bytes)
  // Python logic: data_payload = frame[7:-1] (The header takes 7 bytes total before data starts)
  uint32_t currentTs = millis();
  Serial.write((uint8_t*)&currentTs, 4);

  // 4. Write Actual Packet Data (N Bytes)
  // Matches Sender's Structs (IMU or UWB)
  Serial.write(incomingData, len);

  // 5. Write Footer (1 Byte)
  // Matches Python: if self.buffer[total_frame - 1] != 0xFF:
  Serial.write(0xFF);
}

void setup() {
  // MUST match Python baud rate (self.baud = 115200)
  Serial.begin(115200);

  // Setup WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    // If we can't init, we can't do anything. Blink LED or print error.
    return;
  }

  // Register Callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // No logic needed here; everything is interrupt-driven by OnDataRecv
  delay(1000); 
}