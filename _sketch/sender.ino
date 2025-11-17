// ###########################################
// SENDER (ESP32 NANO) - UWB 
// ###########################################

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel

// DEFINE THE WIFI CHANNEL (Must be 1-13)
// BOTH SENDER AND RECEIVER MUST BE ON THE SAME CHANNEL
#define WIFI_CHANNEL 1

// UWB UART Pins
#define UWB_RX 9
#define UWB_TX 8  

// Structure to send
typedef struct struct_sensor_data {
  float x;
  float y;
  float filtered_x;
  float filtered_y;
} struct_sensor_data;

struct_sensor_data myData;


// -----------------------------------------
// UWB Distance Storage (FROM REFERENCE)
// -----------------------------------------
float distances[8];   // Raw distances
float uwbX = 0, uwbY = 0;       // UWB multilateration output
float uwbXf = 0, uwbYf = 0;      // Filtered outputs


// --------------------
// RECEIVER MAC ADDRESS
// --------------------
uint8_t receiverMAC[] = {0x80, 0xF3, 0xDA, 0x55, 0x9F, 0x6C};
esp_now_peer_info_t peerInfo;
// unsigned long lastSend = 0;    // 
// unsigned long interval = 100;  // will now depend on sensor capability


// -----------------------------------------
// UWB Decode Function (FROM REFERENCE)
// -----------------------------------------
bool decodeUwbDistances(uint8_t* data, int dataLen, float* distances) {
  for (int i = 0; i < 8; i++) distances[i] = -1;

  if (dataLen < 35) return false;
  if (data[0] != 0xAA || data[1] != 0x25 || data[2] != 0x01) return false;

  for (int i = 0; i < 8; i++) {
    int offset = 3 + (i * 4);
    uint32_t raw =
      data[offset] |
      (data[offset+1] << 8) |
      (data[offset+2] << 16) |
      (data[offset+3] << 24);

    if (raw > 0) distances[i] = raw / 1000.0;
  }

  return true;
}

// -----------------------------------------
// UWB Position Compute (FROM REFERENCE)
// (This is where your solver/filter logic lives)
// -----------------------------------------
void computeUwbPosition() {
  // Example: Replace these with your calculated values
  uwbX = distances[0];
  uwbY = distances[1];

  // Filter example (replace with Kalman/LPF)
  uwbXf = 0.8 * uwbXf + 0.2 * uwbX;
  uwbYf = 0.8 * uwbYf + 0.2 * uwbY;
}

// Callback for send status
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.print("Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  }
}




void setup() {
  Serial.begin(115200);

  // *** ADDED: Initialize Serial for UWB Module ***
  Serial1.begin(115200, SERIAL_8N1, UWB_RX, UWB_TX);

  // 1. Set WiFi Mode to Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Disconnect from any existing AP

  // 2. Disable WiFi Power Save (IMPORTANT for Low Latency)
  // ESP_OK success code is 0
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
    Serial.println("Error setting WiFi power save");
    return;
  }

  // 3. Set the WiFi Channel (IMPORTANT for Reliability)
  // Both sender and receiver MUST be on the same channel
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("Error setting WiFi channel");
    return;
  }

  // 4. Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // 5. Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // 6. Register peer
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = WIFI_CHANNEL; // Use the same channel
  peerInfo.ifidx = WIFI_IF_STA;   // Set the interface
  peerInfo.encrypt = false;       // No encryption for fastest speed

  // 7. Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.printf("ESP-NOW sender ready. Sending to %02X:%02X:%02X:%02X:%02X:%02X on Channel %d\n",
                receiverMAC[0], receiverMAC[1], receiverMAC[2], receiverMAC[3], receiverMAC[4], receiverMAC[5], WIFI_CHANNEL);
}



// -----------------------------------------
// Main Logic: UWB Position Compute 
// This loop now waits for UWB data and sends it via ESP-NOW
// -----------------------------------------
void loop() {
  
  // Static buffer variables from reference code
  static uint8_t buffer[256];
  static int index = 0;
  static bool started = false;

  // Check for UWB data
  while (Serial1.available()) {
    uint8_t in = Serial1.read();

    if (!started && in == 0xAA) {
      started = true;
      index = 0;
      buffer[index++] = in;
    }
    else if (started) {
      buffer[index++] = in;

      // Check if we have the full UWB packet (35 bytes)
      if (index >= 35) {
        bool good = decodeUwbDistances(buffer, index, distances);

        if (good) {
          // Calculate the X, Y, and filtered positions
          computeUwbPosition();

          // *** NEW: Populate the ESP-NOW struct ***
          myData.x = uwbX;
          myData.y = uwbY;
          myData.filtered_x = uwbXf;
          myData.filtered_y = uwbYf;

          // *** NEW: Send the real data via ESP-NOW ***
          esp_err_t result = esp_now_send(receiverMAC, (uint8_t*)&myData, sizeof(myData));
          
          if (result != ESP_OK) {
            Serial.println("Error sending data");
          }
        }

        // Reset for next packet
        started = false;
        index = 0;
      }

      // Buffer overflow guard
      if (index >= 256) {
        started = false;
        index = 0;
      }
    }
  }
}