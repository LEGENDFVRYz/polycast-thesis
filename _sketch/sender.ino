// ###########################################
// SENDER (ESP32 NANO) - CORRECTED
// ###########################################

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel

// DEFINE THE WIFI CHANNEL (Must be 1-13)
// BOTH SENDER AND RECEIVER MUST BE ON THE SAME CHANNEL
#define WIFI_CHANNEL 1

// Structure to send
typedef struct struct_sensor_data {
  float x;
  float y;
  float filtered_x;
  float filtered_y;
} struct_sensor_data;

struct_sensor_data myData;

// --------------------
// IMPORTANT: 
// Replace with your receiver's MAC address
// You can get this from the receiver's serial monitor
// --------------------
uint8_t receiverMAC[] = {0x80, 0xF3, 0xDA, 0x55, 0x9F, 0x6C};

esp_now_peer_info_t peerInfo;
unsigned long lastSend = 0;
unsigned long interval = 100; // 

// Callback for send status (CORRECTED Signature)
// This function is called when a packet is sent.
// For fast comms, only print on failure to avoid Serial bottleneck.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.print("Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  }
}

void setup() {
  Serial.begin(115200);

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

void loop() {
  unsigned long now = millis();
  if (now - lastSend > interval) {
    lastSend = now;

    // Generate random floats between 0.000 and 1.000 (like your example 0.450, 0.810)
    myData.x = random(0, 1001) / 1000.0f;
    myData.y = random(0, 1001) / 1000.0f;
    
    // Simulate filtered data (e.g., raw data + small random noise)
    myData.filtered_x = myData.x + (random(-50, 51) / 1000.0f); 
    myData.filtered_y = myData.y + (random(-50, 51) / 1000.0f);

    // Send the data
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t*)&myData, sizeof(myData));
    
    if (result != ESP_OK) {
      Serial.println("Error sending data");
    }
  }
}