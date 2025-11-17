// ###########################################
// RECEIVER (ESP32) - CORRECTED FOR NEW CORE
// ###########################################

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel

// DEFINE THE WIFI CHANNEL (Must be 1-13)
// BOTH SENDER AND RECEIVER MUST BE ON THE SAME CHANNEL
#define WIFI_CHANNEL 1

// Structure to receive
// This MUST match the sender's structure
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;

struct_message myData;

// -----------------------------------------------------------------
// CALLBACK FUNCTION - CORRECTED SIGNATURE
//
// The first argument is now 'const esp_now_recv_info_t * esp_now_info'
// -----------------------------------------------------------------
void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
  // Check if data length matches the structure size
  if (len < sizeof(myData)) {
    Serial.println("Received packet is too small. Ignoring.");
    return;
  }
  
  // Copy the data into our local structure
  memcpy(&myData, incomingData, sizeof(myData));

  // Get the MAC address from the new info struct
  const uint8_t *mac_addr = esp_now_info->src_addr;

  // Print who it came from
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  // Print the received data
  Serial.printf("From: %s | ", macStr);
  Serial.printf("Char: %s | ", myData.a);
  Serial.printf("Int: %d | ", myData.b);
  Serial.printf("Float: %.2f | ", myData.c);
  Serial.printf("Bool: %s\n", myData.d ? "true" : "false");
}

void setup() {
  Serial.begin(115200);

  // 1. Set WiFi Mode to Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Disconnect from any existing AP

  // Print this device's MAC address
  Serial.println("Receiver starting...");
  Serial.print("My MAC Address: ");
  Serial.println(WiFi.macAddress());

  // 2. Disable WiFi Power Save
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
    Serial.println("Error setting WiFi power save");
    return;
  }

  // 3. Set the WiFi Channel
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("Error setting WiFi channel");
    return;
  }

  // 4. Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // 5. Register the receive callback
  // This will now match the expected function signature
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.printf("ESP-NOW receiver ready. Listening on Channel %d\n", WIFI_CHANNEL);
}

void loop() {
  // This loop can be empty.
  delay(1000); 
}