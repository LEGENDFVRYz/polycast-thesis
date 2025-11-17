// ###########################################
// RECEIVER (ESP32) - REVISED FOR SENSOR DATA
// ###########################################

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel

// DEFINE THE WIFI CHANNEL (Must be 1-13)
#define WIFI_CHANNEL 1

// --------------------
// ** NEW DATA STRUCTURE **
// This MUST match the sender's structure
// --------------------
typedef struct struct_sensor_data {
  float x;
  float y;
  float filtered_x;
  float filtered_y;
} struct_sensor_data;

// Create an instance of the new data structure
struct_sensor_data myData;

// -----------------------------------------------------------------
// CALLBACK FUNCTION - Updated for new data
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

  // --------------------
  // ** UPDATED PRINT LOGIC **
  // Print the received data in a new format for the Python script
  // --------------------
  Serial.printf("From: %s | ", macStr);
  Serial.printf("X: %.3f | ", myData.x);
  Serial.printf("Y: %.3f | ", myData.y);
  Serial.printf("F_X: %.3f | ", myData.filtered_x);
  Serial.printf("F_Y: %.3f\n", myData.filtered_y);
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
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.printf("ESP-NOW receiver ready. Listening on Channel %d\n", WIFI_CHANNEL);
}

void loop() {
  // This loop can be empty.
  delay(1000); 
}