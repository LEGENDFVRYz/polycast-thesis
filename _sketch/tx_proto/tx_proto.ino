#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "uwb_module.h"
#include "imu_module.h"


// --- GLOBALS ---
// Reference array to hold the distances of the UWB module
int live_distances[MAX_ANCHOR_LIST_SIZE];


// --- ESPNOW DEFINES ---
#define WIFI_CHANNEL 1

uint8_t receiverMAC[] = {0x80,0xF3,0xDA,0x55,0x9F,0x6C};  // Receiver MAC Address
esp_now_peer_info_t peerInfo;


void setupESPNow() {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        ESP.restart();
    }
    
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel =  WIFI_CHANNEL;
    peerInfo.ifidx =    WIFI_IF_STA;
    peerInfo.encrypt =  false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
    }
}


// ---- MAIN LOGIC ----
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("--- Booting Prototype Transmitter ---");
    
    // Initialize the UWB hardware
    // initUWBConfig();

    // Initialize the IMU hardware
    initIMU();

    // Configuration of ESPNOW Communication
    setupESPNow();
}

void loop() {
    // [CORE 1] Pulling One UWB Cycle
    // bool is_locked = runUWBCycle(live_distances);

    // if (is_locked) {
    //     char print_buf[128];
    //     snprintf(print_buf, sizeof(print_buf), "MAIN CODE DATA -> [A0:%d, A1:%d, A2:%d, A3:%d, A4:%d, A5:%d, A6:%d, A7:%d]", 
    //              live_distances[0], live_distances[1], live_distances[2], live_distances[3],
    //              live_distances[4], live_distances[5], live_distances[6], live_distances[7]);
    //     Serial.println(print_buf); 
    // } else {
    //     Serial.println("[MAIN CODE] Network lost. Hunting for Master Anchor...");
    // }

    
    // [CORE 2] Pulling 3 Samples of IMU
    PacketIMU ready_packet;
    
    if (processIMU(&ready_packet)) {
        // Send IMMEDIATELY over ESP-NOW
        esp_now_send(receiverMAC, (uint8_t *) &ready_packet, sizeof(PacketIMU));
        Serial.println("[IMU] Sent batch of 3 samples!"); // Optional: just so you know it fired
    }
    
}