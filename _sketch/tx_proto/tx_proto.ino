#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "uwb_module.h"
#include "imu_module.h"

// --- PACKET STRUCTURES ---
// Packet 2: Low-Speed UWB Data (every 1 sample)
typedef struct __attribute__((packed)) {
    uint8_t type = 0x02;
    uint32_t packetId;
    float x;
    float y;
    float dist0;
    float dist1;
    float dist2;
    uint32_t ts;
} PacketUWB;

// --- GLOBALS ---
// Reference array to hold the distances of the UWB module
int live_distances[MAX_ANCHOR_LIST_SIZE];
uint32_t uwbPacketCount = 0;

// --- CONFIGURABLE ANCHOR MAPPING ---
// Adjust these indices when the custom PCB arrives!
// Currently mapped to physical anchors 0, 2, and 4 for jumper-wire stability.
const int MAP_DIST0 = 0; 
const int MAP_DIST1 = 2; 
const int MAP_DIST2 = 4; 
// const int MAP_DIST3 = 6; // Ready to be uncommented when receiver is updated

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
    initUWBConfig();

    // Initialize the IMU hardware
    initIMU();

    // Configuration of ESPNOW Communication
    setupESPNow();
}

void loop() {
    // =========================================================
    // [CORE 1] Pulling One UWB Cycle (Low-Speed)
    // =========================================================
    bool is_locked = runUWBCycle(live_distances);

    if (is_locked) {
        // 1. Log the raw array to the Serial Monitor
        char print_buf[128];
        snprintf(print_buf, sizeof(print_buf), "UWB ARRAY -> [A0:%d, A1:%d, A2:%d, A3:%d, A4:%d, A5:%d, A6:%d, A7:%d]", 
                 live_distances[0], live_distances[1], live_distances[2], live_distances[3],
                 live_distances[4], live_distances[5], live_distances[6], live_distances[7]);
        Serial.println(print_buf); 

        // 2. Package and Send the ESP-NOW Payload
        PacketUWB uwb_packet;
        uwb_packet.packetId = uwbPacketCount++;
        
        // Mock data for initial receiver testing
        uwb_packet.x = 1.23f; 
        uwb_packet.y = 4.56f; 
        
        // Map the physical prototype anchors to the payload
        // Casting the integer centimeters to floats
        uwb_packet.dist0 = (float)live_distances[MAP_DIST0];
        uwb_packet.dist1 = (float)live_distances[MAP_DIST1];
        uwb_packet.dist2 = (float)live_distances[MAP_DIST2];
        uwb_packet.ts = micros();

        esp_now_send(receiverMAC, (uint8_t *)&uwb_packet, sizeof(PacketUWB));
        
    } else {
        Serial.println("[MAIN CODE] Network lost. Hunting for Master Anchor...");
    }

    // =========================================================
    // [CORE 2] Pulling 3 Samples of IMU (High-Speed)
    // =========================================================
    // PacketIMU ready_packet;
    
    // if (processIMU(&ready_packet)) {
    //     // Send IMMEDIATELY over ESP-NOW
    //     esp_now_send(receiverMAC, (uint8_t *) &ready_packet, sizeof(PacketIMU));
    // }
}