#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "uwb_module.h"
#include "imu_module.h"

// --- PACKET STRUCTURES ---
typedef struct __attribute__((packed)) {
    uint8_t type = 0x02;
    uint32_t packetId;
    float dist0;
    float dist1;
    float dist2;
    uint32_t ts;
} PacketUWB;

// --- GLOBALS ---
// 'volatile' prevents the compiler from caching the array since two CPU cores access it simultaneously
volatile int live_distances[MAX_ANCHOR_LIST_SIZE];
uint32_t uwbPacketCount = 0;

const int MAP_DIST0 = 0; 
const int MAP_DIST1 = 2; 
const int MAP_DIST2 = 4; 

// --- ESPNOW DEFINES ---
#define WIFI_CHANNEL 1
uint8_t receiverMAC[] = {0x80,0xF3,0xDA,0x55,0x9F,0x6C};  
esp_now_peer_info_t peerInfo;

void setupESPNow() {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) { ESP.restart(); }
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

// =========================================================
// [CORE 0] IMU & ESP-NOW TASK (High-Speed Polling)
// =========================================================
TaskHandle_t IMUTaskHandle;

void IMUTask(void *parameter) {
    PacketIMU ready_packet;
    
    // This is an infinite FreeRTOS loop strictly pinned to Core 0
    for(;;) {
        if (processIMU(&ready_packet)) {
            esp_now_send(receiverMAC, (uint8_t *) &ready_packet, sizeof(PacketIMU));
            // Serial.println("[IMU] Sent Payload");
        }
        
        // CRITICAL: A 1-millisecond yield. 
        // This feeds the Core 0 Watchdog Timer and gives the WiFi stack room to breathe!
        vTaskDelay(1 / portTICK_PERIOD_MS); 
    }
}

// ---- MAIN SETUP ----
void setup() {
    Serial.begin(115200);
    
    uint32_t serial_timeout = millis();
    while (!Serial && (millis() - serial_timeout < 2000)) { 
        delay(10); 
    }
    
    Serial.println("--- Booting Dual-Core Transmitter ---");
    
    initUWBConfig();
    initIMU();
    setupESPNow();

    // Launch the IMU Task onto Core 0 (PRO_CPU)
    xTaskCreatePinnedToCore(
        IMUTask,        // Function to run
        "IMU_Task",     // Name of the task
        4096,           // Stack size (bytes)
        NULL,           // Parameter passed
        1,              // Task priority (1 is safe)
        &IMUTaskHandle, // Task handle
        0               // Pin to Core 0!
    );
}

// =========================================================
// [CORE 1] UWB TASK (Strict TDMA Timing)
// =========================================================
void loop() {

    bool is_locked = runUWBCycle((int*)live_distances);

    if (is_locked) {
        PacketUWB uwb_packet;
        uwb_packet.packetId = uwbPacketCount++;
        uwb_packet.dist0 = (float)live_distances[MAP_DIST0];
        uwb_packet.dist1 = (float)live_distances[MAP_DIST1];
        uwb_packet.dist2 = (float)live_distances[MAP_DIST2];
        uwb_packet.ts = micros();

        esp_now_send(receiverMAC, (uint8_t *)&uwb_packet, sizeof(PacketUWB));
        Serial.println("[UWB] Sent Payload");
    } else {
        Serial.println("[UWB] Network lost. Hunting...");
    }
}
