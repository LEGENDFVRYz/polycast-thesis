#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "uwb_module.h"
#include "imu_module.h"


// --- GLOBALS ---
volatile int live_distances[MAX_ANCHOR_LIST_SIZE];
uint32_t uwbPacketCount = 0;


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
// CORE 0: IMU & ESP-NOW TASK (High-Speed Polling)
// =========================================================
TaskHandle_t IMUTaskHandle;

void IMUTask(void *parameter) {
    PacketIMU ready_packet;
    
    // FreeRTOS loop strictly pinned to IMU process
    for(;;) {
        if (processIMU(&ready_packet)) {
            esp_now_send(receiverMAC, (uint8_t *) &ready_packet, sizeof(PacketIMU));
            // Serial.println("[IMU] Sent Payload");
        }
        
        // Delay: 1-millisecond yield to optimize the WiFi
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
    
    Serial.println("--- Marker Dual-Core Transmitter ---");
    
    initUWBConfig();
    initIMU();
    setupESPNow();

    // Launch the IMU Task
    xTaskCreatePinnedToCore(
        IMUTask,        
        "IMU_Task",     
        4096,           // Stack size (bytes)
        NULL,           // Parameter passed
        1,              // Task priority
        &IMUTaskHandle, // Task handle
        0               // Pin to Core 0
    );
}


// =========================================================
// CORE 1: UWB TASK (Strict TDMA Timing)
// =========================================================
void loop() {

    bool is_locked = runUWBCycle((int*)live_distances);

    if (is_locked) {
        PacketUWB uwb_packet;
        uwb_packet.packetId = uwbPacketCount++;
        uwb_packet.dist0 = (float)live_distances[MAP_DIST0] / 100.0f;
        uwb_packet.dist1 = (float)live_distances[MAP_DIST1] / 100.0f;
        uwb_packet.dist2 = (float)live_distances[MAP_DIST2] / 100.0f;
        uwb_packet.dist2 = (float)live_distances[MAP_DIST3] / 100.0f;
        uwb_packet.ts = micros();

        esp_now_send(receiverMAC, (uint8_t *)&uwb_packet, sizeof(PacketUWB));
        Serial.println("[UWB] Sent Payload");
    } else {
        Serial.println("[UWB] Network lost. Hunting...");
    }
}
