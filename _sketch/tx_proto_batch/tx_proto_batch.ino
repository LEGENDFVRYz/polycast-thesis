#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "uwb_module.h"
#include "imu_module.h"

// --- BATCH CONFIGURATION ---
#define BATCH_SIZE 5

// --- PACKET STRUCTURES ---
struct __attribute__((packed)) Packet {
    uint32_t seq;
    uint32_t batch_ts;
    uint32_t uwb_ts;
    float dist0;
    float dist1;
    float dist2;
    float dist3;
    ImuPacket samples[BATCH_SIZE];
};

// --- GLOBALS ---
volatile float shared_dist0 = -1.0f;
volatile float shared_dist1 = -1.0f;
volatile float shared_dist2 = -1.0f;
volatile float shared_dist3 = -1.0f;
volatile uint32_t shared_uwb_ts = 0;
uint32_t uwbPacketCount = 0;

portMUX_TYPE distMux = portMUX_INITIALIZER_UNLOCKED;

int live_distances[MAX_ANCHOR_LIST_SIZE];

// Prototype anchors
const int MAP_DIST0 = 0; 
const int MAP_DIST1 = 2; 
const int MAP_DIST2 = 4; 
const int MAP_DIST3 = 6; 

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
    Packet tx_packet;
    tx_packet.seq = 0;
    int current_batch_idx = 0;

    // This is an infinite FreeRTOS loop strictly pinned to Core 0
    for(;;) {
        PacketIMU single_sample;

        if (processIMU(&single_sample)) {
            tx_packet.samples[current_batch_idx] = single_sample;
            current_batch_idx++;
            
            // If the batch is full, attach the UWB data and send!
            if (current_batch_idx >= BATCH_SIZE) {
                
                // Copy the latest known UWB data from the Shared Memory
                taskENTER_CRITICAL(&distMux);
                tx_packet.dist0 = shared_dist0;
                tx_packet.dist1 = shared_dist1;
                tx_packet.dist2 = shared_dist2;
                tx_packet.dist3 = shared_dist3;
                tx_packet.uwb_ts = shared_uwb_ts;
                taskEXIT_CRITICAL(&distMux);
                
                tx_packet.batch_ts = micros();
                
                esp_now_send(receiverMAC, (uint8_t *)&tx_packet, sizeof(Packet));
                
                tx_packet.seq++;
                current_batch_idx = 0; // Reset for the next batch
            }
        }
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
        taskENTER_CRITICAL(&distMux);
        shared_dist0 = (float)live_distances[MAP_DIST0] / 100.0f;
        shared_dist1 = (float)live_distances[MAP_DIST1] / 100.0f;
        shared_dist2 = (float)live_distances[MAP_DIST2] / 100.0f;
        shared_dist3 = (float)live_distances[MAP_DIST3] / 100.0f;
        shared_uwb_ts = micros();
        taskEXIT_CRITICAL(&distMux);

        Serial.println("[UWB] Sent Payload");
    } else {
        Serial.println("[UWB] Network lost. Hunting...");
    }
}