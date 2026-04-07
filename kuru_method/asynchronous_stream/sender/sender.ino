/*
 * sender.ino  —  PolyCast Asynchronous Dual-Stream Transmitter
 * =============================================================
 * Sends IMU and UWB data as separate ESP-NOW packets, each at its
 * natural rate.  This eliminates the stale-UWB problem inherent in
 * the batched approach (where 50% of packets carried duplicate UWB).
 *
 * Architecture
 * ------------
 *   Core 0 — IMU task:  polls BNO085, sends ImuPacket at ~100 Hz
 *   Core 1 — UWB task:  TDMA cycle,   sends UwbPacket at ~10 Hz
 *
 * ESP-NOW sends are serialised with a FreeRTOS mutex so the two
 * cores never call esp_now_send() simultaneously.
 *
 * Hardware (Arduino Nano ESP32)
 * ----------------------------
 *   BNO085 IMU:      I2C 0x4A, reset GPIO 4
 *   DW3000 UWB:      SPI CS=10, RST=3, IRQ=2
 *   Tactile button:  A0 (INPUT_PULLDOWN)
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "uwb_module.h"
#include "imu_module.h"

// ── UWB Packet Structure ─────────────────────────────────────────────
struct __attribute__((packed)) UwbPacket {
    uint8_t  type;               // 0x02
    uint32_t seq;
    float    d0, d1, d2, d3;    // distances in metres
    uint32_t ts;                 // micros()
};

// ── Anchor index mapping (same as tx_proto_batch) ────────────────────
static const int MAP_DIST0 = 0;
static const int MAP_DIST1 = 2;
static const int MAP_DIST2 = 4;
static const int MAP_DIST3 = 6;

// ── ESP-NOW ──────────────────────────────────────────────────────────
#define WIFI_CHANNEL 1
static uint8_t receiverMAC[] = {0x80, 0xF3, 0xDA, 0x55, 0x9F, 0x6C};
static esp_now_peer_info_t peerInfo;

// ── Cross-core send mutex ────────────────────────────────────────────
// FreeRTOS mutex (not portMUX) because esp_now_send() takes 1-3 ms
// and disabling interrupts for that long would trigger the watchdog.
static SemaphoreHandle_t espnowMutex;

// ── Per-stream sequence counters ─────────────────────────────────────
static uint32_t imuSeq = 0;
static uint32_t uwbSeq = 0;

// ── UWB live distances (written by Core 1 only — no cross-core share)
static int live_distances[MAX_ANCHOR_LIST_SIZE];


// ═════════════════════════════════════════════════════════════════════
//  ESP-NOW Setup
// ═════════════════════════════════════════════════════════════════════
static void setupESPNow() {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] Init failed — restarting.");
        ESP.restart();
    }

    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.ifidx   = WIFI_IF_STA;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    Serial.println("[ESPNOW] Initialised.");
}


// ═════════════════════════════════════════════════════════════════════
//  [CORE 0]  IMU Task — 100 Hz individual samples
// ═════════════════════════════════════════════════════════════════════
static TaskHandle_t IMUTaskHandle;

static void IMUTask(void* parameter) {
    ImuPacket pkt;

    for (;;) {
        if (processIMU(&pkt)) {
            pkt.seq = imuSeq++;

            if (xSemaphoreTake(espnowMutex, pdMS_TO_TICKS(5))) {
                esp_now_send(receiverMAC, (uint8_t*)&pkt, sizeof(ImuPacket));
                xSemaphoreGive(espnowMutex);
            }
        }

        // 1 ms yield — feeds Core 0 watchdog and gives WiFi stack room.
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


// ═════════════════════════════════════════════════════════════════════
//  Setup
// ═════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 2000)) {
        delay(10);
    }

    Serial.println("--- PolyCast Async Transmitter ---");

    initUWBConfig();
    initIMU();
    setupESPNow();

    espnowMutex = xSemaphoreCreateMutex();

    // Launch IMU task on Core 0
    xTaskCreatePinnedToCore(
        IMUTask,            // function
        "IMU_Task",         // name
        4096,               // stack (bytes)
        NULL,               // parameter
        1,                  // priority
        &IMUTaskHandle,     // handle
        0                   // Core 0
    );
}


// ═════════════════════════════════════════════════════════════════════
//  [CORE 1]  UWB Task — ~10 Hz TDMA cycles
// ═════════════════════════════════════════════════════════════════════
void loop() {
    bool is_locked = runUWBCycle(live_distances);

    if (is_locked) {
        UwbPacket pkt;
        pkt.type = 0x02;
        pkt.seq  = uwbSeq++;
        pkt.d0   = (float)live_distances[MAP_DIST0] / 100.0f;
        pkt.d1   = (float)live_distances[MAP_DIST1] / 100.0f;
        pkt.d2   = (float)live_distances[MAP_DIST2] / 100.0f;
        pkt.d3   = (float)live_distances[MAP_DIST3] / 100.0f;
        pkt.ts   = micros();

        if (xSemaphoreTake(espnowMutex, pdMS_TO_TICKS(5))) {
            esp_now_send(receiverMAC, (uint8_t*)&pkt, sizeof(UwbPacket));
            xSemaphoreGive(espnowMutex);
        }
    } else {
        Serial.println("[UWB] Network lost. Hunting...");
        delay(5);
    }
}
