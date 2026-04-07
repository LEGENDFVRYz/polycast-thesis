/*
 * receiver.ino  —  PolyCast Async Stream Receiver (ESP32 WROOM)
 * ==============================================================
 * Receives interleaved IMU (0x01) and UWB (0x02) ESP-NOW packets
 * and outputs them as CSV lines over serial at 115200 baud.
 *
 * CSV format
 * ----------
 *   IMU:  I,<seq>,<qx>,<qy>,<qz>,<qw>,<ax>,<ay>,<az>,<force>,<ts>
 *   UWB:  U,<seq>,<d0>,<d1>,<d2>,<d3>,<ts>
 *
 * Architecture
 * ------------
 *   The ESP-NOW receive callback copies packets into a 16-slot ring
 *   buffer.  loop() drains the buffer and prints CSV.  This avoids
 *   calling Serial.print() inside the callback (which runs in the
 *   WiFi task context and can block on UART TX full at 100 Hz).
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1

// ── Packet structures (must match sender) ────────────────────────────

struct __attribute__((packed)) ImuPacket {
    uint8_t  type;               // 0x01
    uint32_t seq;
    float    qx, qy, qz, qw;
    float    ax, ay, az;
    float    force;
    uint32_t ts;
};

struct __attribute__((packed)) UwbPacket {
    uint8_t  type;               // 0x02
    uint32_t seq;
    float    d0, d1, d2, d3;
    uint32_t ts;
};

// ── Ring buffer ──────────────────────────────────────────────────────
// 16 slots ≈ 145 ms of buffering at 110 packets/sec.
#define BUF_SLOTS 16
#define MAX_PKT   250

static volatile uint8_t  pktBuf[BUF_SLOTS][MAX_PKT];
static volatile uint16_t pktLen[BUF_SLOTS];
static volatile uint8_t  bufHead = 0;
static volatile uint8_t  bufTail = 0;


// ── ESP-NOW receive callback ─────────────────────────────────────────
void OnDataRecv(const esp_now_recv_info_t* info,
                const uint8_t* data, int len) {
    uint8_t next = (bufHead + 1) % BUF_SLOTS;
    if (next == bufTail) return;       // buffer full — drop packet

    memcpy((void*)pktBuf[bufHead], data, len);
    pktLen[bufHead] = (uint16_t)len;
    bufHead = next;
}


// ── Setup ────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[RX] ESP-NOW init failed.");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("[RX] PolyCast Async Receiver ready.");
}


// ── Loop — drain ring buffer and print CSV ───────────────────────────
void loop() {
    while (bufTail != bufHead) {
        uint8_t  idx  = bufTail;
        uint8_t  type = pktBuf[idx][0];
        uint16_t len  = pktLen[idx];

        // ── IMU packet ───────────────────────────────────────────────
        if (type == 0x01 && len == sizeof(ImuPacket)) {
            ImuPacket pkt;
            memcpy(&pkt, (void*)pktBuf[idx], sizeof(ImuPacket));

            Serial.print("I,");
            Serial.print(pkt.seq);       Serial.print(',');
            Serial.print(pkt.qx, 4);     Serial.print(',');
            Serial.print(pkt.qy, 4);     Serial.print(',');
            Serial.print(pkt.qz, 4);     Serial.print(',');
            Serial.print(pkt.qw, 4);     Serial.print(',');
            Serial.print(pkt.ax, 4);     Serial.print(',');
            Serial.print(pkt.ay, 4);     Serial.print(',');
            Serial.print(pkt.az, 4);     Serial.print(',');
            Serial.print(pkt.force, 2);  Serial.print(',');
            Serial.println(pkt.ts);
        }

        // ── UWB packet ───────────────────────────────────────────────
        else if (type == 0x02 && len == sizeof(UwbPacket)) {
            UwbPacket pkt;
            memcpy(&pkt, (void*)pktBuf[idx], sizeof(UwbPacket));

            Serial.print("U,");
            Serial.print(pkt.seq);       Serial.print(',');
            Serial.print(pkt.d0, 4);     Serial.print(',');
            Serial.print(pkt.d1, 4);     Serial.print(',');
            Serial.print(pkt.d2, 4);     Serial.print(',');
            Serial.print(pkt.d3, 4);     Serial.print(',');
            Serial.println(pkt.ts);
        }

        // Advance tail
        bufTail = (bufTail + 1) % BUF_SLOTS;
    }
}
