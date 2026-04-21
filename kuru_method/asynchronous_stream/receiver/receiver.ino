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

// ── Control packets (Item D — DCD calibration) ───────────────────────
struct __attribute__((packed)) CtrlCmdPacket {
    uint8_t  type;               // 0x10  receiver -> sender
    uint8_t  op;                 // 1 = CAL_SAVE
};

struct __attribute__((packed)) CtrlReplyPacket {
    uint8_t  type;               // 0x11  sender -> receiver
    uint8_t  op;
    int8_t   status;             // 0 = SH2_OK
};

// Auto-learned sender MAC (set on first ImuPacket / UwbPacket).
static uint8_t senderMAC[6] = {0};
static bool    senderMACKnown = false;
static bool    senderPeerAdded = false;

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
    // Auto-learn sender MAC so we can send CAL commands back.
    if (!senderMACKnown && info != nullptr) {
        memcpy(senderMAC, info->src_addr, 6);
        senderMACKnown = true;
    }

    uint8_t next = (bufHead + 1) % BUF_SLOTS;
    if (next == bufTail) return;       // buffer full — drop packet

    memcpy((void*)pktBuf[bufHead], data, len);
    pktLen[bufHead] = (uint16_t)len;
    bufHead = next;
}


// ── Send CAL request to sender (Item D) ──────────────────────────────
static void sendCalRequest() {
    if (!senderMACKnown) {
        Serial.println("CAL:ERR no_sender_mac");
        return;
    }

    // Add sender as ESP-NOW peer the first time.
    if (!senderPeerAdded) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, senderMAC, 6);
        peer.channel = WIFI_CHANNEL;
        peer.ifidx   = WIFI_IF_STA;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) != ESP_OK) {
            Serial.println("CAL:ERR add_peer");
            return;
        }
        senderPeerAdded = true;
    }

    CtrlCmdPacket cmd;
    cmd.type = 0x10;
    cmd.op   = 1;   // CAL_SAVE
    esp_err_t r = esp_now_send(senderMAC, (uint8_t*)&cmd, sizeof(cmd));
    if (r != ESP_OK) {
        Serial.print("CAL:ERR send=");
        Serial.println((int)r);
    } else {
        Serial.println("CAL:SENT");
    }
}


// ── Setup ────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(921600);

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


// ── Read serial line, return true if a complete CAL\n was received ───
static bool serialPollCal() {
    static char    buf[16];
    static uint8_t pos = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            buf[pos] = '\0';
            bool isCal = (pos == 3 &&
                          buf[0] == 'C' && buf[1] == 'A' && buf[2] == 'L');
            pos = 0;
            if (isCal) return true;
        } else if (pos < sizeof(buf) - 1) {
            buf[pos++] = c;
        } else {
            pos = 0;   // overflow — discard
        }
    }
    return false;
}


// ── Loop — drain ring buffer and print CSV ───────────────────────────
void loop() {
    if (serialPollCal()) sendCalRequest();

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

        // ── Control reply (Item D) ───────────────────────────────────
        else if (type == 0x11 && len == sizeof(CtrlReplyPacket)) {
            CtrlReplyPacket rep;
            memcpy(&rep, (void*)pktBuf[idx], sizeof(CtrlReplyPacket));
            if (rep.status == 0) {
                Serial.println("CAL:OK");
            } else {
                Serial.print("CAL:ERR ");
                Serial.println((int)rep.status);
            }
        }

        // Advance tail
        bufTail = (bufTail + 1) % BUF_SLOTS;
    }
}
