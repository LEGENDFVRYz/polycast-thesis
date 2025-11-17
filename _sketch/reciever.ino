// ###########################################
// RECEIVER (ESP32) - MATCHING NEW FULL STRUCT
// ###########################################

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1

typedef struct struct_combine_sensor_data {
    float x;
    float y;
    float filtered_x;
    float filtered_y;

    float qx;
    float qy;
    float qz;
    float qw;

    float ax;
    float ay;
    float az;

    uint32_t timestamp;
} struct_combine_sensor_data;

struct_combine_sensor_data myData;

// -----------------------------------------------------------------
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (len < sizeof(myData)) {
        Serial.println("Packet too small!");
        return;
    }

    memcpy(&myData, incomingData, sizeof(myData));

    const uint8_t *mac = info->src_addr;
    char macStr[18];
    snprintf(macStr, sizeof(macStr),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // ------------------------------
    // PRINT IN PYTHON-FRIENDLY FORMAT
    // ------------------------------
    Serial.printf("From: %s | X: %.3f | Y: %.3f | FX: %.3f | FY: %.3f | "
                  "QX: %.3f | QY: %.3f | QZ: %.3f | QW: %.3f | "
                  "AX: %.3f | AY: %.3f | AZ: %.3f | TS: %u\n",
                  macStr,
                  myData.x, myData.y,
                  myData.filtered_x, myData.filtered_y,
                  myData.qx, myData.qy, myData.qz, myData.qw,
                  myData.ax, myData.ay, myData.az,
                  myData.timestamp);
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW error");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("Receiver ready.");
}

void loop() {
    delay(100);
}
