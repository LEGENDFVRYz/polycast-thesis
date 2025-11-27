// Receiver: ESP32 WROOM
// Receives compressed ESP-NOW packets described above and prints a Python-friendly CSV line.

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1

// scales must match sender
const float Q_SCALE = 32767.0f;
const float A_SCALE = 100.0f;
const uint8_t EXPECTED_VERSION = 0x01;
const uint8_t IMU_SAMPLES_PER_PACKET = 10;

void printHexMac(const uint8_t *mac) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.print(buf);
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len < 4 + 4 + 8 + 12 + (size_t)IMU_SAMPLES_PER_PACKET * 18) {
    Serial.println("Received packet too small or corrupt.");
    return;
  }
  const uint8_t *p = incomingData;
  int idx = 0;
  // header magic
  uint8_t m0 = p[idx++], m1 = p[idx++];
  if (m0 != 0xAA || m1 != 0x55) {
    Serial.println("Bad magic");
    return;
  }
  uint8_t ver = p[idx++]; uint8_t sample_count = p[idx++];
  if (ver != EXPECTED_VERSION) {
    Serial.println("Version mismatch");
    return;
  }
  if (sample_count != IMU_SAMPLES_PER_PACKET) {
    Serial.printf("Unexpected sample_count %d (expect %d)\n", sample_count, IMU_SAMPLES_PER_PACKET);
    // continue but be cautious
  }
  // read packet timestamp (uint32)
  uint32_t packet_ts = (uint32_t)p[idx] | ((uint32_t)p[idx+1]<<8) | ((uint32_t)p[idx+2]<<16) | ((uint32_t)p[idx+3]<<24);
  idx += 4;
  // read filtered_x, filtered_y floats
  auto readFloat = [&](float &out){
    union { uint8_t b[4]; float f; } u;
    u.b[0]=p[idx++]; u.b[1]=p[idx++]; u.b[2]=p[idx++]; u.b[3]=p[idx++]; out = u.f;
  };
  float filt_x, filt_y;
  readFloat(filt_x); readFloat(filt_y);
  // read dist0..2
  float dist0, dist1, dist2;
  readFloat(dist0); readFloat(dist1); readFloat(dist2);

  // Now read IMU samples
  struct ImuS {
    int16_t qx,qy,qz,qw;
    int16_t ax,ay,az;
    uint32_t ts;
  } imu[IMU_SAMPLES_PER_PACKET];

  for (int s=0;s<IMU_SAMPLES_PER_PACKET;s++){
    // qx
    imu[s].qx = (int16_t)((uint16_t)p[idx] | ((uint16_t)p[idx+1]<<8)); idx+=2;
    imu[s].qy = (int16_t)((uint16_t)p[idx] | ((uint16_t)p[idx+1]<<8)); idx+=2;
    imu[s].qz = (int16_t)((uint16_t)p[idx] | ((uint16_t)p[idx+1]<<8)); idx+=2;
    imu[s].qw = (int16_t)((uint16_t)p[idx] | ((uint16_t)p[idx+1]<<8)); idx+=2;
    imu[s].ax = (int16_t)((uint16_t)p[idx] | ((uint16_t)p[idx+1]<<8)); idx+=2;
    imu[s].ay = (int16_t)((uint16_t)p[idx] | ((uint16_t)p[idx+1]<<8)); idx+=2;
    imu[s].az = (int16_t)((uint16_t)p[idx] | ((uint16_t)p[idx+1]<<8)); idx+=2;
    imu[s].ts = (uint32_t)p[idx] | ((uint32_t)p[idx+1]<<8) | ((uint32_t)p[idx+2]<<16) | ((uint32_t)p[idx+3]<<24);
    idx += 4;
  }

  // Print one CSV row: filtered_x, filtered_y, dist0, dist1, dist2, packet_ts, then for each sample: qx,qy,qz,qw, ax,ay,az, ts
  // Decompress IMU values to floats for readability in Python
  Serial.print(filt_x); Serial.print(",");
  Serial.print(filt_y); Serial.print(",");
  Serial.print(dist0); Serial.print(",");
  Serial.print(dist1); Serial.print(",");
  Serial.print(dist2); Serial.print(",");
  Serial.print(packet_ts);

  for (int s=0;s<IMU_SAMPLES_PER_PACKET;s++){
    float qx = (float)imu[s].qx / Q_SCALE;
    float qy = (float)imu[s].qy / Q_SCALE;
    float qz = (float)imu[s].qz / Q_SCALE;
    float qw = (float)imu[s].qw / Q_SCALE;
    float ax = (float)imu[s].ax / A_SCALE;
    float ay = (float)imu[s].ay / A_SCALE;
    float az = (float)imu[s].az / A_SCALE;
    Serial.print(",");
    // order: qx,qy,qz,qw,ax,ay,az,ts
    Serial.print(qx,6); Serial.print(","); Serial.print(qy,6); Serial.print(","); Serial.print(qz,6); Serial.print(","); Serial.print(qw,6);
    Serial.print(","); Serial.print(ax,4); Serial.print(","); Serial.print(ay,4); Serial.print(","); Serial.print(az,4);
    Serial.print(","); Serial.print((unsigned long)imu[s].ts);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while(1) delay(1000);
  }
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Receiver ready. Waiting for ESP-NOW packets...");
}

void loop() {
  delay(100);
}
