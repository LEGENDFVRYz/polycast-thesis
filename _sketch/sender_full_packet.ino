// Sender: Arduino Nano ESP32
// Sends one ESP-NOW packet per UWB reception, containing:
//  - filtered_x, filtered_y (float)
//  - dist0,dist1,dist2 (float)
//  - 10 compressed IMU samples: qx,qy,qz,qw (int16 scaled), ax,ay,az (int16 scaled), force (int16 scaled), ts (uint32)
//
// Requires Adafruit_BNO08x library and ESP-NOW support.

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1
#define UWB_RX 9
#define UWB_TX 8

// Compression scales
const float Q_SCALE = 32767.0f; // quaternion -1..1 -> int16
const float A_SCALE = 1000.0f;   // accel m/s^2 * 1000 -> int16
const float F_SCALE = 100.0f;    // Force: Multiplies float by 100 to keep 2 decimals
                                 // e.g., 30.55 becomes 3055. Fits in int16.

// number of IMU samples per packet
const uint8_t IMU_SAMPLES_PER_PACKET = 10;

// Data buffer sizes
// IMU Sample size: 
// 4x int16 (quat) = 8 bytes
// 3x int16 (accel) = 6 bytes
// 1x int16 (force) = 2 bytes  (Required for 0-31 range with decimals, as 31.00*100 = 3100 > 255)
// 1x uint32 (ts)   = 4 bytes
// Total = 20 bytes per sample
const size_t IMU_SAMPLE_SIZE = 20; 
const size_t MAX_PACKET_SIZE = 250; 

// Anchor config
struct Position { float x, y; };
Position* base0 = new Position{1.24, 0};
Position* base1 = new Position{1.24, 1.23};
Position* base2 = new Position{0, 1.23};
Position* base_stations[8] = {base0, base1, base2, NULL, NULL, NULL, NULL, NULL};
float distance_offsets[8] = {-0.15, -0.1, -0.1, 0,0,0,0,0};

// IMU
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
bool imuFound = false;

// circular buffer for last N IMU samples (uncompressed floats)
struct ImuRaw {
  float qx,qy,qz,qw;
  float ax,ay,az;
  float force; // Holds decimal value (0.00 to 31.00)
  uint32_t ts;
};
ImuRaw imuRing[IMU_SAMPLES_PER_PACKET];
int imuRingHead = 0; // next write index
int imuRingCount = 0;

// UWB distances
float distances[8];

// filtered positions (EMA) kept local
float uwbX = 0, uwbY = 0;
float uwbXf = 0, uwbYf = 0;

// ------------- ESP-NOW peer -------------
uint8_t receiverMAC[] = {0x80,0xF3,0xDA,0x55,0x9F,0x6C}; // REPLACE with your receiver MAC
esp_now_peer_info_t peerInfo;

// ---------------- helper serialization ----------------
void writeUint8(uint8_t *buf, size_t &idx, uint8_t v){ buf[idx++] = v; }
void writeUint16(uint8_t *buf, size_t &idx, uint16_t v){
  buf[idx++] = v & 0xFF; buf[idx++] = (v>>8) & 0xFF;
}
void writeInt16(uint8_t *buf, size_t &idx, int16_t v){ writeUint16(buf, idx, (uint16_t)v); }
void writeUint32(uint8_t *buf, size_t &idx, uint32_t v){
  buf[idx++] = v & 0xFF; buf[idx++] = (v>>8) & 0xFF; buf[idx++] = (v>>16) & 0xFF; buf[idx++] = (v>>24) & 0xFF;
}
void writeFloatLE(uint8_t *buf, size_t &idx, float f){
  union { float f; uint8_t b[4]; } u; u.f = f;
  buf[idx++] = u.b[0]; buf[idx++] = u.b[1]; buf[idx++] = u.b[2]; buf[idx++] = u.b[3];
}

// ---------------- IMU buffering ----------------
void pushImuSample(float qx,float qy,float qz,float qw,float ax,float ay,float az, float force, uint32_t ts){
  imuRing[imuRingHead].qx = qx;
  imuRing[imuRingHead].qy = qy;
  imuRing[imuRingHead].qz = qz;
  imuRing[imuRingHead].qw = qw;
  imuRing[imuRingHead].ax = ax;
  imuRing[imuRingHead].ay = ay;
  imuRing[imuRingHead].az = az;
  imuRing[imuRingHead].force = force;
  imuRing[imuRingHead].ts = ts;
  imuRingHead = (imuRingHead + 1) % IMU_SAMPLES_PER_PACKET;
  if (imuRingCount < IMU_SAMPLES_PER_PACKET) imuRingCount++;
}

// get last N samples in chronological order
void getLastImuSamples(ImuRaw *dest){
  int count = imuRingCount;
  int start = imuRingHead - count;
  if (start < 0) start += IMU_SAMPLES_PER_PACKET;
  
  for (int i=0;i<count;i++){
    int idx = (start + i) % IMU_SAMPLES_PER_PACKET;
    dest[i] = imuRing[idx];
  }
  
  if (count < IMU_SAMPLES_PER_PACKET){
    ImuRaw pad = {0,0,0,1.0, 0,0,0, 0.0f, micros()};
    if (count>0) pad = imuRing[(imuRingHead - 1 + IMU_SAMPLES_PER_PACKET) % IMU_SAMPLES_PER_PACKET];
    for (int i=count;i<IMU_SAMPLES_PER_PACKET;i++){
      dest[i] = pad;
    }
  }
}

// ---------------- UWB decode ----------------
bool decodeUwbDistances(uint8_t* data, int dataLen, float* distances) {
    for (int i = 0; i < 8; i++) distances[i] = -1;
    if (dataLen < 35) return false;
    if (data[0] != 0xAA || data[1] != 0x25 || data[2] != 0x01) return false;
    for (int i = 0; i < 8; i++) {
        int offset = 3 + (i * 4);
        uint32_t raw =
            ((uint32_t)data[offset]) |
            ((uint32_t)data[offset+1] << 8) |
            ((uint32_t)data[offset+2] << 16) |
            ((uint32_t)data[offset+3] << 24);
        if (raw > 0) distances[i] = (raw / 1000.0f) + distance_offsets[i];
    }
    return true;
}

// ---------------- Trilateration ----------------
bool trilaterate2d(float* distances, float* x, float* y, float* out_d0, float* out_d1, float* out_d2) {
    *out_d0 = *out_d1 = *out_d2 = 0.0f;
    struct ValidData { float x,y,dist; };
    ValidData valid_data[8]; int valid_count=0;
    for (int i=0;i<8;i++){
      if (base_stations[i] != NULL && distances[i] > 0){
        valid_data[valid_count].x = base_stations[i]->x;
        valid_data[valid_count].y = base_stations[i]->y;
        valid_data[valid_count].dist = distances[i];
        valid_count++;
      }
    }
    if (valid_count < 3) return false;
    *out_d0 = valid_data[0].dist;
    *out_d1 = valid_data[1].dist;
    *out_d2 = valid_data[2].dist;
    float x1 = valid_data[0].x, y1 = valid_data[0].y, r1 = valid_data[0].dist;
    float A00, A01, A10, A11, b0, b1;
    A00 = 2*(valid_data[1].x - x1);
    A01 = 2*(valid_data[1].y - y1);
    b0 = valid_data[1].dist*valid_data[1].dist - r1*r1 - valid_data[1].x*valid_data[1].x + x1*x1 - valid_data[1].y*valid_data[1].y + y1*y1;
    A10 = 2*(valid_data[2].x - x1);
    A11 = 2*(valid_data[2].y - y1);
    b1 = valid_data[2].dist*valid_data[2].dist - r1*r1 - valid_data[2].x*valid_data[2].x + x1*x1 - valid_data[2].y*valid_data[2].y + y1*y1;
    float det = A00*A11 - A01*A10;
    if (fabs(det) < 1e-6) return false;
    *x = -(b0 * A11 - b1 * A01) / det;
    *y = -(A00 * b1 - A10 * b0) / det;
    return true;
}

// ---------------- IMU setup ----------------
void setupIMU() {
  long start = millis();
  bool found = false;
  while (millis() - start < 3000) {
    if (bno08x.begin_I2C(0x4A, &Wire)) { found = true; break; }
    delay(50);
  }
  if (!found) { imuFound = false; return; }
  imuFound = true;
  bno08x.enableReport(SH2_ROTATION_VECTOR, 5000); 
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000); 
}

// ---------------- ESP-NOW helper ----------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // debug
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial1.begin(115200, SERIAL_8N1, UWB_RX, UWB_TX);

  pinMode(A0, INPUT); 

  Wire.begin();
  setupIMU();

  imuRingHead = 0; imuRingCount = 0;

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    while(1) delay(1000);
  }
  esp_now_register_send_cb(OnDataSent);
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  uwbXf = 0; uwbYf = 0;
  Serial.println("[SENDER] Ready");
}

// ---------------- update IMU ----------------
void updateIMU() {
  if (!imuFound) return;
  if (!bno08x.getSensorEvent(&sensorValue)) return;
  if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    static float last_qx=0,last_qy=0,last_qz=0,last_qw=1.0;
    last_qx = sensorValue.un.rotationVector.i;
    last_qy = sensorValue.un.rotationVector.j;
    last_qz = sensorValue.un.rotationVector.k;
    last_qw = sensorValue.un.rotationVector.real;
    
    imuRing[ (imuRingHead==0?IMU_SAMPLES_PER_PACKET-1:imuRingHead-1) ].qx = last_qx;
    imuRing[ (imuRingHead==0?IMU_SAMPLES_PER_PACKET-1:imuRingHead-1) ].qy = last_qy;
    imuRing[ (imuRingHead==0?IMU_SAMPLES_PER_PACKET-1:imuRingHead-1) ].qz = last_qz;
    imuRing[ (imuRingHead==0?IMU_SAMPLES_PER_PACKET-1:imuRingHead-1) ].qw = last_qw;

  } else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
    float ax = sensorValue.un.linearAcceleration.x;
    float ay = sensorValue.un.linearAcceleration.y;
    float az = sensorValue.un.linearAcceleration.z;
    
    // --- FORCE SENSOR MAPPING (0-31 range with decimals) ---
    // Read A0 (0-4095)
    int rawForce = analogRead(A0);
    
    // Map 0-4095 to 0.00-31.00
    // Example: raw 2048 -> ~15.50
    float forceVal = (rawForce / 4095.0f) * 31.0f;
    // ---------------------------------------

    uint32_t ts = micros();
    float qx=0,qy=0,qz=0,qw=1;
    int lastIdx = (imuRingHead - 1 + IMU_SAMPLES_PER_PACKET) % IMU_SAMPLES_PER_PACKET;
    qx = imuRing[lastIdx].qx;
    qy = imuRing[lastIdx].qy;
    qz = imuRing[lastIdx].qz;
    qw = imuRing[lastIdx].qw;

    pushImuSample(qx,qy,qz,qw, ax, ay, az, forceVal, ts);
  }
}

// ---------------- main loop ----------------
void loop() {
  updateIMU();

  static uint8_t buffer[256];
  static int idx=0;
  static bool started=false;
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (!started) {
      if (b == 0xAA) { started=true; idx=0; buffer[idx++]=b; }
    } else {
      buffer[idx++] = b;
      if (idx >= 35) {
        bool ok = decodeUwbDistances(buffer, idx, distances);
        if (ok) {
          float d0,d1,d2;
          bool success = trilaterate2d(distances, &uwbX, &uwbY, &d0, &d1, &d2);
          if (success) {
            uwbXf = 0.8f * uwbXf + 0.2f * uwbX;
            uwbYf = 0.8f * uwbYf + 0.2f * uwbY;
          }
          
          uint8_t pkt[MAX_PACKET_SIZE];
          size_t pidx = 0;
          
          // Header
          writeUint8(pkt, pidx, 0xAA);
          writeUint8(pkt, pidx, 0x55);
          writeUint8(pkt, pidx, 0x01); 
          writeUint8(pkt, pidx, IMU_SAMPLES_PER_PACKET);
          writeUint32(pkt, pidx, micros());
          writeFloatLE(pkt, pidx, uwbX);
          writeFloatLE(pkt, pidx, uwbY);
          
          float od0 = (success? d0 : -1.0f);
          float od1 = (success? d1 : -1.0f);
          float od2 = (success? d2 : -1.0f);
          writeFloatLE(pkt, pidx, od0);
          writeFloatLE(pkt, pidx, od1);
          writeFloatLE(pkt, pidx, od2);
          
          // IMU Samples
          ImuRaw tmp[IMU_SAMPLES_PER_PACKET];
          getLastImuSamples(tmp);
          
          for (int s=0;s<IMU_SAMPLES_PER_PACKET;s++){
            int16_t qxs = (int16_t)constrain(round(tmp[s].qx * Q_SCALE), -32767, 32767);
            int16_t qys = (int16_t)constrain(round(tmp[s].qy * Q_SCALE), -32767, 32767);
            int16_t qzs = (int16_t)constrain(round(tmp[s].qz * Q_SCALE), -32767, 32767);
            int16_t qws = (int16_t)constrain(round(tmp[s].qw * Q_SCALE), -32767, 32767);
            
            int16_t axs = (int16_t)constrain(round(tmp[s].ax * A_SCALE), -32767, 32767);
            int16_t ays = (int16_t)constrain(round(tmp[s].ay * A_SCALE), -32767, 32767);
            int16_t azs = (int16_t)constrain(round(tmp[s].az * A_SCALE), -32767, 32767);
            
            // --- FORCE COMPRESSION (0-31 range, 2 decimals) ---
            // Max value is 31.00
            // Multiply by 100 -> 3100
            // This fits comfortably in int16 (max 32767)
            int16_t forceInt = (int16_t)(tmp[s].force * F_SCALE); 
            
            writeInt16(pkt, pidx, qxs);
            writeInt16(pkt, pidx, qys);
            writeInt16(pkt, pidx, qzs);
            writeInt16(pkt, pidx, qws);
            writeInt16(pkt, pidx, axs);
            writeInt16(pkt, pidx, ays);
            writeInt16(pkt, pidx, azs);
            
            // Write 2 bytes for force
            writeInt16(pkt, pidx, forceInt); 
            writeUint32(pkt, pidx, tmp[s].ts);
          }
          
          esp_err_t res = esp_now_send(receiverMAC, pkt, pidx);
          if (res != ESP_OK) {
            Serial.printf("[ESP-NOW] send fail: %d\n", (int)res);
          } else {
            Serial.printf("[ESP-NOW] Sent packet %u bytes\n", (unsigned)pidx);
          }
        }
        started = false; idx = 0;
      }
      if (idx >= 256) { started=false; idx=0; }
    }
  } 
}