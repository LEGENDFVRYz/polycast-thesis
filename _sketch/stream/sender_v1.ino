// Revised Pipeline Architecture
//    SENDER: Arduino Nano ESP32
//    High-Speed IMU Packet + Low-Speed UWB Packet (Parallel Streaming)
// -------------------------------------------------------------------
// 
//    Treat the IMU and UWB as two independent streams of data that merge at the Receiver/PC.
//

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// CONFIGURATION 
#define WIFI_CHANNEL 1
#define UWB_RX 9
#define UWB_TX 8

uint8_t receiverMAC[] = {0x80,0xF3,0xDA,0x55,0x9F,0x6C};  // Receiver MAC Address
esp_now_peer_info_t peerInfo;


// SCALING FACTORS
const float Q_SCALE = 32767.0f;   // quaternion -1..1 -> int16
const float A_SCALE = 1000.0f;    // accel m/s^2 * 1000 -> int16
const float F_SCALE = 100.0f;     // Force: Multiplies float by 100 to keep 2 decimals
                                  // e.g., 30.55 becomes 3055. Fits in int16.


// --- PACKET STRUCTURES (BINARY STREAMING) ---
// Packet 1: High-Speed IMU Data (every ~3 samples)
typedef struct __attribute__((packed)) {
  uint8_t type = 0x01;
  uint32_t packetId; // Counter to detect dropped packets
  struct {
    int16_t qx, qy, qz, qw;
    int16_t ax, ay, az;
    int16_t force;
    uint32_t ts;
  } samples[3];
} PacketIMU;

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
Adafruit_BNO08x bno08x(-1);       // IMU Harcware Config
sh2_SensorValue_t sensorValue;
bool imuFound = false;

uint32_t imuPacketCount = 0;      // IMU Packet Counters

PacketIMU currentImuPacket;       // IMU Batching Buffer
uint8_t imuSampleIndex = 0;

uint32_t uwbPacketCount = 0;      // UWB Packet Counters

float distances[8];               // UWB Parsing
float uwbX = 0, uwbY = 0;

struct Position { float x, y; };  // UWB Anchor Config
Position* base_stations[8] = {
  new Position{0.90, 0.90}, 
  new Position{0.00, 0.90}, 
  new Position{0.00, 0.00}, 
  NULL, NULL, NULL, NULL, NULL    // --- note: anchor 3-7 are offline
};

float distance_offsets[8] = {     
  -0.15, -0.1, -0.1, 
  0, 0, 0, 0, 0                   // --- note: anchor 3-7 are offline
};


// --- HELPER: SETUP FUNCTIONS ---
void setupIMU() {
  long start = millis();
  while (millis() - start < 3000) {
    if (bno08x.begin_I2C(0x4A, &Wire)) { imuFound = true; break; }
    delay(50);
  }
  if (imuFound) {
    bno08x.enableReport(SH2_ROTATION_VECTOR, 5000); 
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000); 
  }
}

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


// --- HELPER: IMU CORE FUNCTIONS ---
void _sendIMUPacket() {
  currentImuPacket.packetId = imuPacketCount++;
  esp_now_send(receiverMAC, (uint8_t *) &currentImuPacket, sizeof(PacketIMU));
  
  imuSampleIndex = 0;   // --- note: reset index for next batch
}

void handleIMU() {
  if (!imuFound) return;
  if (!bno08x.getSensorEvent(&sensorValue)) return;

  // We need BOTH Rotation and Accel. 
  // Simple strategy: Update static vars, only "commit" sample when Accel arrives 
  // (Assuming Accel and Rot come in pairs or close enough)
  static float cache_qx=0, cache_qy=0, cache_qz=0, cache_qw=1;
  
  if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    cache_qx = sensorValue.un.rotationVector.i;
    cache_qy = sensorValue.un.rotationVector.j;
    cache_qz = sensorValue.un.rotationVector.k;
    cache_qw = sensorValue.un.rotationVector.real;
  } 
  else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
    // Get Acceleration
    float ax = sensorValue.un.linearAcceleration.x;
    float ay = sensorValue.un.linearAcceleration.y;
    float az = sensorValue.un.linearAcceleration.z;

    // Get Pressure Force (attached to this instant)
    int rawForce = analogRead(A0);
    float forceVal = (rawForce / 4095.0f) * 31.0f;  // note: 0 - 31 range

    // Populate current sample slot
    currentImuPacket.samples[imuSampleIndex].qx = (int16_t)constrain(round(cache_qx * Q_SCALE), -32767, 32767);
    currentImuPacket.samples[imuSampleIndex].qy = (int16_t)constrain(round(cache_qy * Q_SCALE), -32767, 32767);
    currentImuPacket.samples[imuSampleIndex].qz = (int16_t)constrain(round(cache_qz * Q_SCALE), -32767, 32767);
    currentImuPacket.samples[imuSampleIndex].qw = (int16_t)constrain(round(cache_qw * Q_SCALE), -32767, 32767);
    
    currentImuPacket.samples[imuSampleIndex].ax = (int16_t)constrain(round(ax * A_SCALE), -32767, 32767);
    currentImuPacket.samples[imuSampleIndex].ay = (int16_t)constrain(round(ay * A_SCALE), -32767, 32767);
    currentImuPacket.samples[imuSampleIndex].az = (int16_t)constrain(round(az * A_SCALE), -32767, 32767);
    
    currentImuPacket.samples[imuSampleIndex].force = (int16_t)(forceVal * F_SCALE);
    currentImuPacket.samples[imuSampleIndex].ts = micros();

    // Push and Check the IMU Buffering
    imuSampleIndex++;
    if (imuSampleIndex >= 3) {
      _sendIMUPacket();
    }
  }
}


// --- HELPER: IMU CORE FUNCTIONS ---
bool _decodeUwbDistances(uint8_t* data, int dataLen, float* distances) {
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

bool _trilaterate2d(float* distances, float* x, float* y, float* d0, float* d1, float* d2) {
    // Simplified logic reusing your math
    *d0 = distances[0]; *d1 = distances[1]; *d2 = distances[2];
    if (*d0 < 0 || *d1 < 0 || *d2 < 0) return false;
    
    // (Using your explicit variables for clarity and safety)
    float x1 = base_stations[0]->x, y1 = base_stations[0]->y;
    float x2 = base_stations[1]->x, y2 = base_stations[1]->y;
    float x3 = base_stations[2]->x, y3 = base_stations[2]->y;
    float r1 = *d0, r2 = *d1, r3 = *d2;

    float A = 2 * (x2 - x1);
    float B = 2 * (y2 - y1);
    float C = r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2;
    float D = 2 * (x3 - x2);
    float E = 2 * (y3 - y2);
    float F = r2*r2 - r3*r3 - x2*x2 + x3*x3 - y2*y2 + y3*y3;

    float det = A*E - B*D;
    if (fabs(det) < 1e-6) return false;

    *x = (C*E - F*B) / det;
    *y = (A*F - C*D) / det;
    return true;
}

void handleUWB() {
  static uint8_t buffer[256];
  static int idx = 0;
  static bool started = false;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    
    if (!started) {
      if (b == 0xAA) { started = true; idx = 0; buffer[idx++] = b; }
    } else {
      buffer[idx++] = b;
      if (idx >= 35) { // Packet Complete
        bool ok = _decodeUwbDistances(buffer, idx, distances);
        if (ok) {
           float d0, d1, d2;
           if (_trilaterate2d(distances, &uwbX, &uwbY, &d0, &d1, &d2)) {
             // Create Packet
             PacketUWB pkt;
             pkt.packetId = uwbPacketCount++;
             pkt.x = uwbX;
             pkt.y = uwbY;
             pkt.dist0 = d0;
             pkt.dist1 = d1;
             pkt.dist2 = d2;
             pkt.ts = micros();
             
             // Send IMMEDIATELY
             esp_now_send(receiverMAC, (uint8_t *)&pkt, sizeof(PacketUWB));
             Serial.println("Sent UWB");
           }
        }
        started = false; 
        idx = 0;
      }
      if (idx >= 256) { started = false; idx = 0; } // Safety reset
    }
  }
}


// const uint8_t IMU_SAMPLES_PER_PACKET = 10;

// // Data buffer sizes
// // IMU Sample size: 
// // 4x int16 (quat) = 8 bytes
// // 3x int16 (accel) = 6 bytes
// // 1x int16 (force) = 2 bytes  (Required for 0-31 range with decimals, as 31.00*100 = 3100 > 255)
// // 1x uint32 (ts)   = 4 bytes
// // Total = 20 bytes per sample
// const size_t IMU_SAMPLE_SIZE = 20; 
// const size_t MAX_PACKET_SIZE = 250; 

// // Anchor config
// struct Position { float x, y; };
// Position* base0 = new Position{1.24, 0};
// Position* base1 = new Position{1.24, 1.23};
// Position* base2 = new Position{0, 1.23};
// Position* base_stations[8] = {base0, base1, base2, NULL, NULL, NULL, NULL, NULL};
// float distance_offsets[8] = {-0.15, -0.1, -0.1, 0,0,0,0,0};



// // circular buffer for last N IMU samples (uncompressed floats)
// struct ImuRaw {
//   float qx,qy,qz,qw;
//   float ax,ay,az;
//   float force; // Holds decimal value (0.00 to 31.00)
//   uint32_t ts;
// };
// ImuRaw imuRing[IMU_SAMPLES_PER_PACKET];
// int imuRingHead = 0; // next write index
// int imuRingCount = 0;

// // UWB distances
// float distances[8];

// // filtered positions (EMA) kept local
// float uwbX = 0, uwbY = 0;
// float uwbXf = 0, uwbYf = 0;



// // ---------------- helper serialization ----------------
// void writeUint8(uint8_t *buf, size_t &idx, uint8_t v){ buf[idx++] = v; }
// void writeUint16(uint8_t *buf, size_t &idx, uint16_t v){
//   buf[idx++] = v & 0xFF; buf[idx++] = (v>>8) & 0xFF;
// }
// void writeInt16(uint8_t *buf, size_t &idx, int16_t v){ writeUint16(buf, idx, (uint16_t)v); }
// void writeUint32(uint8_t *buf, size_t &idx, uint32_t v){
//   buf[idx++] = v & 0xFF; buf[idx++] = (v>>8) & 0xFF; buf[idx++] = (v>>16) & 0xFF; buf[idx++] = (v>>24) & 0xFF;
// }
// void writeFloatLE(uint8_t *buf, size_t &idx, float f){
//   union { float f; uint8_t b[4]; } u; u.f = f;
//   buf[idx++] = u.b[0]; buf[idx++] = u.b[1]; buf[idx++] = u.b[2]; buf[idx++] = u.b[3];
// }

// // ---------------- IMU buffering ----------------
// void pushImuSample(float qx,float qy,float qz,float qw,float ax,float ay,float az, float force, uint32_t ts){
//   imuRing[imuRingHead].qx = qx;
//   imuRing[imuRingHead].qy = qy;
//   imuRing[imuRingHead].qz = qz;
//   imuRing[imuRingHead].qw = qw;
//   imuRing[imuRingHead].ax = ax;
//   imuRing[imuRingHead].ay = ay;
//   imuRing[imuRingHead].az = az;
//   imuRing[imuRingHead].force = force;
//   imuRing[imuRingHead].ts = ts;
//   imuRingHead = (imuRingHead + 1) % IMU_SAMPLES_PER_PACKET;
//   if (imuRingCount < IMU_SAMPLES_PER_PACKET) imuRingCount++;
// }

// // get last N samples in chronological order
// void getLastImuSamples(ImuRaw *dest){
//   int count = imuRingCount;
//   int start = imuRingHead - count;
//   if (start < 0) start += IMU_SAMPLES_PER_PACKET;
  
//   for (int i=0;i<count;i++){
//     int idx = (start + i) % IMU_SAMPLES_PER_PACKET;
//     dest[i] = imuRing[idx];
//   }
  
//   if (count < IMU_SAMPLES_PER_PACKET){
//     ImuRaw pad = {0,0,0,1.0, 0,0,0, 0.0f, micros()};
//     if (count>0) pad = imuRing[(imuRingHead - 1 + IMU_SAMPLES_PER_PACKET) % IMU_SAMPLES_PER_PACKET];
//     for (int i=count;i<IMU_SAMPLES_PER_PACKET;i++){
//       dest[i] = pad;
//     }
//   }
// }

// ---------------- UWB decode ----------------


// ---------------- Trilateration ----------------
// bool trilaterate2d(float* distances, float* x, float* y, float* out_d0, float* out_d1, float* out_d2) {
//     *out_d0 = *out_d1 = *out_d2 = 0.0f;
//     struct ValidData { float x,y,dist; };
//     ValidData valid_data[8]; int valid_count=0;
//     for (int i=0;i<8;i++){
//       if (base_stations[i] != NULL && distances[i] > 0){
//         valid_data[valid_count].x = base_stations[i]->x;
//         valid_data[valid_count].y = base_stations[i]->y;
//         valid_data[valid_count].dist = distances[i];
//         valid_count++;
//       }
//     }
//     if (valid_count < 3) return false;
//     *out_d0 = valid_data[0].dist;
//     *out_d1 = valid_data[1].dist;
//     *out_d2 = valid_data[2].dist;
//     float x1 = valid_data[0].x, y1 = valid_data[0].y, r1 = valid_data[0].dist;
//     float A00, A01, A10, A11, b0, b1;
//     A00 = 2*(valid_data[1].x - x1);
//     A01 = 2*(valid_data[1].y - y1);
//     b0 = valid_data[1].dist*valid_data[1].dist - r1*r1 - valid_data[1].x*valid_data[1].x + x1*x1 - valid_data[1].y*valid_data[1].y + y1*y1;
//     A10 = 2*(valid_data[2].x - x1);
//     A11 = 2*(valid_data[2].y - y1);
//     b1 = valid_data[2].dist*valid_data[2].dist - r1*r1 - valid_data[2].x*valid_data[2].x + x1*x1 - valid_data[2].y*valid_data[2].y + y1*y1;
//     float det = A00*A11 - A01*A10;
//     if (fabs(det) < 1e-6) return false;
//     *x = -(b0 * A11 - b1 * A01) / det;
//     *y = -(A00 * b1 - A10 * b0) / det;
//     return true;
// }

// ---------------- ESP-NOW helper ----------------
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   // debug
// }


// --- MAIN ---
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, UWB_RX, UWB_TX);
  
  pinMode(A0, INPUT);
  
  Wire.begin();
  setupIMU();
  setupESPNow();
  
  Serial.println("[SENDER] Stream Ready");
}

void loop() {
  // Check IMU (High Freq)
  handleIMU();
  
  // Check UWB (Low Freq)
  handleUWB();
}