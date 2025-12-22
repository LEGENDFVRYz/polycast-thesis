// SENDER CODE - ARDUINO NANO ESP32
// IMU - 100 Hz
// UWB - 20 Hz

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// --- CONFIGURATION ---
const uint8_t RECEIVER_MAC[] = {0x80, 0xF3, 0xDA, 0x55, 0x9F, 0x6C}; // <--- CHANGE THIS
#define WIFI_CHANNEL 1
#define FORCE_PIN A0
#define UWB_RX 9
#define UWB_TX 8

// --- TIMING ---
const unsigned long INTERVAL_US = 10000; // 10ms
unsigned long lastTime = 0;

// --- DATA STRUCTURES ---
struct ImuSample {
  float qx, qy, qz, qw; 
  float ax, ay, az;     
  float force;          
  uint32_t ts;          
}; 

struct Packet {
  float dist0; // Anchor 0 Distance
  float dist1; // Anchor 1 Distance
  float dist2; // Anchor 2 Distance
  ImuSample samples[5]; // Batch of 5 samples
}; 
// Total Size: 12 bytes (UWB) + 180 bytes (IMU) = 192 bytes. (Safe < 250)

Packet myPacket;
int sampleCounter = 0;

// --- GLOBALS ---
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

// "Latest" variables (Sample & Hold)
float latest_qx, latest_qy, latest_qz, latest_qw;
float latest_ax, latest_ay, latest_az;
float latest_d0 = -1, latest_d1 = -1, latest_d2 = -1; // Default to -1 if no signal

// --- UWB DECODER (Extracts 3 Distances) ---
void pollUWB() {
  static uint8_t buf[256];
  static int idx = 0;
  static bool started = false;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    
    // 1. Find Header 0xAA
    if (!started) {
      if (b == 0xAA) { started = true; idx = 0; buf[idx++] = b; }
    } else {
      buf[idx++] = b;
      
      // 2. Wait for full packet (35 bytes based on your previous code)
      if (idx >= 35) {
        // Check Header match
        if (buf[1] == 0x25 && buf[2] == 0x01) {
          // Extract 3 Distances (Indices based on AI Thinker Protocol)
          // Distance 1 (Offset 3)
          uint32_t r0 = (uint32_t)buf[3] | ((uint32_t)buf[4]<<8) | ((uint32_t)buf[5]<<16) | ((uint32_t)buf[6]<<24);
          // Distance 2 (Offset 7)
          uint32_t r1 = (uint32_t)buf[7] | ((uint32_t)buf[8]<<8) | ((uint32_t)buf[9]<<16) | ((uint32_t)buf[10]<<24);
          // Distance 3 (Offset 11)
          uint32_t r2 = (uint32_t)buf[11] | ((uint32_t)buf[12]<<8) | ((uint32_t)buf[13]<<16) | ((uint32_t)buf[14]<<24);

          // Convert to float (Meters) and update "Latest"
          if(r0 > 0) latest_d0 = r0 / 1000.0f;
          if(r1 > 0) latest_d1 = r1 / 1000.0f;
          if(r2 > 0) latest_d2 = r2 / 1000.0f;
        }
        started = false; idx = 0;
      }
    }
  }
}

// --- IMU READER ---
void pollIMU() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      latest_qx = sensorValue.un.rotationVector.i;
      latest_qy = sensorValue.un.rotationVector.j;
      latest_qz = sensorValue.un.rotationVector.k;
      latest_qw = sensorValue.un.rotationVector.real;
    } 
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      latest_ax = sensorValue.un.linearAcceleration.x;
      latest_ay = sensorValue.un.linearAcceleration.y;
      latest_az = sensorValue.un.linearAcceleration.z;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, UWB_RX, UWB_TX);
  Wire.begin();
  pinMode(FORCE_PIN, INPUT);

  if (bno08x.begin_I2C(0x4A, &Wire)) {
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000); 
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000); 
  }

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK)
    delay(1000);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  pollIMU();
  pollUWB();

  unsigned long now = micros();
  if (now - lastTime >= INTERVAL_US) {
    lastTime = now;

    int rawForce = analogRead(FORCE_PIN);
    float forceVal = (rawForce / 4095.0f) * 100.0f; 

    myPacket.samples[sampleCounter].qx = latest_qx;
    myPacket.samples[sampleCounter].qy = latest_qy;
    myPacket.samples[sampleCounter].qz = latest_qz;
    myPacket.samples[sampleCounter].qw = latest_qw;
    myPacket.samples[sampleCounter].ax = latest_ax;
    myPacket.samples[sampleCounter].ay = latest_ay;
    myPacket.samples[sampleCounter].az = latest_az;
    myPacket.samples[sampleCounter].force = forceVal;
    myPacket.samples[sampleCounter].ts = now;

    sampleCounter++;

    if (sampleCounter >= 5) {
      // Add the 3 Latest UWB Distances
      myPacket.dist0 = latest_d0;
      myPacket.dist1 = latest_d1;
      myPacket.dist2 = latest_d2;

      esp_now_send(RECEIVER_MAC, (uint8_t *) &myPacket, sizeof(myPacket));
      sampleCounter = 0;
    }
  }
}