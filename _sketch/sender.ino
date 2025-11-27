// ###########################################
// SENDER (ESP32 NANO) - UWB + IMU
// WITH TRILATERATION, OFFSETS & DEBUG
// ###########################################

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// WIFI CHANNEL (Must match receiver: 1-13)
#define WIFI_CHANNEL 1

// -----------------------------------------
// Base Station Configuration
// -----------------------------------------
struct Position {
  float x, y;
};

Position* base0 = new Position{1.75, 0};
Position* base1 = new Position{1.75, 1.61};
Position* base2 = new Position{0, 0};
Position* base3 = NULL;
Position* base4 = NULL;
Position* base5 = NULL;
Position* base6 = NULL;
Position* base7 = NULL;

Position* base_stations[8] = {base0, base1, base2, base3, base4, base5, base6, base7};
float distance_offsets[8] = {-0.2, -0.18, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0};

// -----------------------------------------
// Data Structure to Send
// -----------------------------------------
typedef struct struct_combine_sensor_data {
    float x;            // raw UWB x
    float y;            // raw UWB y
    float filtered_x;   // filtered UWB x
    float filtered_y;   // filtered UWB y
    float qx, qy, qz, qw;  // IMU Quaternion
    float ax, ay, az;      // IMU Accelerometer
    uint32_t timestamp;
} struct_combine_sensor_data;

struct_combine_sensor_data myData;

// -----------------------------------------
// UWB UART Pins
// -----------------------------------------
#define UWB_RX 9
#define UWB_TX 8

// -----------------------------------------
// IMU Globals
// -----------------------------------------
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
bool imuFound = false;

// -----------------------------------------
// UWB Distance Storage
// -----------------------------------------
float distances[8];
float uwbX = 0, uwbY = 0;
float uwbXf = 0, uwbYf = 0;

// -----------------------------------------
// Receiver MAC Address (REPLACE WITH YOURS)
// -----------------------------------------
uint8_t receiverMAC[] = {0x80, 0xF3, 0xDA, 0x55, 0x9F, 0x6C};
esp_now_peer_info_t peerInfo;

// -----------------------------------------
// UWB Decode with Offsets
// -----------------------------------------
bool decodeUwbDistances(uint8_t* data, int dataLen, float* distances) {
  for (int i = 0; i < 8; i++) distances[i] = -1;

  if (dataLen < 35) {
    Serial.println("[UWB Decode] FAILED: Packet too short.");
    return false;
  }
  if (data[0] != 0xAA || data[1] != 0x25 || data[2] != 0x01) {
    Serial.println("[UWB Decode] FAILED: Header mismatch.");
    return false;
  }

  for (int i = 0; i < 8; i++) {
    int offset = 3 + (i * 4);
    uint32_t raw =
      data[offset] |
      (data[offset+1] << 8) |
      (data[offset+2] << 16) |
      (data[offset+3] << 24);

    if (raw > 0) {
      // Apply distance offset correction
      distances[i] = (raw / 1000.0) + distance_offsets[i];
    }
  }
  
  Serial.println("[UWB Decode] SUCCESS.");
  Serial.print("  Distances: ");
  for (int i = 0; i < 8; i++) {
    if (distances[i] > 0) {
      Serial.printf("[%d]=%.2fm ", i, distances[i]);
    }
  }
  Serial.println();
  
  return true;
}

// -----------------------------------------
// Trilateration (Least Squares Solution)
// -----------------------------------------
bool trilaterate2d(float* distances, float* x, float* y) {
  struct ValidData {
    float x, y, dist;
  };
  
  ValidData valid_data[8];
  int valid_count = 0;
  
  // Collect valid base stations
  for (int i = 0; i < 8; i++) {
    if (base_stations[i] != NULL && distances[i] > 0) {
      valid_data[valid_count].x = base_stations[i]->x;
      valid_data[valid_count].y = base_stations[i]->y;
      valid_data[valid_count].dist = distances[i];
      valid_count++;
    }
  }
  
  Serial.printf("  [Trilaterate] Valid anchors: %d\n", valid_count);
  
  if (valid_count < 3) {
    Serial.println("  [Trilaterate] FAILED: Need at least 3 anchors.");
    return false;
  }
  
  // Use first point as reference
  float x1 = valid_data[0].x;
  float y1 = valid_data[0].y;
  float r1 = valid_data[0].dist;
  
  // Build system of equations
  float A[7][2];
  float b[7];
  int eq_count = 0;
  
  for (int i = 1; i < valid_count && eq_count < 7; i++) {
    float xi = valid_data[i].x;
    float yi = valid_data[i].y;
    float ri = valid_data[i].dist;
    
    A[eq_count][0] = 2 * (xi - x1);
    A[eq_count][1] = 2 * (yi - y1);
    b[eq_count] = ri*ri - r1*r1 - xi*xi + x1*x1 - yi*yi + y1*y1;
    eq_count++;
  }
  
  // Solve 2x2 system (using first two equations)
  if (eq_count >= 2) {
    float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (abs(det) < 1e-6) {
      Serial.println("  [Trilaterate] FAILED: Singular matrix.");
      return false;
    }
    
    *x = -(b[0] * A[1][1] - b[1] * A[0][1]) / det;
    *y = -(A[0][0] * b[1] - A[1][0] * b[0]) / det;
    
    Serial.printf("  [Trilaterate] SUCCESS: (%.3f, %.3f)\n", *x, *y);
    return true;
  }
  
  return false;
}

// -----------------------------------------
// Compute UWB Position
// -----------------------------------------
void computeUwbPosition() {
  Serial.println("  [DEBUG] computeUwbPosition() called.");
  
  bool success = trilaterate2d(distances, &uwbX, &uwbY);
  
  if (success) {
    // Simple exponential filter
    uwbXf = 0.8 * uwbXf + 0.2 * uwbX;
    uwbYf = 0.8 * uwbYf + 0.2 * uwbY;
    
    Serial.printf("  [DEBUG]   Raw (X,Y): (%.3f, %.3f) | Filtered (Xf,Yf): (%.3f, %.3f)\n", 
                  uwbX, uwbY, uwbXf, uwbYf);
  } else {
    Serial.println("  [DEBUG]   Trilateration FAILED - using last filtered values.");
    // Keep last filtered values
  }
}

// -----------------------------------------
// IMU Init
// -----------------------------------------
void setupIMU() {
  Serial.println("[IMU] Trying to find BNO085...");
  
  long startTime = millis();
  bool found = false;
  
  while (millis() - startTime < 3000) {
    if (bno08x.begin_I2C(0x4A, &Wire)) {
      Serial.println("[IMU] SUCCESS: BNO085 found!");
      found = true;
      break;
    }
    delay(100);
  }

  if (!found) {
    Serial.println("******************************************");
    Serial.println("[IMU] FAILED: BNO085 not found!");
    Serial.println("Continuing without IMU data...");
    Serial.println("******************************************");
    imuFound = false;
    return;
  }

  imuFound = true;
  Serial.println("[IMU] Enabling Rotation Vector report.");
  bno08x.enableReport(SH2_ROTATION_VECTOR, 5000); // 200Hz
  Serial.println("[IMU] Enabling Linear Acceleration report.");
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000);
}

// -----------------------------------------
// IMU Read
// -----------------------------------------
void updateIMU() {
  if (imuFound && bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      Serial.println("  [DEBUG] IMU: Got Rotation Vector.");
      myData.qx = sensorValue.un.rotationVector.i;
      myData.qy = sensorValue.un.rotationVector.j;
      myData.qz = sensorValue.un.rotationVector.k;
      myData.qw = sensorValue.un.rotationVector.real;
    }
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      Serial.println("  [DEBUG] IMU: Got Linear Acceleration.");
      myData.ax = sensorValue.un.linearAcceleration.x;
      myData.ay = sensorValue.un.linearAcceleration.y;
      myData.az = sensorValue.un.linearAcceleration.z;
    }
  }
}

// -----------------------------------------
// ESP-NOW: Callback for send status
// -----------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("  [ESP-NOW] Send Status: Success");
  } else {
    Serial.println("  [ESP-NOW] Send Status: FAILED");
  }
}

// -----------------------------------------
// SETUP
// -----------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- ESP32 SENDER DEBUG ---");

  // Initialize UWB UART
  Serial.println("[UWB] Initializing Serial1 (UART)...");
  Serial1.begin(115200, SERIAL_8N1, UWB_RX, UWB_TX);
  Serial.println("[UWB] Serial1 OK.");

  // Initialize data struct
  memset(&myData, 0, sizeof(struct_combine_sensor_data));
  myData.qw = 1.0f;

  // Setup IMU
  setupIMU();

  // Setup WiFi & ESP-NOW
  Serial.println("[WiFi] Setting Mode to STA...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("[WiFi] Setting Power Save to NONE...");
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
    Serial.println("[WiFi] ERROR: Failed to set power save!");
    return;
  }

  Serial.printf("[WiFi] Setting Channel to: %d\n", WIFI_CHANNEL);
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("[WiFi] ERROR: Failed to set channel!");
    return;
  }

  Serial.println("[ESP-NOW] Initializing...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] ERROR: Failed to initialize!");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW] ERROR: Failed to add peer!");
    return;
  }
  
  Serial.println("[ESP-NOW] Peer Added OK.");
  Serial.printf("  Sending to: %02X:%02X:%02X:%02X:%02X:%02X\n",
                receiverMAC[0], receiverMAC[1], receiverMAC[2], 
                receiverMAC[3], receiverMAC[4], receiverMAC[5]);
  
  // Print base station configuration
  Serial.println("\n[CONFIG] Base Stations:");
  for (int i = 0; i < 8; i++) {
    if (base_stations[i] != NULL) {
      Serial.printf("  Base %d: (%.2f, %.2f) offset=%.2fm\n", 
                    i, base_stations[i]->x, base_stations[i]->y, distance_offsets[i]);
    }
  }
  
  Serial.println("\n--- SETUP COMPLETE. Starting loop... ---\n");
}

// -----------------------------------------
// Main Loop
// -----------------------------------------
void loop() {
  updateIMU();

  static uint8_t buffer[256];
  static int index = 0;
  static bool started = false;

  while (Serial1.available()) {
    uint8_t in = Serial1.read();

    if (!started && in == 0xAA) {
      Serial.println("[UWB] Found start byte 0xAA.");
      started = true;
      index = 0;
      buffer[index++] = in;
    }
    else if (started) {
      buffer[index++] = in;
      
      if (index >= 35) {
        Serial.println("[UWB] Buffer full (35 bytes). Decoding...");
        bool good = decodeUwbDistances(buffer, index, distances);

        if (good) {
          Serial.println("[UWB] PACKET OK.");
          
          // Compute position using trilateration
          computeUwbPosition();
          
          // Populate all UWB data
          myData.x = uwbX;
          myData.y = uwbY;
          myData.filtered_x = uwbXf;
          myData.filtered_y = uwbYf;
          
          // Set timestamp right before sending
          myData.timestamp = micros();

          // Send packet
          Serial.printf("[ESP-NOW] Sending packet, timestamp: %u\n", myData.timestamp);
          esp_err_t result = esp_now_send(receiverMAC, (uint8_t*)&myData, sizeof(myData));
          
          if (result != ESP_OK) {
            Serial.println("[ESP-NOW] ERROR: esp_now_send() failed immediately.");
          }
          
          Serial.println("---"); // Separator for readability
        } else {
          Serial.println("[UWB] PACKET BAD.");
        }

        started = false;
        index = 0;
      }
      
      if (index >= 256) {
        Serial.println("[UWB] ERROR: Buffer overflow. Resetting parser.");
        started = false;
        index = 0;
      }
    }
  }
}