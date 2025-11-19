// ###########################################
// SENDER (ESP32 NANO) - UWB + IMU
// FULLY REVISED CODE WITH DEBUG PRINTS
// ###########################################

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel

// DEFINE THE WIFI CHANNEL (Must be 1-13)
// BOTH SENDER AND RECEIVER MUST BE ON THE SAME CHANNEL
#define WIFI_CHANNEL 1


// Structure to send
typedef struct struct_combine_sensor_data {
    // --- UWB ---
    float x;            // raw UWB x
    float y;            // raw UWB y
    float filtered_x;   // filtered UWB x
    float filtered_y;   // filtered UWB y
    
    // --- IMU Quaternion ---
    float qx;
    float qy;
    float qz;
    float qw;

    // --- IMU Accelerometer ---
    float ax;
    float ay;
    float az;

    uint32_t timestamp; // Timestamp set right before sending
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
bool imuFound = false; // Flag to track if IMU was found

// -----------------------------------------
// UWB Distance Storage (FROM REFERENCE)
// -----------------------------------------
float distances[8];   // Raw distances
float uwbX = 0, uwbY = 0;       // UWB multilateration output
float uwbXf = 0, uwbYf = 0;      // Filtered outputs


// --------------------
// RECEIVER MAC ADDRESS
// --------------------
// (REPLACE WITH YOUR RECEIVER'S MAC)
uint8_t receiverMAC[] = {0x80, 0xF3, 0xDA, 0x55, 0x9F, 0x6C};
esp_now_peer_info_t peerInfo;


// -----------------------------------------
// UWB Decode Function (FROM REFERENCE)
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

    if (raw > 0) distances[i] = raw / 1000.0;
  }
  Serial.println("[UWB Decode] SUCCESS.");
  return true;
}

// -----------------------------------------
// UWB Position Compute (FROM REFERENCE)
// -----------------------------------------
void computeUwbPosition() {
  Serial.println("  [DEBUG] computeUwbPosition() called.");
  // Example: Replace these with your calculated values
  uwbX = distances[0];
  uwbY = distances[1];

  // Filter example (replace with Kalman/LPF)
  uwbXf = 0.8 * uwbXf + 0.2 * uwbX;
  uwbYf = 0.8 * uwbYf + 0.2 * uwbY;
  
  Serial.printf("  [DEBUG]   Raw (X,Y): (%.3f, %.3f) | Filtered (Xf,Yf): (%.3f, %.3f)\n", uwbX, uwbY, uwbXf, uwbYf);
}

// -----------------------------------------
// IMU Init
// -----------------------------------------
void setupIMU() {
  Serial.println("[IMU] Trying to find BNO085...");
  
  long startTime = millis();
  bool found = false;
  
  // Try for 3 seconds to find the IMU
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
    return; // Exit setupIMU, but don't freeze the whole program
  }

  imuFound = true;
  Serial.println("[IMU] Enabling Rotation Vector report.");
  bno08x.enableReport(SH2_ROTATION_VECTOR);
  Serial.println("[IMU] Enabling Linear Acceleration report.");
  bno08x.enableReport(SH2_LINEAR_ACCELERATION);
}

// -----------------------------------------
// IMU Read
// -----------------------------------------
void updateIMU() {
  // Only try to get data if the IMU was found during setup
  if (imuFound && bno08x.getSensorEvent(&sensorValue)) {
    
    // Note: We removed the timestamp from here. It's now set just before sending.

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
  delay(1000); // Wait for Serial Monitor to open
  Serial.println("\n--- ESP32 SENDER DEBUG ---");

  // *** Initialize Serial for UWB Module ***
  Serial.println("[UWB] Initializing Serial1 (UART)...");
  Serial1.begin(115200, SERIAL_8N1, UWB_RX, UWB_TX);
  Serial.println("[UWB] Serial1 OK.");

  // Initialize the data struct
  memset(&myData, 0, sizeof(struct_combine_sensor_data));
  myData.qw = 1.0f; // Set default orientation (w=1)

  // --- Setup IMU ---
  setupIMU();

  // --- Setup WiFi & ESP-NOW ---
  Serial.println("[WiFi] Setting Mode to STA...");
  WiFi.mode(WIFI_STA);
  Serial.println("[WiFi] Disconnecting from any AP...");
  WiFi.disconnect(); 

  // 1. Disable WiFi Power Save (IMPORTANT for Low Latency)
  Serial.println("[WiFi] Setting Power Save to NONE...");
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
    Serial.println("[WiFi] ERROR: Failed to set power save!");
    return;
  }
  Serial.println("[WiFi] Power Save OK.");

  // 2. Set the WiFi Channel
  Serial.printf("[WiFi] Setting Channel to: %d\n", WIFI_CHANNEL);
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("[WiFi] ERROR: Failed to set channel!");
    return;
  }
  Serial.println("[WiFi] Channel OK.");

  // 3. Initialize ESP-NOW
  Serial.println("[ESP-NOW] Initializing...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] ERROR: Failed to initialize!");
    return;
  }
  Serial.println("[ESP-NOW] Init OK.");

  // 4. Register the send callback
  Serial.println("[ESP-NOW] Registering Send Callback...");
  esp_now_register_send_cb(OnDataSent);

  // 5. Register peer
  Serial.println("[ESP-NOW] Registering Peer...");
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = WIFI_CHANNEL; 
  peerInfo.ifidx = WIFI_IF_STA;   
  peerInfo.encrypt = false;      

  // 6. Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW] ERROR: Failed to add peer!");
    return;
  }
  
  Serial.println("[ESP-NOW] Peer Added OK.");
  Serial.printf("  [ESP-NOW] Sending to: %02X:%02X:%02X:%02X:%02X:%02X\n",
                receiverMAC[0], receiverMAC[1], receiverMAC[2], receiverMAC[3], receiverMAC[4], receiverMAC[5]);
  
  Serial.println("\n--- SETUP COMPLETE. Starting loop... ---");
}


// -----------------------------------------
// Main Loop
// -----------------------------------------
void loop() {
  
  // --- STEP 1: Always check for new IMU data ---
  updateIMU(); 

  // --- STEP 2: Check for UWB data (State Machine) ---
  static uint8_t buffer[256];
  static int index = 0;
  static bool started = false;

  while (Serial1.available()) {
    // Serial.println("  [DEBUG] Serial1.available() > 0"); // Uncomment if you suspect UART isn't working at all
    uint8_t in = Serial1.read();

    if (!started && in == 0xAA) {
      Serial.println("[UWB] Found start byte 0xAA.");
      started = true;
      index = 0;
      buffer[index++] = in;
    }
    else if (started) {
      buffer[index++] = in;
      
      // Check if we have a full packet (35 bytes expected)
      if (index >= 35) { 
        Serial.println("[UWB] Buffer full (35 bytes). Decoding...");
        bool good = decodeUwbDistances(buffer, index, distances);

        if (good) {
          Serial.println("[UWB] PACKET OK.");
          
          // --- COMPUTE & POPULATE ---
          computeUwbPosition();
          
          // *** CRITICAL FIX 1: Populate all UWB data ***
          myData.x = uwbX;
          myData.y = uwbY;
          myData.filtered_x = uwbXf;
          myData.filtered_y = uwbYf;
          
          // *** CRITICAL FIX 2: Set timestamp right before sending ***
          myData.timestamp = micros();

          // --- SEND PACKET ---
          Serial.printf("[ESP-NOW] Sending packet, timestamp: %u\n", myData.timestamp);
          esp_err_t result = esp_now_send(receiverMAC, (uint8_t*)&myData, sizeof(myData));
          
          if (result != ESP_OK) {
            Serial.println("[ESP-NOW] ERROR: esp_now_send() failed immediately.");
          }
        } else {
          Serial.println("[UWB] PACKET BAD.");
        }

        // Reset for next packet
        started = false;
        index = 0;
      }
      
      // Buffer overflow guard
      if (index >= 256) {
        Serial.println("[UWB] ERROR: Buffer overflow. Resetting parser.");
        started = false;
        index = 0;
      }
    }
  }
}