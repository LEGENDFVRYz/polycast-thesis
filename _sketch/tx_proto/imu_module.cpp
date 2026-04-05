#include "imu_module.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// SCALING FACTORS
const float Q_SCALE = 32767.0f;    
const float A_SCALE = 1000.0f;     
const float F_SCALE = 1.0f;      

// >>> THE HARDWARE RESET FIX <<<
#define IMU_RESET_PIN 4 

// --- GLOBALS ---
static Adafruit_BNO08x bno08x(IMU_RESET_PIN); 
static sh2_SensorValue_t sensorValue;
static bool imuFound = false;

static uint32_t imuPacketCount = 0;      
static PacketIMU currentImuPacket;       
static uint8_t imuSampleIndex = 0;       

void initIMU() {
    // Wait for power stabilization
    delay(1000); 
    
    Wire.begin(); 
    
    // ESP32: If A0 fails, use a dedicated GPIO like 26 or 33
    pinMode(A0, INPUT_PULLDOWN); 

    Serial.println("[IMU] Searching for BNO08x...");
    long start = millis();
    while (millis() - start < 3000) {
        if (bno08x.begin_I2C(0x4A, &Wire)) { 
            imuFound = true; 
            break;
        }
        delay(50);
    }
    
    if (imuFound) {
        bno08x.enableReport(SH2_ROTATION_VECTOR, 10000); 
        bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
        Serial.println("[IMU] BNO08x Initialized Successfully.");
    } else {
        Serial.println("[IMU] Failed! Check wiring/Reset Pin.");
    }
}

bool processIMU(PacketIMU* out_packet) {
    if (!imuFound) return false;
    
    // Non-blocking check for new sensor data
    if (!bno08x.getSensorEvent(&sensorValue)) return false;

    static float cache_qx = 0, cache_qy = 0, cache_qz = 0, cache_qw = 1;

    // 1. Update Rotation Cache
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        cache_qx = sensorValue.un.rotationVector.i;
        cache_qy = sensorValue.un.rotationVector.j;
        cache_qz = sensorValue.un.rotationVector.k;
        cache_qw = sensorValue.un.rotationVector.real;
    } 
    
    // 2. Process Acceleration and Button (The Trigger)
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        float ax = sensorValue.un.linearAcceleration.x;
        float ay = sensorValue.un.linearAcceleration.y;
        float az = sensorValue.un.linearAcceleration.z;

        // Immediate button read
        bool isPressed = (digitalRead(A0) == HIGH);

        // Store Scaled Rotation
        currentImuPacket.samples[imuSampleIndex].qx = (int16_t)constrain(round(cache_qx * Q_SCALE), -32767, 32767);
        currentImuPacket.samples[imuSampleIndex].qy = (int16_t)constrain(round(cache_qy * Q_SCALE), -32767, 32767);
        currentImuPacket.samples[imuSampleIndex].qz = (int16_t)constrain(round(cache_qz * Q_SCALE), -32767, 32767);
        currentImuPacket.samples[imuSampleIndex].qw = (int16_t)constrain(round(cache_qw * Q_SCALE), -32767, 32767);
        
        // Store Scaled Acceleration
        currentImuPacket.samples[imuSampleIndex].ax = (int16_t)constrain(round(ax * A_SCALE), -32767, 32767);
        currentImuPacket.samples[imuSampleIndex].ay = (int16_t)constrain(round(ay * A_SCALE), -32767, 32767);
        currentImuPacket.samples[imuSampleIndex].az = (int16_t)constrain(round(az * A_SCALE), -32767, 32767);
        
        // Store Scaled Force (3.0 -> 300, 30.0 -> 3000)
        currentImuPacket.samples[imuSampleIndex].force = (int16_t)(isPressed ? 30 * F_SCALE : 3 * F_SCALE);

        currentImuPacket.samples[imuSampleIndex].ts = micros();

        // Increment index and check if packet is full
        imuSampleIndex++;
        if (imuSampleIndex >= 3) {
            currentImuPacket.packetId = imuPacketCount++;
            *out_packet = currentImuPacket; // Copy to output
            imuSampleIndex = 0;   
            return true;          
        }
    }
    
    return false; 
}