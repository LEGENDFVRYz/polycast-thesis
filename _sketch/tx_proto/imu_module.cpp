#include "imu_module.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// SCALING FACTORS
const float Q_SCALE = 32767.0f;    // quaternion -1..1 -> int16 [cite: 73]
const float A_SCALE = 1000.0f;     // accel m/s^2 * 1000 -> int16 [cite: 73]
const float F_SCALE = 100.0f;      // Force: Multiplies float by 100 to keep 2 decimals [cite: 73, 74]

// --- GLOBALS ---
static Adafruit_BNO08x bno08x(-1); // IMU Hardware Config [cite: 80]
static sh2_SensorValue_t sensorValue;
static bool imuFound = false;

static uint32_t imuPacketCount = 0;      // IMU Packet Counters [cite: 80]
static PacketIMU currentImuPacket;       // IMU Batching Buffer [cite: 80, 81]
static uint8_t imuSampleIndex = 0;       

void initIMU() {
    pinMode(A0, INPUT); // Prepare the force sensor analog pin

    long start = millis();
    while (millis() - start < 3000) {
        if (bno08x.begin_I2C(0x4A, &Wire)) { 
            imuFound = true; 
            break;
        }
        delay(50);
    }
    
    if (imuFound) {
        bno08x.enableReport(SH2_ROTATION_VECTOR, 5000); 
        bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000);
    } else {
        Serial.println("[IMU] BNO08x Initialization Failed!");
    }
}

bool processIMU(PacketIMU* out_packet) {
    if (!imuFound) return false;
    if (!bno08x.getSensorEvent(&sensorValue)) return false;

    // We need BOTH Rotation and Accel. 
    // Simple strategy: Update static vars, only "commit" sample when Accel arrives [cite: 93]
    static float cache_qx = 0, cache_qy = 0, cache_qz = 0, cache_qw = 1;

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
        float forceVal = (rawForce / 4095.0f) * 31.0f;  // note: 0 - 31 range [cite: 97]

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
            currentImuPacket.packetId = imuPacketCount++;
            
            // Transfer the batched packet out to the main scope
            *out_packet = currentImuPacket;
            
            imuSampleIndex = 0;   // --- note: reset index for next batch [cite: 92]
            return true;          // Flag the main loop to transmit the data
        }
    }
    
    return false; // Batch is not full yet
}