#include "imu_module.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// SCALING FACTORS
const float Q_SCALE = 32767.0f;    
const float A_SCALE = 1000.0f;     
const float F_SCALE = 100.0f;      

// >>> THE HARDWARE RESET FIX <<<
// Define the pin connected to the BNO08x RST pin. 
// This forces the IMU to reboot whenever the ESP32 reboots.
#define IMU_RESET_PIN 4 

// --- GLOBALS ---
static Adafruit_BNO08x bno08x(IMU_RESET_PIN); // Tell the library to manage the reset pin
static sh2_SensorValue_t sensorValue;
static bool imuFound = false;

static uint32_t imuPacketCount = 0;      
static PacketIMU currentImuPacket;       
static uint8_t imuSampleIndex = 0;       

void initIMU() {
    // >>> THE POWERBANK FIX <<<
    // Wait 1 full second for the powerbank voltage to stabilize 
    // and the BNO08x's internal processor to boot up.
    delay(1000); 
    
    Wire.begin(); 
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
        Serial.println("[IMU] BNO08x Initialization Failed! Check I2C wiring and RST pin.");
    }
}

bool processIMU(PacketIMU* out_packet) {
    if (!imuFound) return false;
    if (!bno08x.getSensorEvent(&sensorValue)) return false;

    static float cache_qx = 0, cache_qy = 0, cache_qz = 0, cache_qw = 1;

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        cache_qx = sensorValue.un.rotationVector.i;
        cache_qy = sensorValue.un.rotationVector.j;
        cache_qz = sensorValue.un.rotationVector.k;
        cache_qw = sensorValue.un.rotationVector.real;
    } 
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        float ax = sensorValue.un.linearAcceleration.x;
        float ay = sensorValue.un.linearAcceleration.y;
        float az = sensorValue.un.linearAcceleration.z;

        int rawForce = analogRead(A0);
        float forceVal = (rawForce / 4095.0f) * 31.0f;  

        currentImuPacket.samples[imuSampleIndex].qx = (int16_t)constrain(round(cache_qx * Q_SCALE), -32767, 32767);
        currentImuPacket.samples[imuSampleIndex].qy = (int16_t)constrain(round(cache_qy * Q_SCALE), -32767, 32767);
        currentImuPacket.samples[imuSampleIndex].qz = (int16_t)constrain(round(cache_qz * Q_SCALE), -32767, 32767);
        currentImuPacket.samples[imuSampleIndex].qw = (int16_t)constrain(round(cache_qw * Q_SCALE), -32767, 32767);
        
        currentImuPacket.samples[imuSampleIndex].ax = (int16_t)constrain(round(ax * A_SCALE), -32767, 32767);
        currentImuPacket.samples[imuSampleIndex].ay = (int16_t)constrain(round(ay * A_SCALE), -32767, 32767);
        currentImuPacket.samples[imuSampleIndex].az = (int16_t)constrain(round(az * A_SCALE), -32767, 32767);
        
        currentImuPacket.samples[imuSampleIndex].force = (int16_t)(forceVal * F_SCALE);
        currentImuPacket.samples[imuSampleIndex].ts = micros();

        imuSampleIndex++;
        if (imuSampleIndex >= 3) {
            currentImuPacket.packetId = imuPacketCount++;
            *out_packet = currentImuPacket;
            imuSampleIndex = 0;   
            return true;          
        }
    }
    
    return false; 
}