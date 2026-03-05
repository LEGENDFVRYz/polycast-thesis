#include "imu_module.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

 

// >>> THE HARDWARE RESET FIX <<<
#define IMU_RESET_PIN 4 

// --- GLOBALS ---
static Adafruit_BNO08x bno08x(IMU_RESET_PIN); // Tell the library to manage the reset pin
static sh2_SensorValue_t sensorValue;
static bool imuFound = false;


void initIMU() {
    // >>> THE POWERBANK FIX <<<
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
        bno08x.enableReport(SH2_ROTATION_VECTOR, 10000); 
        bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
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
        // We catch the accel, attach the cached quat, and package the float struct!
        out_packet->ax = sensorValue.un.linearAcceleration.x;
        out_packet->ay = sensorValue.un.linearAcceleration.y;
        out_packet->az = sensorValue.un.linearAcceleration.z;

        out_packet->qx = cache_qx;
        out_packet->qy = cache_qy;
        out_packet->qz = cache_qz;
        out_packet->qw = cache_qw;

        int rawForce = analogRead(A0);
        out_packet->force = (rawForce / 4095.0f) * 31.0f;  
        out_packet->ts = micros();

        return true;
    }
    
    return false; 
}