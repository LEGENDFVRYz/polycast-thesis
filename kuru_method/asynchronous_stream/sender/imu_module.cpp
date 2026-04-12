/*
 * imu_module.cpp  —  PolyCast Async IMU + FSR Contact Module (Raw Data)
 * ==============================================================
 * Hardware:
 * - BNO085 IMU on I2C at address 0x4A, reset pin GPIO 4
 * - FSR Voltage Divider: 3.3V -> FSR -> A0 -> 10k Resistor -> GND
 * - FSR depresses when marker tip touches board -> A0 voltage rises
 * * Note: All filtering, thresholds, and debouncing are offloaded 
 * to the main processor's sensor fusion algorithm.
 */

#include "imu_module.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ── Hardware ──────────────────────────────────────────────────────────
#define IMU_RESET_PIN 4
#define FSR_PIN A0

// ── Module globals ───────────────────────────────────────────────────
static Adafruit_BNO08x   bno08x(IMU_RESET_PIN);
static sh2_SensorValue_t sensorValue;
static bool              imuFound = false;

// ── initIMU() ────────────────────────────────────────────────────────
void initIMU() {
    // Power-on delay — allows USB CDC / powerbank voltage to stabilise
    delay(1000);

    Wire.begin();

    // The external 10k resistor acts as the hardware pulldown.
    pinMode(FSR_PIN, INPUT);

    // BNO085 initialisation with 3-second timeout
    long startMs = millis();
    while (millis() - startMs < 3000) {
        if (bno08x.begin_I2C(0x4A, &Wire)) {
            imuFound = true;
            break;
        }
        delay(50);
    }

    if (imuFound) {
        // 10 000 µs interval = 100 Hz for both reports
        bno08x.enableReport(SH2_ROTATION_VECTOR,     10000);
        bno08x.enableReport(SH2_LINEAR_ACCELERATION,  10000);
        Serial.println("[IMU] BNO085 initialised at 100 Hz. Raw FSR enabled.");
    } else {
        Serial.println("[IMU] BNO085 init failed — check I2C wiring and RST pin.");
    }
}


// ── processIMU() ─────────────────────────────────────────────────────
bool processIMU(ImuPacket* out) {
    if (!imuFound) return false;
    if (!bno08x.getSensorEvent(&sensorValue)) return false;

    // Cached quaternion — updated on every rotation vector report
    static float cache_qx = 0.0f, cache_qy = 0.0f,
                 cache_qz = 0.0f, cache_qw = 1.0f;

    // ── Rotation vector report → cache and wait for accel ────────────
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        cache_qx = sensorValue.un.rotationVector.i;
        cache_qy = sensorValue.un.rotationVector.j;
        cache_qz = sensorValue.un.rotationVector.k;
        cache_qw = sensorValue.un.rotationVector.real;
        return false;   // not ready — wait for acceleration
    }

    // ── Linear acceleration report → build packet ────────────────────
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        out->type = 0x01;
        // seq is set by the caller (sender.ino)

        out->qx = cache_qx;
        out->qy = cache_qy;
        out->qz = cache_qz;
        out->qw = cache_qw;

        out->ax = sensorValue.un.linearAcceleration.x;
        out->ay = sensorValue.un.linearAcceleration.y;
        out->az = sensorValue.un.linearAcceleration.z;

        // ── Read Raw Analog FSR Signal (0-4095) ──────────────────────
        int rawFSR = analogRead(FSR_PIN);

        // Cast the raw 12-bit integer to the float expected by the packet
        out->force = (float)rawFSR;
        
        out->ts = micros();
        return true;
    }

    return false;
}