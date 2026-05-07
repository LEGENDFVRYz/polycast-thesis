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
#include <sh2.h>           // SH-2 API: sh2_saveDcdNow, sh2_setCalConfig
#include <sh2_SensorValue.h>

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
    // 400 kHz I2C — required headroom for paired 200 Hz reports (Item D).
    Wire.setClock(400000);

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
        // 5 000 µs interval = 200 Hz paired reports (Item D).  Aggregate
        // SHTP throughput on I2C tops out near 600 reports/sec; two paired
        // reports at 200 Hz = 400/sec, comfortably under the limit.
        bno08x.enableReport(SH2_ROTATION_VECTOR,     5000);
        bno08x.enableReport(SH2_LINEAR_ACCELERATION,  5000);
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
        Serial.println("[IMU] BNO085 initialised at 200 Hz paired. Raw FSR enabled.");
    } else {
        Serial.println("[IMU] BNO085 init failed — check I2C wiring and RST pin.");
    }
}


// ── requestDcdSave() ─────────────────────────────────────────────────
// Called via remote command (CAL).  Enables runtime self-cal on accel,
// gyro, and mag, then persists the current DCD to chip flash.  On the
// next power-up the BNO085 restores DCD automatically.
int requestDcdSave() {
    if (!imuFound) return -1;
    // Keep all background self-cal sources enabled.
    sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
    return sh2_saveDcdNow();
}


// ── processIMU() ─────────────────────────────────────────────────────
bool processIMU(ImuPacket* out) {
    if (!imuFound) return false;
    if (!bno08x.getSensorEvent(&sensorValue)) return false;

    // Cached quaternion — updated on every rotation vector report
    static float cache_qx = 0.0f, cache_qy = 0.0f,
                 cache_qz = 0.0f, cache_qw = 1.0f;
    static float cache_gx = 0.0f, cache_gy = 0.0f, cache_gz = 0.0f;

    // ── Rotation vector report → cache and wait for accel ────────────
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        cache_qx = sensorValue.un.rotationVector.i;
        cache_qy = sensorValue.un.rotationVector.j;
        cache_qz = sensorValue.un.rotationVector.k;
        cache_qw = sensorValue.un.rotationVector.real;
        return false;   // not ready — wait for acceleration
    }

    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        cache_gx = sensorValue.un.gyroscope.x;
        cache_gy = sensorValue.un.gyroscope.y;
        cache_gz = sensorValue.un.gyroscope.z;
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

        out->gx = cache_gx;
        out->gy = cache_gy;
        out->gz = cache_gz;

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