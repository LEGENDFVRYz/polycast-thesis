/*
 * imu_module.cpp  —  PolyCast Async IMU + Button Contact Module
 * ==============================================================
 * Hardware:
 *   - BNO085 IMU on I2C at address 0x4A, reset pin GPIO 4
 *   - Tactile push-button: one leg → 3.3 V, other leg → A0 (INPUT_PULLDOWN)
 *   - Button depresses when marker tip touches board → A0 reads HIGH
 *
 * Key difference from the batched version:
 *   processIMU() returns true on EVERY LINEAR_ACCELERATION event
 *   (no batching of 3 or 5 samples).  Each sample becomes its own
 *   ESP-NOW packet, sent at the sensor's natural 100 Hz rate.
 */

#include "imu_module.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ── Hardware ──────────────────────────────────────────────────────────
#define IMU_RESET_PIN 4

// ── Force values (must match button_detector.py constants) ───────────
#define FORCE_WRITING  31.0f    // button pressed  (A0 HIGH)
#define FORCE_LIFTING   3.0f    // button released (A0 LOW)

// ── Firmware-level debounce ──────────────────────────────────────────
// 2 consecutive identical digitalRead() results before accepting change.
// At 100 Hz one count ≈ 10 ms → 20 ms debounce window.
#define DEBOUNCE_COUNT 2

// ── Module globals ───────────────────────────────────────────────────
static Adafruit_BNO08x   bno08x(IMU_RESET_PIN);
static sh2_SensorValue_t sensorValue;
static bool              imuFound = false;

// Debounce state
static bool    confirmedButtonState = false;
static bool    pendingState         = false;
static uint8_t pendingCount         = 0;


// ── initIMU() ────────────────────────────────────────────────────────
void initIMU() {
    // Power-on delay — allows USB CDC / powerbank voltage to stabilise
    // and BNO085 internal boot sequence to complete before I2C traffic.
    delay(1000);

    Wire.begin();

    // INPUT_PULLDOWN: pin sits LOW by default.
    // When button connects A0 to 3.3 V, pin goes HIGH.
    pinMode(A0, INPUT_PULLDOWN);

    // Pre-seed debounce state to the actual pin level at boot
    confirmedButtonState = (digitalRead(A0) == HIGH);
    pendingState         = confirmedButtonState;
    pendingCount         = DEBOUNCE_COUNT;

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
        Serial.println("[IMU] BNO085 initialised at 100 Hz.");
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

        // ── Firmware-level debounce ──────────────────────────────────
        bool currentRead = (digitalRead(A0) == HIGH);

        if (currentRead == pendingState) {
            if (pendingCount < DEBOUNCE_COUNT) {
                pendingCount++;
            }
            if (pendingCount >= DEBOUNCE_COUNT) {
                confirmedButtonState = pendingState;
            }
        } else {
            pendingState = currentRead;
            pendingCount = 1;
        }

        out->force = confirmedButtonState ? FORCE_WRITING : FORCE_LIFTING;
        out->ts    = micros();
        return true;
    }

    return false;
}
