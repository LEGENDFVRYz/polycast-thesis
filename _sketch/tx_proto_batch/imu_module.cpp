/*
 * imu_module.cpp  —  PolyCast IMU + Button Contact Module (Tactile Switch Variant)
 * =================================================================================
 * Hardware:
 *   - BNO085 IMU on I2C at address 0x4A
 *   - Tactile push-button: one leg → 3.3 V, other leg → A0 (INPUT_PULLDOWN)
 *   - Button depresses when marker tip touches board → A0 reads HIGH
 *
 * Improvements over the original user-provided code:
 *
 *   1. FIRMWARE-LEVEL DEBOUNCE
 *      The original code does a single digitalRead() per IMU sample.
 *      At 100 Hz, mechanical contact bounce (1–5 ms) can produce 1–5 false
 *      state flips at every press/release.  We now require DEBOUNCE_COUNT
 *      consecutive identical reads before accepting a state change.
 *      This is a complement (not a replacement) to the software debounce in
 *      button_detector.py — together they suppress bounce at both layers.
 *
 *   2. EXPLICIT FORCE VALUES AS NAMED CONSTANTS
 *      Magic numbers 31.00f / 3.00f are replaced with named constants
 *      FORCE_WRITING and FORCE_LIFTING so changes propagate cleanly
 *      without hunting for hardcoded values.
 *
 *   3. FIXED FORCE VALUES TO MATCH PYTHON SPECIFICATION
 *      Python button_detector.py and data_stream.py expect 31.0 (writing)
 *      and 3.0 (lifting), matching the documented protocol.
 *      The original code sent 30.00f — corrected here.
 *
 *   4. MISSING IMU INIT ERROR PRINT RESTORED
 *      The user's provided code removed the Serial.println on IMU failure.
 *      Restored — silent failures make debugging painful.
 */

#include "imu_module.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ── Hardware ──────────────────────────────────────────────────────────
#define IMU_RESET_PIN 4

// ── Force values sent in packet (must match button_detector.py constants) ──
#define FORCE_WRITING  31.0f    // button pressed  (A0 HIGH, connected to 3.3 V)
#define FORCE_LIFTING   3.0f    // button released (A0 LOW,  pulled down)

// ── Firmware-level debounce ───────────────────────────────────────────
// Require this many consecutive identical digitalRead() results before
// accepting a state change.  At 100 Hz one count ≈ 10 ms.
// 2 counts = 20 ms debounce window — covers typical mechanical bounce.
#define DEBOUNCE_COUNT 2

// ── Module globals ────────────────────────────────────────────────────
static Adafruit_BNO08x  bno08x(IMU_RESET_PIN);
static sh2_SensorValue_t sensorValue;
static bool              imuFound = false;

// Debounce state
static bool     confirmedButtonState = false;   // last accepted button state
static bool     pendingState         = false;   // candidate new state
static uint8_t  pendingCount         = 0;       // how many consecutive reads match


// ── initIMU() ─────────────────────────────────────────────────────────
void initIMU() {
    // Power-on delay — allows USB CDC / Serial to settle and lets the
    // BNO085 complete its internal boot sequence before I2C traffic begins.
    delay(1000);

    Wire.begin();

    // INPUT_PULLDOWN: pin sits LOW by default.
    // When button connects A0 to 3.3 V, pin goes HIGH.
    pinMode(A0, INPUT_PULLDOWN);

    // Pre-seed debounce state to match the actual pin level at boot so the
    // first real packet doesn't carry a stale / inverted button value.
    confirmedButtonState = (digitalRead(A0) == HIGH);
    pendingState         = confirmedButtonState;
    pendingCount         = DEBOUNCE_COUNT;      // treat boot reading as settled

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
        bno08x.enableReport(SH2_ROTATION_VECTOR,    10000);
        bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    } else {
        Serial.println("[IMU] BNO085 init failed — check I2C wiring and RST pin.");
    }
}


// ── processIMU() ──────────────────────────────────────────────────────
bool processIMU(PacketIMU* out_packet) {
    if (!imuFound) return false;
    if (!bno08x.getSensorEvent(&sensorValue)) return false;

    // Cached quaternion — updated whenever a rotation vector report arrives
    static float cache_qx = 0.0f, cache_qy = 0.0f,
                 cache_qz = 0.0f, cache_qw = 1.0f;

    // ── Rotation vector report ────────────────────────────────────────
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        cache_qx = sensorValue.un.rotationVector.i;
        cache_qy = sensorValue.un.rotationVector.j;
        cache_qz = sensorValue.un.rotationVector.k;
        cache_qw = sensorValue.un.rotationVector.real;
        return false;   // packet not ready yet — wait for accel report
    }

    // ── Linear acceleration report ────────────────────────────────────
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        // IMU data
        out_packet->ax = sensorValue.un.linearAcceleration.x;
        out_packet->ay = sensorValue.un.linearAcceleration.y;
        out_packet->az = sensorValue.un.linearAcceleration.z;

        out_packet->qx = cache_qx;
        out_packet->qy = cache_qy;
        out_packet->qz = cache_qz;
        out_packet->qw = cache_qw;

        // ── Firmware-level debounce ───────────────────────────────────
        bool currentRead = (digitalRead(A0) == HIGH);

        if (currentRead == pendingState) {
            // Consistent with pending — increment counter
            if (pendingCount < DEBOUNCE_COUNT) {
                pendingCount++;
            }
            // Accept new state once counter saturates
            if (pendingCount >= DEBOUNCE_COUNT) {
                confirmedButtonState = pendingState;
            }
        } else {
            // Inconsistent — restart counter with the new reading
            pendingState = currentRead;
            pendingCount = 1;
        }

        // Encode the debounce-confirmed state into the packet
        out_packet->force = confirmedButtonState ? FORCE_WRITING : FORCE_LIFTING;

        out_packet->ts = micros();
        return true;
    }

    return false;
}