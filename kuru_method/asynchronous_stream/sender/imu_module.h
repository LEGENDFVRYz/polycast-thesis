#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>

// ── Async IMU Packet — sent individually at 100 Hz ───────────────────
struct __attribute__((packed)) ImuPacket {
    uint8_t  type;               // 0x01
    uint32_t seq;                // per-stream sequence counter (caller sets)
    float    qx, qy, qz, qw;   // rotation vector quaternion
    float    ax, ay, az;        // linear acceleration (m/s²)
    float    force;             // Raw FSR ADC value (0-4095)
    uint32_t ts;                // micros() timestamp
};

/**
 * @brief Initialises the BNO085 over I2C and the FSR voltage divider on A0.
 *        Enables Rotation Vector and Linear Acceleration at 100 Hz.
 */
void initIMU();

/**
 * @brief Polls the BNO085 for a new sensor event.
 *
 * @param out  Pointer to an ImuPacket.  On success, all fields except
 *             `seq` are populated (caller sets seq before sending).
 * @return true   A LINEAR_ACCELERATION event arrived — packet is ready.
 * @return false  No new acceleration event yet (rotation cached internally).
 */
bool processIMU(ImuPacket* out);

#endif // IMU_MODULE_H
