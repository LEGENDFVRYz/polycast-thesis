#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>

// ── Async IMU Packet — sent individually at 100 Hz ───────────────────
// ── Async IMU Packet — sent individually at 100 Hz ───────────────────
struct __attribute__((packed)) ImuPacket {
    uint8_t  type;               // 0x01
    uint32_t seq;                // per-stream sequence counter (caller sets)
    float    qx, qy, qz, qw;     // rotation vector quaternion
    float    ax, ay, az;         // linear acceleration (m/s²)
    float    gx, gy, gz;         // gyroscope (rad/s)  <-- NEW
    float    force;              // Raw FSR ADC value (0-4095)
    uint32_t ts;                 // micros() timestamp
};

/**
 * @brief Initialises the BNO085 over I2C at 400 kHz and the FSR voltage
 *        divider on A0.  Enables Rotation Vector and Linear Acceleration at
 *        200 Hz paired (Item D).
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

/**
 * @brief Persist the BNO085 Dynamic Calibration Data (DCD) to internal flash
 *        so it survives power-cycles.  Triggered remotely by the PC via the
 *        receiver (Item D).
 *
 * @return SH2 status code. 0 (SH2_OK) on success.
 */
int requestDcdSave();

#endif // IMU_MODULE_H
