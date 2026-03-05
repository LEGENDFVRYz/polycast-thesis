#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>

// --- PACKET STRUCTURES (BINARY STREAMING) ---
// Packet 1: High-Speed IMU Data (every ~3 samples) [cite: 75]
typedef struct __attribute__((packed)) ImuPacket{
    float qx, qy, qz, qw;
    float ax, ay, az;
    float force;
    uint32_t ts;
} PacketIMU;

/**
 * @brief Initializes the BNO08x IMU over I2C and sets up the analog force pin.
 * Enables Rotation Vector and Linear Acceleration reports at 5000us intervals.
 */
void initIMU();

/**
 * @brief Polls the IMU for new sensor events and batches them.
 * * @param out_packet Pointer to an empty PacketIMU struct.
 * @return true if 3 samples have been collected and the out_packet is ready to send.
 * @return false if the batch is still filling up or no new data is available.
 */
bool processIMU(PacketIMU* out_packet);


#endif // IMU_MODULE_H