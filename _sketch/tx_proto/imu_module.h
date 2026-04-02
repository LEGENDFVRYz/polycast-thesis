#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>

// --- PACKET STRUCTURES (BINARY STREAMING) ---
typedef struct __attribute__((packed)) {
    uint8_t type = 0x01;
    uint32_t packetId; // Counter to detect dropped packets
    struct {
        int16_t qx, qy, qz, qw;
        int16_t ax, ay, az;
        int16_t force;
        uint32_t ts;
    } samples[3];
} PacketIMU;

/**
 * @brief Initializes the BNO08x IMU and button pin.
 */
void initIMU();

/**
 * @brief Polls IMU and batches 3 samples before returning true.
 */
bool processIMU(PacketIMU* out_packet);

#endif