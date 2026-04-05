#ifndef UWB_MODULE_H
#define UWB_MODULE_H

#include <Arduino.h>

// -- PINNED USED ---
// #define PIN_RST 3
// #define PIN_IRQ 2
// #define PIN_SS  10


// --- PACKET STRUCTURES ---
typedef struct __attribute__((packed)) {
    uint8_t type = 0x02;
    uint32_t packetId;
    float dist0;
    float dist1;
    float dist2;
    float dist3;
    uint32_t ts;
} PacketUWB;


// Anchor Mapping
const int MAP_DIST0 = 0;
const int MAP_DIST1 = 2;
const int MAP_DIST2 = 4;
const int MAP_DIST3 = 6;


// Maximum number of anchors the system can track
#define MAX_ANCHOR_LIST_SIZE 8


/*! ------------------------------------------------------------------------------------
 *  @brief   Initializes the DW3000 UWB hardware and configures ESP32 SPI bus.
 *
 *           This function handles the core setup of the UWB module. It executes a 
 *           soft-reset, applies the copper trace antenna delays, and overclocks 
 *           the SPI bus to 36MHz to completely eliminate data transfer bottlenecks.
 *           It matches the AT+SETCFG(X, Y, 1, 1) of the BUO3 Dev Kit Default Config.
 *
 *  @note    This must be called exactly once inside the setup() loop.
 */
void initUWBConfig();


/*! ------------------------------------------------------------------------------------
 *  @brief   Executes a single TDMA Superframe cycle and populate the given array.
 *
 *           This engine broadcasts a POLL packet, and opens a high-speed tracking window  
 *           to catch sequentially cascading anchors. It automatically calculates 
 *           network phase-shifts and natively manages the 100ms TDMA sleep schedule.
 *
 *  @return true  The ESP32 is successfully phase-locked with the Master Anchor.
 *  @return false The ESP32 lost network and will listen/waiting for Anchors.
 */
bool runUWBCycle(int* out_distances);


#endif