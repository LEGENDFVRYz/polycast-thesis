#include "uwb_module.h"

// Reference array to hold the distances of the UWB module
int live_distances[MAX_ANCHOR_LIST_SIZE];

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("--- Booting Modular UWB Swarm System ---");

    // Initialize the UWB hardware
    initUWBConfig();
}

void loop() {
    // [CORE 1] Pulling One UWB Cycle
    bool is_locked = runUWBCycle(live_distances);
    
    // [CORE 1] Pulling One UWB Cycle
    if (is_locked) {
        char print_buf[128];
        snprintf(print_buf, sizeof(print_buf), "MAIN CODE DATA -> [A0:%d, A1:%d, A2:%d, A3:%d, A4:%d, A5:%d, A6:%d, A7:%d]", 
                 live_distances[0], live_distances[1], live_distances[2], live_distances[3],
                 live_distances[4], live_distances[5], live_distances[6], live_distances[7]);
        Serial.println(print_buf); 
    } else {
        Serial.println("[MAIN CODE] Network lost. Hunting for Master Anchor...");
    }
}