#include "uwb_module.h"
#include "dw3000.h"
#include "dw3000_port.h"
#include <SPI.h> 

extern void reselect(uint8_t ss);

#define PIN_RST 3
#define PIN_IRQ 2
#define PIN_SS  10

// --- ANTENNA DELAY CALIBRATIONS ---
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385



// --- PACKET STRUCTURE DEFINITIONS ---
#pragma pack(push,1)

typedef struct {
    uint16_t frameCtrl;    
    uint8_t  seqNum;       
    uint16_t panID;        
    uint16_t destAddr;     
    uint16_t sourceAddr;   
} mac_header_ss_t;

typedef struct {
    uint8_t  fCode;         
    int32_t  slotCorr_ms;   
    uint8_t  rNum;          
    uint16_t a2t_usercmd;   
    uint16_t pre_dist_cm;   
} resp_t;

typedef struct {
    mac_header_ss_t mac;
    resp_t          resp;
    uint8_t         fcs[2]; 
} resp_msg_t;

typedef struct {
    uint8_t  fCode;                        
    uint8_t  rNum;                         
    uint64_t pollTx_ts;                    
    uint64_t responseRx_ts[MAX_ANCHOR_LIST_SIZE]; 
    uint64_t finalTx_ts;                   
    uint8_t  rxResponseMask;               
    uint16_t dist_cm[MAX_ANCHOR_LIST_SIZE];
} final_t;

typedef struct {
    mac_header_ss_t mac;
    final_t         final;
    uint8_t         fcs[2];
} final_msg_t;

#pragma pack(pop)
// ------------------------------------------------------------

static dwt_config_t config = {
    5, DWT_PLEN_64, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, 
    DWT_PHRMODE_STD, DWT_PHRRATE_STD, (65 + 8 - 8), 
    DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0      
};

extern dwt_txconfig_t txconfig_options;
static uint8_t rx_buffer[127];

static uint8_t tx_poll_msg[] = {
    0x88, 0x41, 0x00, 0x11, 0x11, 0xFF, 0xFF, 0x00, 0x00, 0x1A, 0x00
};

final_msg_t my_final_msg;



// Helper function to extract 40-bit hardware timestamps
void get_timestamp(uint64_t *ts, bool is_tx) {
    uint8_t ts_buf[5];
    if (is_tx) dwt_readtxtimestamp(ts_buf);
    else dwt_readrxtimestamp(ts_buf);
    
    *ts = 0;
    for (int i = 4; i >= 0; i--) { *ts = (*ts << 8) + ts_buf[i]; }
}

void initUWBConfig() {
    spiBegin(PIN_IRQ, PIN_RST);
    reselect(PIN_SS); delay(5);
    pinMode(PIN_RST, OUTPUT); digitalWrite(PIN_RST, LOW);
    delay(5); pinMode(PIN_RST, INPUT); delay(30); 

    while (!dwt_checkidlerc()) { delay(10); }
    dwt_softreset(); delay(200);
    while (!dwt_checkidlerc()) { delay(10); }

    dwt_initialise(DWT_DW_INIT);
    dwt_configure(&config);
    dwt_configuretxrf(&txconfig_options);
    
    SPI.setFrequency(36000000); 
    
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxtimeout(0); 
    dwt_setrxaftertxdelay(300);

    my_final_msg.mac.frameCtrl = 0x4188;
    my_final_msg.mac.panID = 0x1111;
    my_final_msg.mac.destAddr = 0xFFFF;
    my_final_msg.mac.sourceAddr = 0x0000;
    my_final_msg.final.fCode = 0x1C; 
    
    for(int i = 0; i < MAX_ANCHOR_LIST_SIZE; i++) {
        my_final_msg.final.responseRx_ts[i] = 0;
        my_final_msg.final.dist_cm[i] = 0;
    }
}

bool runUWBCycle(int* out_distances) {
    uint32_t superframe_start_ms = millis();
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);

    tx_poll_msg[2]++; 
    tx_poll_msg[10]++; 

    for(int i = 0; i < MAX_ANCHOR_LIST_SIZE; i++) {
        out_distances[i] = -1; 
    }
    
    uint8_t rxResponseMask = 0;             
    int32_t latest_slot_corr = 0;           

    noInterrupts(); 

    dwt_setrxtimeout(3400); 
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); 
    dwt_writetxfctrl(sizeof(tx_poll_msg) + 2, 0, 0); 
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    
    uint32_t tx_wait_start = micros();
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & (1UL<<7))) {
        if (micros() - tx_wait_start > 5000) break; 
    }; 
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF); 

    uint64_t pollTx_ts;
    get_timestamp(&pollTx_ts, true);  
    
    uint32_t rx_window_start = micros();

    while (true) {
        uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        
        if (status_reg & (1UL<<14)) { 
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & 0x7F;
            if (frame_len <= sizeof(rx_buffer)) dwt_readrxdata(rx_buffer, frame_len, 0);
            uint8_t ts_buf[5];
            dwt_readrxtimestamp(ts_buf);
            
            dwt_setrxtimeout(600); 
            dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF); 
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            
            if (frame_len > 9 && rx_buffer[9] == 0x1B) { 
                resp_msg_t* anchor_reply = (resp_msg_t*)rx_buffer;
                uint16_t anchor_id = anchor_reply->mac.sourceAddr;
                
                if (anchor_id < MAX_ANCHOR_LIST_SIZE) {
                    uint64_t rx_ts = 0;
                    for (int i = 4; i >= 0; i--) rx_ts = (rx_ts << 8) + ts_buf[i]; 
                    
                    my_final_msg.final.responseRx_ts[anchor_id] = rx_ts;
                    rxResponseMask |= (1 << anchor_id); 
                    
                    if (anchor_id == 0) latest_slot_corr = anchor_reply->resp.slotCorr_ms;
                    
                    if (anchor_reply->resp.pre_dist_cm > 0) {
                        out_distances[anchor_id] = anchor_reply->resp.pre_dist_cm; 
                    }
                }
            }
        } else if (status_reg & (1UL << 17)) { 
            dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
            break; 
        } else if (status_reg & ((1UL << 12) | (1UL << 15) | (1UL << 21) | (1UL << 26))) { 
            dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        } else if (micros() - rx_window_start > 4500) { 
            break; 
        }
    }

    interrupts(); 

    bool is_synced = false;

    if (rxResponseMask > 0) {
        is_synced = true;
        
        // >>> THE TURNAROUND FIX <<<
        // Changed from 6500us to 5200us for faster anchor TX scheduling
        uint64_t target_final_ts = pollTx_ts + 332069120ULL;
        uint32_t delayed_tx_time = (uint32_t)(target_final_ts >> 8);
        
        my_final_msg.mac.seqNum = tx_poll_msg[2] + 1; 
        my_final_msg.final.rNum = tx_poll_msg[10]; 
        my_final_msg.final.pollTx_ts = pollTx_ts; 
        my_final_msg.final.rxResponseMask = rxResponseMask; 
        my_final_msg.final.finalTx_ts = (((uint64_t)(delayed_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY; 
        
        dwt_setdelayedtrxtime(delayed_tx_time); 
        dwt_writetxdata(sizeof(final_msg_t) - 2, (uint8_t *)&my_final_msg, 0); 
        dwt_writetxfctrl(sizeof(final_msg_t), 0, 0); 
        
        int tx_status = dwt_starttx(DWT_START_TX_DELAYED); 
        if(tx_status == DWT_SUCCESS) {
            uint32_t final_tx_start = micros();
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & (1UL<<7))) { 
                // >>> FAILSAFE BUMP <<<
                // Increased to 10000us so the ESP32 doesn't timeout while waiting for the 6500us TX slot
                if (micros() - final_tx_start > 10000) break; 
            }; 
            dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF); 
        }

        // TDMA Sleep Calculation
        uint32_t elapsed_ms = millis() - superframe_start_ms;
        int32_t sleep_time_ms = 100 - elapsed_ms;
        if (latest_slot_corr != 0) sleep_time_ms += latest_slot_corr;
        if (sleep_time_ms < 5 || sleep_time_ms > 100) sleep_time_ms = 100 - elapsed_ms; 
        
        if (sleep_time_ms > 0) delay(sleep_time_ms);

    } else {
        delay(5); 
    }

    return is_synced;
}