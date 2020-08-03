/*

ale_hal.c
Biblioteca para abstração do hardware de um gateway LoRaWAN

*/
#include "ale_hal.h"

void wait_ms(unsigned long a) {
    struct timespec dly;
    struct timespec rem;
    dly.tv_sec = a / 1000;
    dly.tv_nsec = ((long)a % 1000) * 1000000;
    if((dly.tv_sec > 0) || ((dly.tv_sec == 0) && (dly.tv_nsec > 100000))) {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &dly, &rem);
    }
    return;
}


int lgw_board_setconf(struct lgw_conf_board_s conf){
    return LGW_HAL_SUCCESS;
}
int lgw_lbt_setconf(struct lgw_conf_lbt_s conf){
    return LGW_HAL_SUCCESS;
}
int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s conf){
    return LGW_HAL_SUCCESS;
}

int lgw_rxif_setconf(uint8_t if_chain, struct lgw_conf_rxif_s conf){
    return LGW_HAL_SUCCESS;
}
int lgw_txgain_setconf(struct lgw_tx_gain_lut_s *conf){
    return LGW_HAL_SUCCESS;
}
int lgw_start(void){
    return LGW_HAL_SUCCESS;
}
int lgw_stop(void){
    return LGW_HAL_SUCCESS;
}

int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data){
    // ALE receive the packets and return buffer packets count...
    //Packet structure:
    struct pacoteUDP {
        //header:2 dest:1 source:1 lenght:1 pktnum:1 payload:0-251 retry:1
        uint8_t header[6]; // header,dest,source,pktnum,retry 
        uint8_t size;
        uint8_t payload[251]; 
    } bufferALE;
    int len, n; 
    n = recvfrom(sockALE, &bufferALE, 258, MSG_WAITALL, ( struct sockaddr *) &cliaddrALE, &len); 
    if (n>0) {
        pkt_data->freq_hz = 0;              // uint32_t central frequency of the IF chain
        pkt_data->if_chain = 1;             //uint8_t   by which IF chain was packet received       
        pkt_data->status = STAT_CRC_OK;     //uint8_t   status of the received packet       
        pkt_data->count_us = 1;             //uint32_t  internal concentrator counter for timestamping, 1 microsecond resolution              
        pkt_data->rf_chain = 1;             //uint8_t   through which RF chain the packet was received       
        pkt_data->modulation = MOD_LORA;    //uint8_t   modulation used by the packet        
        pkt_data->bandwidth = BW_125KHZ;    //uint8_t   modulation bandwidth (LoRa only)        
        pkt_data->datarate = DR_LORA_SF12;  //uint32_t  RX datarate of the packet (SF for LoRa)        
        pkt_data->coderate = CR_LORA_4_5;   //uint8_t   error-correcting code of the packet (LoRa only)  
        pkt_data->rssi = -57;               //float     average packet RSSI in dB     
        pkt_data->snr = 7;                     //float     average packet SNR, in dB (LoRa only)         
        pkt_data->snr_min = 7;                //float     minimum packet SNR, in dB (LoRa only)       
        pkt_data->snr_max = 7;                //float     maximum packet SNR, in dB (LoRa only)       
        pkt_data->crc = 0;                  //uint16_t  CRC that was received in the payload    
        pkt_data->size = bufferALE.size;    //uint16_t  payload size in bytes       
        memcpy(&pkt_data->payload, &bufferALE.payload, bufferALE.size);     //uint8_t buffer containing the payload        

        return 1;
    } else {
        return 0;
    }
}

int lgw_send(struct lgw_pkt_tx_s pkt_data){
    return LGW_HAL_SUCCESS;
}

int lgw_status(uint8_t select, uint8_t *code){
    return LGW_HAL_SUCCESS;
}

int lgw_abort_tx(void){
    return LGW_HAL_SUCCESS;
}

int lgw_get_trigcnt(uint32_t* trig_cnt_us){
    return LGW_HAL_SUCCESS;
}

const char* lgw_version_info(void){
    return "Void";
}

uint32_t lgw_time_on_air(struct lgw_pkt_tx_s *packet){
    return 1;
}

int lgw_gps_enable(char* tty_path, char* gps_familly, speed_t target_brate, int* fd_ptr){
    return LGW_GPS_SUCCESS;
}

int lgw_gps_disable(int fd){
    return LGW_GPS_SUCCESS;
}

enum gps_msg lgw_parse_nmea(const char* serial_buff, int buff_size){
    return IGNORED;
}

enum gps_msg lgw_parse_ubx(const char* serial_buff, size_t buff_size, size_t *msg_size){
    return IGNORED;
}

int lgw_gps_get(struct timespec *utc, struct timespec *gps_time, struct coord_s *loc, struct coord_s *err){
    return LGW_GPS_SUCCESS;
}

int lgw_gps_sync(struct tref *ref, uint32_t count_us, struct timespec utc, struct timespec gps_time){
    return LGW_GPS_SUCCESS;
}

int lgw_cnt2utc(struct tref ref, uint32_t count_us, struct timespec* utc){
    return LGW_GPS_SUCCESS;
}

int lgw_utc2cnt(struct tref ref,struct timespec utc, uint32_t* count_us){
    return LGW_GPS_SUCCESS;
}

int lgw_cnt2gps(struct tref ref, uint32_t count_us, struct timespec* gps_time){
    return LGW_GPS_SUCCESS;
}

int lgw_gps2cnt(struct tref ref, struct timespec gps_time, uint32_t* count_us){
    return LGW_GPS_SUCCESS;
}
