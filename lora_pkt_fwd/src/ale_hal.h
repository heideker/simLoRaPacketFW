/*

ale_hal.c
Biblioteca para abstração do hardware de um gateway LoRaWAN





*/

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <time.h>       /* time library */
#include <termios.h>    /* speed_t */
#include <unistd.h>     /* ssize_t */
#include <string.h>

#define _GNU_SOURCE


//#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define IS_LORA_BW(bw)          ((bw == BW_125KHZ) || (bw == BW_250KHZ) || (bw == BW_500KHZ))
#define IS_LORA_STD_DR(dr)      ((dr == DR_LORA_SF7) || (dr == DR_LORA_SF8) || (dr == DR_LORA_SF9) || (dr == DR_LORA_SF10) || (dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))
#define IS_LORA_MULTI_DR(dr)    ((dr & ~DR_LORA_MULTI) == 0) /* ones outside of DR_LORA_MULTI bitmask -> not a combination of LoRa datarates */
#define IS_LORA_CR(cr)          ((cr == CR_LORA_4_5) || (cr == CR_LORA_4_6) || (cr == CR_LORA_4_7) || (cr == CR_LORA_4_8))

#define IS_FSK_BW(bw)           ((bw >= 1) && (bw <= 7))
#define IS_FSK_DR(dr)           ((dr >= DR_FSK_MIN) && (dr <= DR_FSK_MAX))

#define IS_TX_MODE(mode)        ((mode == IMMEDIATE) || (mode == TIMESTAMPED) || (mode == ON_GPS))

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* return status code */
#define LGW_HAL_SUCCESS     0
#define LGW_HAL_ERROR       -1
#define LGW_LBT_ISSUE       1

/* radio-specific parameters */
#define LGW_XTAL_FREQU      32000000            /* frequency of the RF reference oscillator */
#define LGW_RF_CHAIN_NB     2                   /* number of RF chains */
#define LGW_RF_RX_BANDWIDTH {1000000, 1000000}  /* bandwidth of the radios */

/* type of if_chain + modem */
#define IF_UNDEFINED        0
#define IF_LORA_STD         0x10    /* if + standard single-SF LoRa modem */
#define IF_LORA_MULTI       0x11    /* if + LoRa receiver with multi-SF capability */
#define IF_FSK_STD          0x20    /* if + standard FSK modem */

/* concentrator chipset-specific parameters */
/* to use array parameters, declare a local const and use 'if_chain' as index */
#define LGW_IF_CHAIN_NB     10    /* number of IF+modem RX chains */
#define LGW_PKT_FIFO_SIZE   16    /* depth of the RX packet FIFO */
#define LGW_DATABUFF_SIZE   1024    /* size in bytes of the RX data buffer (contains payload & metadata) */
#define LGW_REF_BW          125000    /* typical bandwidth of data channel */
#define LGW_MULTI_NB        8    /* number of LoRa 'multi SF' chains */
#define LGW_IFMODEM_CONFIG {\
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_MULTI, \
        IF_LORA_STD, \
        IF_FSK_STD } /* configuration of available IF chains and modems on the hardware */

/* values available for the 'modulation' parameters */
/* NOTE: arbitrary values */
#define MOD_UNDEFINED   0
#define MOD_LORA        0x10
#define MOD_FSK         0x20

/* values available for the 'bandwidth' parameters (LoRa & FSK) */
/* NOTE: directly encode FSK RX bandwidth, do not change */
#define BW_UNDEFINED    0
#define BW_500KHZ       0x01
#define BW_250KHZ       0x02
#define BW_125KHZ       0x03
#define BW_62K5HZ       0x04
#define BW_31K2HZ       0x05
#define BW_15K6HZ       0x06
#define BW_7K8HZ        0x07

/* values available for the 'datarate' parameters */
/* NOTE: LoRa values used directly to code SF bitmask in 'multi' modem, do not change */
#define DR_UNDEFINED    0
#define DR_LORA_SF7     0x02
#define DR_LORA_SF8     0x04
#define DR_LORA_SF9     0x08
#define DR_LORA_SF10    0x10
#define DR_LORA_SF11    0x20
#define DR_LORA_SF12    0x40
#define DR_LORA_MULTI   0x7E
/* NOTE: for FSK directly use baudrate between 500 bauds and 250 kbauds */
#define DR_FSK_MIN      500
#define DR_FSK_MAX      250000

/* values available for the 'coderate' parameters (LoRa only) */
/* NOTE: arbitrary values */
#define CR_UNDEFINED    0
#define CR_LORA_4_5     0x01
#define CR_LORA_4_6     0x02
#define CR_LORA_4_7     0x03
#define CR_LORA_4_8     0x04

/* values available for the 'status' parameter */
/* NOTE: values according to hardware specification */
#define STAT_UNDEFINED  0x00
#define STAT_NO_CRC     0x01
#define STAT_CRC_BAD    0x11
#define STAT_CRC_OK     0x10

/* values available for the 'tx_mode' parameter */
#define IMMEDIATE       0
#define TIMESTAMPED     1
#define ON_GPS          2
//#define ON_EVENT      3
//#define GPS_DELAYED   4
//#define EVENT_DELAYED 5

/* values available for 'select' in the status function */
#define TX_STATUS       1
#define RX_STATUS       2

/* status code for TX_STATUS */
/* NOTE: arbitrary values */
#define TX_STATUS_UNKNOWN   0
#define TX_OFF              1    /* TX modem disabled, it will ignore commands */
#define TX_FREE             2    /* TX modem is free, ready to receive a command */
#define TX_SCHEDULED        3    /* TX modem is loaded, ready to send the packet after an event and/or delay */
#define TX_EMITTING         4    /* TX modem is emitting */

/* status code for RX_STATUS */
/* NOTE: arbitrary values */
#define RX_STATUS_UNKNOWN   0
#define RX_OFF              1    /* RX modem is disabled, it will ignore commands  */
#define RX_ON               2    /* RX modem is receiving */
#define RX_SUSPENDED        3    /* RX is suspended while a TX is ongoing */

/* Maximum size of Tx gain LUT */
#define TX_GAIN_LUT_SIZE_MAX 16

/* LBT constants */
#define LBT_CHANNEL_FREQ_NB 8 /* Number of LBT channels */

#define LGW_GPS_SUCCESS 0
#define LGW_GPS_ERROR   -1

#define LGW_GPS_MIN_MSG_SIZE      (8)
#define LGW_GPS_UBX_SYNC_CHAR     (0xB5)
#define LGW_GPS_NMEA_SYNC_CHAR    (0x24)


/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

/**
@enum lgw_radio_type_e
@brief Radio types that can be found on the LoRa Gateway
*/
enum lgw_radio_type_e {
    LGW_RADIO_TYPE_NONE,
    LGW_RADIO_TYPE_SX1255,
    LGW_RADIO_TYPE_SX1257,
    LGW_RADIO_TYPE_SX1272,
    LGW_RADIO_TYPE_SX1276
};

/**
@struct lgw_conf_board_s
@brief Configuration structure for board specificities
*/
struct lgw_conf_board_s {
    bool    lorawan_public; /*!> Enable ONLY for *public* networks using the LoRa MAC protocol */
    uint8_t clksrc;         /*!> Index of RF chain which provides clock to concentrator */
};

/**
@struct lgw_conf_lbt_chan_s
@brief Configuration structure for LBT channels
*/
struct lgw_conf_lbt_chan_s {
    uint32_t freq_hz;
    uint16_t scan_time_us;
};

/**
@struct lgw_conf_lbt_s
@brief Configuration structure for LBT specificities
*/
struct lgw_conf_lbt_s {
    bool                        enable;             /*!> enable or disable LBT */
    int8_t                      rssi_target;        /*!> RSSI threshold to detect if channel is busy or not (dBm) */
    uint8_t                     nb_channel;         /*!> number of LBT channels */
    struct lgw_conf_lbt_chan_s  channels[LBT_CHANNEL_FREQ_NB];
    int8_t                      rssi_offset;        /*!> RSSI offset to be applied to SX127x RSSI values */
};

/**
@struct lgw_conf_rxrf_s
@brief Configuration structure for a RF chain
*/
struct lgw_conf_rxrf_s {
    bool                    enable;         /*!> enable or disable that RF chain */
    uint32_t                freq_hz;        /*!> center frequency of the radio in Hz */
    float                   rssi_offset;    /*!> Board-specific RSSI correction factor */
    enum lgw_radio_type_e   type;           /*!> Radio type for that RF chain (SX1255, SX1257....) */
    bool                    tx_enable;      /*!> enable or disable TX on that RF chain */
    uint32_t                tx_notch_freq;  /*!> TX notch filter frequency [126KHz..250KHz] */
};

/**
@struct lgw_conf_rxif_s
@brief Configuration structure for an IF chain
*/
struct lgw_conf_rxif_s {
    bool        enable;         /*!> enable or disable that IF chain */
    uint8_t     rf_chain;       /*!> to which RF chain is that IF chain associated */
    int32_t     freq_hz;        /*!> center frequ of the IF chain, relative to RF chain frequency */
    uint8_t     bandwidth;      /*!> RX bandwidth, 0 for default */
    uint32_t    datarate;       /*!> RX datarate, 0 for default */
    uint8_t     sync_word_size; /*!> size of FSK sync word (number of bytes, 0 for default) */
    uint64_t    sync_word;      /*!> FSK sync word (ALIGN RIGHT, eg. 0xC194C1) */
};

/**
@struct lgw_pkt_rx_s
@brief Structure containing the metadata of a packet that was received and a pointer to the payload
*/
struct lgw_pkt_rx_s {
    uint32_t    freq_hz;        /*!> central frequency of the IF chain */
    uint8_t     if_chain;       /*!> by which IF chain was packet received */
    uint8_t     status;         /*!> status of the received packet */
    uint32_t    count_us;       /*!> internal concentrator counter for timestamping, 1 microsecond resolution */
    uint8_t     rf_chain;       /*!> through which RF chain the packet was received */
    uint8_t     modulation;     /*!> modulation used by the packet */
    uint8_t     bandwidth;      /*!> modulation bandwidth (LoRa only) */
    uint32_t    datarate;       /*!> RX datarate of the packet (SF for LoRa) */
    uint8_t     coderate;       /*!> error-correcting code of the packet (LoRa only) */
    float       rssi;           /*!> average packet RSSI in dB */
    float       snr;            /*!> average packet SNR, in dB (LoRa only) */
    float       snr_min;        /*!> minimum packet SNR, in dB (LoRa only) */
    float       snr_max;        /*!> maximum packet SNR, in dB (LoRa only) */
    uint16_t    crc;            /*!> CRC that was received in the payload */
    uint16_t    size;           /*!> payload size in bytes */
    uint8_t     payload[256];   /*!> buffer containing the payload */
};

/**
@struct lgw_pkt_tx_s
@brief Structure containing the configuration of a packet to send and a pointer to the payload
*/
struct lgw_pkt_tx_s {
    uint32_t    freq_hz;        /*!> center frequency of TX */
    uint8_t     tx_mode;        /*!> select on what event/time the TX is triggered */
    uint32_t    count_us;       /*!> timestamp or delay in microseconds for TX trigger */
    uint8_t     rf_chain;       /*!> through which RF chain will the packet be sent */
    int8_t      rf_power;       /*!> TX power, in dBm */
    uint8_t     modulation;     /*!> modulation to use for the packet */
    uint8_t     bandwidth;      /*!> modulation bandwidth (LoRa only) */
    uint32_t    datarate;       /*!> TX datarate (baudrate for FSK, SF for LoRa) */
    uint8_t     coderate;       /*!> error-correcting code of the packet (LoRa only) */
    bool        invert_pol;     /*!> invert signal polarity, for orthogonal downlinks (LoRa only) */
    uint8_t     f_dev;          /*!> frequency deviation, in kHz (FSK only) */
    uint16_t    preamble;       /*!> set the preamble length, 0 for default */
    bool        no_crc;         /*!> if true, do not send a CRC in the packet */
    bool        no_header;      /*!> if true, enable implicit header mode (LoRa), fixed length (FSK) */
    uint16_t    size;           /*!> payload size in bytes */
    uint8_t     payload[256];   /*!> buffer containing the payload */
};

/**
@struct lgw_tx_gain_s
@brief Structure containing all gains of Tx chain
*/
struct lgw_tx_gain_s {
    uint8_t dig_gain;   /*!> 2 bits, control of the digital gain of SX1301 */
    uint8_t pa_gain;    /*!> 2 bits, control of the external PA (SX1301 I/O) */
    uint8_t dac_gain;   /*!> 2 bits, control of the radio DAC */
    uint8_t mix_gain;   /*!> 4 bits, control of the radio mixer */
    int8_t  rf_power;   /*!> measured TX power at the board connector, in dBm */
};

/**
@struct lgw_tx_gain_lut_s
@brief Structure defining the Tx gain LUT
*/
struct lgw_tx_gain_lut_s {
    struct lgw_tx_gain_s    lut[TX_GAIN_LUT_SIZE_MAX];  /*!> Array of Tx gain struct */
    uint8_t                 size;                       /*!> Number of LUT indexes */
};


struct tref {
    time_t          systime;    /*!> system time when solution was calculated */
    uint32_t        count_us;   /*!> reference concentrator internal timestamp */
    struct timespec utc;        /*!> reference UTC time (from GPS/NMEA) */
    struct timespec gps;        /*!> reference GPS time (since 01.Jan.1980) */
    double          xtal_err;   /*!> raw clock error (eg. <1 'slow' XTAL) */
};

/**
@struct coord_s
@brief Geodesic coordinates
*/
struct coord_s {
    double  lat;    /*!> latitude [-90,90] (North +, South -) */
    double  lon;    /*!> longitude [-180,180] (East +, West -)*/
    short   alt;    /*!> altitude in meters (WGS 84 geoid ref.) */
};

/**
@enum gps_msg
@brief Type of GPS (and other GNSS) sentences
*/
enum gps_msg {
    UNKNOWN,         /*!> neutral value */
    IGNORED,         /*!> frame was not parsed by the system */
    INVALID,         /*!> system try to parse frame but failed */
    INCOMPLETE,      /*!> frame parsed was missing bytes */
    /* NMEA messages of interest */
    NMEA_RMC,        /*!> Recommended Minimum data (time + date) */
    NMEA_GGA,        /*!> Global positioning system fix data (pos + alt) */
    NMEA_GNS,        /*!> GNSS fix data (pos + alt, sat number) */
    NMEA_ZDA,        /*!> Time and Date */
    /* NMEA message useful for time reference quality assessment */
    NMEA_GBS,        /*!> GNSS Satellite Fault Detection */
    NMEA_GST,        /*!> GNSS Pseudo Range Error Statistics */
    NMEA_GSA,        /*!> GNSS DOP and Active Satellites (sat number) */
    NMEA_GSV,        /*!> GNSS Satellites in View (sat SNR) */
    /* Misc. NMEA messages */
    NMEA_GLL,        /*!> Latitude and longitude, with time fix and status */
    NMEA_TXT,        /*!> Text Transmission */
    NMEA_VTG,        /*!> Course over ground and Ground speed */
    /* uBlox proprietary NMEA messages of interest */
    UBX_NAV_TIMEGPS, /*!> GPS Time Solution */
    UBX_NAV_TIMEUTC  /*!> UTC Time Solution */
};


void wait_ms(unsigned long a);

int lgw_board_setconf(struct lgw_conf_board_s conf);

int lgw_lbt_setconf(struct lgw_conf_lbt_s conf);

int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s conf);

int lgw_rxif_setconf(uint8_t if_chain, struct lgw_conf_rxif_s conf);

int lgw_txgain_setconf(struct lgw_tx_gain_lut_s *conf);

int lgw_start(void);

int lgw_stop(void);

int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data);

int lgw_send(struct lgw_pkt_tx_s pkt_data);

int lgw_status(uint8_t select, uint8_t *code);

int lgw_abort_tx(void);

int lgw_get_trigcnt(uint32_t* trig_cnt_us);

const char* lgw_version_info(void);

uint32_t lgw_time_on_air(struct lgw_pkt_tx_s *packet);

int lgw_gps_enable(char* tty_path, char* gps_familly, speed_t target_brate, int* fd_ptr);

int lgw_gps_disable(int fd);

enum gps_msg lgw_parse_nmea(const char* serial_buff, int buff_size);

enum gps_msg lgw_parse_ubx(const char* serial_buff, size_t buff_size, size_t *msg_size);

int lgw_gps_get(struct timespec *utc, struct timespec *gps_time, struct coord_s *loc, struct coord_s *err);

int lgw_gps_sync(struct tref *ref, uint32_t count_us, struct timespec utc, struct timespec gps_time);

int lgw_cnt2utc(struct tref ref, uint32_t count_us, struct timespec* utc);

int lgw_utc2cnt(struct tref ref,struct timespec utc, uint32_t* count_us);

int lgw_cnt2gps(struct tref ref, uint32_t count_us, struct timespec* gps_time);

int lgw_gps2cnt(struct tref ref, struct timespec gps_time, uint32_t* count_us);



