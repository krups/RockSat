#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG 1 // usb serial debug switch
#ifdef DEBUG
  //#define DEBUG_GPS 1 // print raw gga to serial
  //#define DEBUG_QUEUE 1 // print info on log queue operations
  //#define DEBUG_VERBOSE 1
  //#define DEBUG_BARO 1
  //#define DEBUG_IRD 1
  #define DEBUG_LOG 1
  #define DEBUG_PAR 1
  //#define DEBUG_RAD 1
  //#define DEBUG_SPEC 1
  //#define DEBUG_IMU 1
  //#define DEBUG_TC
  //#define DEBUG_TPMS_TRANSFER
//#define DEBUG_MCP_STARTUP
#endif

#define TIME_SCALE 100 // ticks per time unit

#define UNIT_SCALE 10 // multiplier applied

#define PAR_DEPLOY_DELAY_MS 300000

// uncomment to enable GPS
// gps sample period is default 1Hz
#define USE_GPS


#define TC_SAMPLE_PERIOD_MS   1000
#define SPEC_SAMPLE_PERIOD_MS 1000
#define IMU_SAMPLE_PERIOD_MS  1000

// uncomment to enable spectrometer
#define USE_SPECTROMETER
#ifdef USE_SPECTROMETER
#define NUM_SPEC_CHANNELS 288
#endif

// uncomment to enable error led reporting
#define USE_LEDS

#define SEND_PACKETS 0 // set 1 for mission
#define IRIDIUM_PACKET_PERIOD 30000 // milliseconds, send a packet every minute
#define CHECK_SIGNAL_PERIOD   5000 // milliseconds
#define DIAGNOSTICS false// Change this to see diagnostics
#define SBD_TX_SZ 340

// how often to build a packet (in milliseconds)
#define PACKET_BUILD_PERIOD 20000

// calibration for barometer altitude reading
#define BAR_SEA_PRESSURE 1013.26

// NUM_TC_CHANNELS + NUM_HF_CHANNELS should always be equal to the total number of MCP9600 chips (TOT_MCP_COUNT)
#define NUM_TC_CHANNELS       6 // deg celcius

// the logfilename to use in the format [A-Z]{3}[0-9]{2}.CSV
// see https://regex101.com/
#define LOGFILE_NAME              "LG000.DAT"
#define LOGFILE_NAME_LENGTH 10 // including null terminator

#define LOGBUF_HEADER_SIZE 2048

// log buffer size in bytes (how many to accumulate before a write)
#define LOGBUF_BLOCK_SIZE         2048              // 32768 / 32
#define LOGBUF_FULL_SIZE    LOGBUF_BLOCK_SIZE - 512 // compressed iridium packet gauranteed to fit

#define BLZ_HASH_BITS 12

// for debug radio
#define TLM_SEND_PERIOD   5000 // in scheduler ticks (should be 1ms)
#define RX_TIMEOUT_PERIOD 500  // also in scheduler ticks
#define RBUF_SIZE 260
#define SBUF_SIZE 240

#define LOGID_TMP             0
#define LOGID_PRS             1
#define LOGID_GGA             2
#define LOGID_RMC             3
#define LOGID_BAR             4
#define LOGID_ACC             5
#define LOGID_IMU             6

#define ERR_BOOT              0
#define ERR_2                 1
#define ERR_3                 2
#define ERR_4                 3
#define ERR_SD_BUSY           -1
#define OK                    123

#endif
