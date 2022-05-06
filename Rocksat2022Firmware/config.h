#ifndef CONFIG_H
#define CONFIG_H

//#define DEBUG 1 // usb serial debug switch
#ifdef DEBUG
  //#define DEBUG_GPS 1 // print raw gga to serial
  //#define DEBUG_QUEUE 1 // print info on log queue operations
  //#define DEBUG_VERBOSE 1
  //#define DEBUG_BARO 1
  #define DEBUG_IRD 1
  //#define DEBUG_LOG 1
  #define DEBUG_PAR 1
  #define DEBUG_RAD 1
  //#define DEBUG_TPMS_TRANSFER
#endif

// uncomment to enable GPS
//#define USE_GPS

// uncomment to enable spectrometer
#define USE_SPECTROMETER
#ifdef USE_SPECTROMETER
#define NUM_SPEC_CHANNELS 288
#endif

// uncomment to enable error led reporting
#define USE_LEDS

#define SEND_PACKETS 0 // set 1 for mission
#define IRIDIUM_PACKET_PERIOD 60000 // milliseconds, send a packet every minute
#define CHECK_SIGNAL_PERIOD   5000 // milliseconds
#define DIAGNOSTICS false// Change this to see diagnostics
#define SBD_TX_SZ 340

// NUM_TC_CHANNELS + NUM_HF_CHANNELS should always be equal to the total number of MCP9600 chips (TOT_MCP_COUNT)
#define NUM_TC_CHANNELS       6 // deg celcius

// Servo positions for the linear actuators that control the parachute
#define ACT_POS_HOME          10
#define ACT_POS_ACT           200

// the logfilename to use in the format [A-Z]{3}[0-9]{2}.CSV
// see https://regex101.com/
#define LOGFILE_NAME              "tlm00.csv"

// log buffer size in bytes (how many to accumulate before a write)
#define LOGBUF_SIZE 32768 / 8

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


#endif
