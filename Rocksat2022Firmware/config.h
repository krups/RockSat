#ifndef CONFIG_H
#define CONFIG_H

#define USELEDS
#define ERR_LED_ONSTATE 1

#define IRIDIUM_PACKET_PERIOD 60000 // milliseconds, send a packet every minute
#define CHECK_SIGNAL_PERIOD   5000 // milliseconds
#define DIAGNOSTICS false// Change this to see diagnostics
#define SBD_TX_SZ 340

// NUM_TC_CHANNELS + NUM_HF_CHANNELS should always be equal to the total number of MCP9600 chips (TOT_MCP_COUNT)
#define NUM_TC_CHANNELS       6 // deg celcius



// TODO: improve logfile system, should be local to boards not here
#define NUM_LOG_FILES         1
#define LOGFILE0              "tlm00.csv"

#define LOGID_TMP             0
#define LOGID_PRS             1
#define LOGID_GGA             2
#define LOGID_RMC             3
#define LOGID_BAR             4
#define LOGID_ACC             5
#define LOGID_IMU             6

#endif
