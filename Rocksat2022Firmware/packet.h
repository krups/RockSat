#ifndef PACKET_H
#define PACKET_H

// logging packet structure
#include "config.h"

#define PTYPE_GGA  (1 << 0)  // nmea::GgaData
#define PTYPE_RMC  (1 << 1)  // nmea::RmcData
#define PTYPE_ACC  (1 << 2)
#define PTYPE_IMU  (1 << 3)
#define PTYPE_TC   (1 << 4)
#define PTYPE_PRS  (1 << 5)
#define PTYPE_SPEC (1 << 6)



// type PTYPE_ACC
struct acc_t {
  unsigned long t;
  float data[3];
}; // 4 bytes

// type PTYPE_IMU
struct imu_t {
  unsigned long t;
  float data[6];
}; // 7 bytes

// type PTYPE_TMP
struct tc_t {
  unsigned long t;
  uint16_t data[NUM_TC_CHANNELS];
}; // NUM_TC_CHANNELS + 1 bytes

//type PTYPE_BAR
struct bar_t {
  unsigned long t;
  float prs;
  float alt;
  float tmp;
}; // 4 bytes

// type PTYPE_TELEM
#ifdef USE_GPS
struct tlm_t {
  unsigned long t; // system time when packet was sent in # of scheduler ticks (ms)
  float lat;     // gps latitude
  float lon;     // gps longitude
  float vel;     // gps velocity
  float alt_gps; // gps altitude
  float alt_bar; // barometer altitude
  float barp;    // capsule internal barometric pressure
  float tmp;     // capsule internal temperature
  float bat;     // battery voltage
  int   irsig;   // iridium signal strength
  bool  pardep;  // parachute deployed yes/no
  tc_t tc;      // thermocouple data
  prs_t prs;     // external pressure sensors
};
#else
struct tlm_t {
  unsigned long t; // system time when packet was sent in # of scheduler ticks (ms)
  float tmp;     // capsule internal temperature
  float bat;     // battery voltage
  int   irsig;   // iridium signal strength
  bool  pardep;  // parachute deployed yes/no
  tc_t tc;      // thermocouple data
};
#endif

#ifdef USE_SPECTROMETER
struct spec_t {
  unsigned long t;
  float ch1;
  float ch2;
};
#endif

// not a packet type, used in the groundstation firmware to hold the extra radio receive info
struct rxtlm_t {
  tlm_t tlm;
  int rssi;
  int snr;
};

#endif
