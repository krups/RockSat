#ifndef PACKET_H
#define PACKET_H

// logging packet structure
#include "config.h"

#define PTYPE_GGA  1  // nmea::GgaData
#define PTYPE_RMC  2  // nmea::RmcData
#define PTYPE_ACC  3
#define PTYPE_IMU  4
#define PTYPE_TC   5
#define PTYPE_PRS  6
#define PTYPE_SPEC 7
#define PTYPE_BAR  22
#define PTYPE_PACKET 99 // compressed packet written to logfile

struct packet_t {
  unsigned long t;
  unsigned long size; // actual data in packet
  char data[SBD_TX_SZ];
};

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
  float data[NUM_TC_CHANNELS];
}; // NUM_TC_CHANNELS + 1 bytes

//type PTYPE_BAR
struct bar_t {
  unsigned long t;
  float prs;
  float alt;
  float tmp;
}; // 4 bytes

struct rmc_t {
  unsigned long t; // microprocessor time in ms
  int time[4]; // hh:mm:ss:us GPS time
  float lat;
  float lon;
  float speed;
  float course;
};

struct gga_t {
  unsigned long t;
  int time[4];
  float lat;
  float lon;
  float hdop;
  float alt;
};

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
