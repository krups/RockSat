/*
* KREPE ISS Mission
* Dec 2020
*
* Main mission config params
* 
*/


#ifndef CONFIG_H
#define CONFIG_H

#include "pins.h"

// the different mission scenarios
#define MISSION_REAL_SHUTTLE 1
#define MISSION_REAL_JSC     2
#define MISSION_TEST_SHUTTLE 3
#define MISSION_TEST_JSC     4

// mission type
#define MISSION_TYPE         MISSION_TEST_SHUTTLE

// auto send packets 
#define CONFIG_AUTO_BUILD_PACKETS 1

// activation source config
#define CONFIG_USE_ACT_TC    0
#define CONFIG_USE_ACT_CAP   1

//////////////////////////////////////////
// real mission for the two capsules with shuttle tile TPS
#if (MISSION_TYPE == MISSION_REAL_SHUTTLE)
  #define USBSERIAL_DEBUG      0
  #define ISM_DEBUG            0
  #define TC_COUNT             4
  #define TC_THRESHOLD         10.0 // degrees celcius
  #define TC_CONSENSUS         3   // number of TCs that need to exceed threshold
  
  // TODO: refer to TcInterface for which physical connections these are if all
  //       8 possible thermocouple connections are not specified
  #define TC_TYPE_STRING       "KKKK" 
#endif

// real mission for the JSC TPS
//////////////////////////////////////////
#if (MISSION_TYPE == MISSION_REAL_JSC)
  #define USBSERIAL_DEBUG      0
  #define ISM_DEBUG            0
  #define TC_COUNT             4
  #define TC_THRESHOLD         10.0 // degrees celcius
  #define TC_CONSENSUS         3   // number of TCs that need to exceed threshold
  
  #define TC_TYPE_STRING       "RKKRK" 
#endif

// ground trial setup for shuttle tile TPS capsules
//////////////////////////////////////////
#if (MISSION_TYPE == MISSION_TEST_SHUTTLE)
  #define USBSERIAL_DEBUG      0
  #define ISM_DEBUG            0
  #define TC_COUNT             4
  #define TC_THRESHOLD         10.0 // degrees celcius
  #define TC_CONSENSUS         3   // number of TCs that need to exceed threshold
  
  #define TC_TYPE_STRING       "KKKK" 
#endif

// ground trial setup for the capsule with JSC TPS 
//////////////////////////////////////////
#if (MISSION_TYPE == MISSION_TEST_JSC)
  #define USBSERIAL_DEBUG      0
  #define ISM_DEBUG            0
  #define TC_COUNT             4
  #define TC_THRESHOLD         10.0 // degrees celcius
  #define TC_CONSENSUS         3   // number of TCs that need to exceed threshold
  
  #define TC_TYPE_STRING       "RKKRK" 
#endif


/************************************************************************************************
* LOG CONFIG
*/
// log source/file handle identifiers
#define LOGID_TELEM             0
#define LOGID_TC                1
#define LOGID_ACC               2
#define LOGID_IMU               3
#define LOGID_SYS               4

// a number is appended to this tag each run so the filename would be "telem01.dat" 
#define LOGNAME_TELEM           "telem"
#define LOGNAME_TC              "tc"
#define LOGNAME_ACC             "acc"
#define LOGNAME_IMU             "imu"
#define LOGNAME_SYS             "sys"

// default log intervals in milliseconds
#define LOGINT_TELEM            1000
#define LOGINT_TC               1000
#define LOGINT_ACC              100
#define LOGINT_IMU              100

/**********************************************************************************************
* IMU AND ACC SAMPLE RATE CONFIG
*/
#define IMU_SAMPLE_PERIOD       500
//#define IMU_BUFFER_SAMPLES      

#define ACC_SAMPLE_PERIOD       500
//#define ACC_BUFFER_SAMPLES


/************************************************************************************************
* battery voltage calibration
*/
//#define VBAT_DIV                0.50761421319 // for board "X",   R6=97k   to Vin, R7=100k to GND
#define VBAT_DIV                0.542        // for board "1",   R6=96.8k to Vin, R7=99.3k to GND
//#define VBAT_DIV                0.5046        // for board "2",   R6=97.3k to Vin, R7=99.1k to GND

/************************************************************************************************
* queue size control (applies to both ISM debug and SD log. 
*/
#define MAX_TCQ_SIZE            5
#define MAX_ACCQ_SIZE           10
#define MAX_IMUQ_SIZE           10
#define MAX_TELEMQ_SIZE         10

/************************************************************************************************
* SLEEP CONFIG
*/
#define CONFIG_AUTOMATIC_SLEEP  1
#define SLEEP_TIME_AWAKE        (int)10000 //when the sleep thread wakes up, stay awake for this long
#define SLEEP_DURATION_MINUTES  5760 // 1440 min = 24 hrs

/************************************************************************************************
* LED CONFIG
*/
#define CONFIG_USE_LEDS         0

#endif
