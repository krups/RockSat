/* Thermocouple thread measurement test

   Like TcTest but with measurement in a timesliced task using TeensyThreads

   see the following link regarding sending structs
   https://arduino.stackexchange.com/questions/45066/radiohead-library-nrf24-sending-and-receiving-struct-data-problem
*/

// radio logger node class needs this defined
#define NODE_ADDRESS CAPSULE_ADDRESS

#include "src/packets.h"
#include "src/SDLogger.h"
#include "src/brieflz.h"
#include "src/TcInterface.h"
#include "src/RadioLogger.h"
#include <IridiumSBD.h>
#include <TeensyThreads.h>
#include <Snooze.h>
#include <ICM_20948.h>

#include <queue>

// IMU
ICM_20948_I2C myICM;

// Load drivers for low power snooze library
// don't load snoozeUSBSerial helper class unless we have usb debugging enabbled
//SnoozeUSBSerial usb;
SnoozeDigital   snoozeDigital;
SnoozeSPI       snoozeSPI;
//SnoozeTimer     timer;
#if USBSERIAL_DEBUG
SnoozeUSBSerial snoozeSerial;
SnoozeBlock     config_teensy35(snoozeDigital, snoozeSPI, snoozeSerial);
#else
SnoozeBlock     config_teensy35(snoozeDigital, snoozeSPI);
#endif

Threads::Mutex ns_lock; // need sleep lock, access to needSleep var
volatile bool needSleep = false; // is the sleep thread requesting that other threads sleep

Threads::Mutex sr_radio_lock;
volatile bool sr_radio = false;  // is the radio thread ready for sleep

Threads::Mutex sr_tc_lock;
volatile bool sr_tc = false;     // is the tc thread ready to sleep

Threads::Mutex sr_acc_lock;
volatile bool sr_acc = false;    // acccel is ready to sleep

Threads::Mutex sr_imu_lock;
volatile bool sr_imu = false;  // imu is ready to sleep

Threads::Mutex sr_sd_lock;    // is the sd logger ready to sleep
volatile bool sr_sd = false;  // ready to sleep means  all packets written to logfiles

Threads::Mutex sr_ir_lock;
volatile bool sr_ir = false;  // iridium thread is ready to sleep

Threads::Mutex sr_telem_lock;
volatile bool sr_telem = false; // is the telem thread ready to sleep

Threads::Mutex drp_lock;
volatile bool debugRadioPresent = false; // is the debug radio powered?

// for checking if we have a syslog file yet or not
Threads::Mutex syslog_lock;
volatile bool sysLogCreated = false;

// are there packets in the command packet queue
Threads::Mutex cmdPacket_lock;
volatile int cmdPacketCount = 0;
volatile CommandPacket* cmdPackets[MAX_CMDQ_SIZE];

// have we detected activation
volatile bool activation = false;
Threads::Mutex act_lock;

// activation condition detected on thermocouples
volatile bool tcAct = false;
Threads::Mutex tcAct_lock;


// Thermocouple to digital converter interface with mux control
TcInterface tc(TC_TYPE_STRING);


// sd log interface, supports creating and writing to files
SDLogger sdlog;

////////////////////////
// SD LOG PACKET QUEUES
// packet queues to be taken from for SD logs and added to by
// data generating threads
volatile TcPacket*    tcdataq[MAX_TCQ_SIZE];
volatile AccPacket*   accdataq[MAX_ACCQ_SIZE];
volatile IMUPacket*   imudataq[MAX_IMUQ_SIZE];
volatile TelemPacket* telemdataq[MAX_TELEMQ_SIZE];
// the number of items in each packet queue and mutexes to protect them
volatile int tcq_count    = 0;
volatile int accq_count   = 0;
volatile int imuq_count   = 0;
volatile int telemq_count = 0;
// read and write indexes to use the data arrays as queues with a ring buffer
volatile int tcq_read     = 0;
volatile int tcq_write    = 0;
volatile int accq_read    = 0;
volatile int accq_write   = 0;
volatile int imuq_read    = 0;
volatile int imuq_write   = 0;
volatile int telemq_read  = 0;
volatile int telemq_write = 0;
// MUTEXES FOR ABOVE QUEUE VARS
Threads::Mutex tcq_lock;
Threads::Mutex accq_lock;
Threads::Mutex imuq_lock;
Threads::Mutex telemq_lock;
// END SD LOG PACKET QUEUES
///////////////////////////////


//////////////////////////////
// ISM DEBUG LOG PACKET QUEUES
volatile TcPacket*    logtcdataq[MAX_TCQ_SIZE];
volatile AccPacket*   logaccdataq[MAX_ACCQ_SIZE];
volatile IMUPacket*   logimudataq[MAX_IMUQ_SIZE];
volatile TelemPacket* logtelemdataq[MAX_TELEMQ_SIZE];
// the number of items in each packet queue and mutexes to protect them
volatile int logtcq_count    = 0;
volatile int logaccq_count   = 0;
volatile int logimuq_count   = 0;
volatile int logtelemq_count = 0;
// read and write indexes to use the data arrays as queues with a ring buffer
volatile int logtcq_read     = 0;
volatile int logtcq_write    = 0;
volatile int logaccq_read    = 0;
volatile int logaccq_write   = 0;
volatile int loglogimuq_read = 0;
volatile int logimuq_write   = 0;
volatile int logtelemq_read  = 0;
volatile int logtelemq_write = 0;
// MUTEXES FOR ABOVE QUEUE VARS
Threads::Mutex logtcq_lock;
Threads::Mutex logaccq_lock;
Threads::Mutex logimuq_lock;
Threads::Mutex logtelemq_lock;
// END ISM DEBUG PACKET QUEUES
///////////////////////////////


// ism debug radio interface
static RadioLogger logNode;



// data ready flags for each data generating threads
volatile bool dr_tc    = false;
volatile bool dr_telem = false;
volatile bool dr_imu   = false;
volatile bool dr_acc   = false;
//volatile bool icm_int  = false; // used by the imu thread with the icm-20948
// data ready flag mutex
Threads::Mutex dr_tc_lock;
Threads::Mutex dr_telem_lock;
Threads::Mutex dr_imu_lock;
Threads::Mutex dr_acc_lock;


// Mutex for hardware resources
Threads::Mutex sd_lock;
Threads::Mutex millis_lock;
Threads::Mutex spi_lock;
Threads::Mutex ser_lock;
Threads::Mutex i2c_lock;


// compression thread vars
volatile bool buildPacket = false;
Threads::Mutex buildPacket_lock;



// enable flags for ism debug logging from individual threads
volatile bool logen_tc = true;
volatile bool logen_telem = true;


// activation detected on capacitive sensor
volatile bool capAct = false;
Threads::Mutex capAct_lock;

volatile bool capReading = false;
Threads::Mutex capReading_lock;

// IRIDIUM VARS
#define IRIDIUM_SERIAL Serial4
#define DIAGNOSTICS false// Change this to see diagnostics
#define SBD_TX_SZ 340
IridiumSBD modem(Serial4);
int signalQuality = 0;
Threads::Mutex sq_lock;
int irerr;
//Threads::Mutex ircmd_lock;    // for giving commands to the iridium thread
//volatile int ircmd = 0;
uint8_t signalBrightness[6] = {0, 2, 10, 50, 100, 250};

Threads::Mutex irready_lock;
volatile bool irready = false;

Threads::Mutex irbuf_lock;
volatile bool irbuf_ready = false;
volatile uint8_t irbuf[SBD_TX_SZ];
volatile int irbuf_len = 0;

Threads::Mutex irpc_lock;
volatile int irpc = 0;

/************************
 * THread IDs
 */
int tid_sd       = -1,
    tid_telem    = -1,
    tid_tc       = -1,
    tid_acc      = -1,
    tid_iridium  = -1,
    tid_imu      = -1,
    tid_command  = -1,
    tid_compress = -1,
    tid_cap      = -1,
    tid_sleep    = -1,
    tid_radio    = -1;
// end thread IDs

/*************************************
* Parachute Deployment Thresholds
*/

int parachute_time = 373000; // 550 seconds
float accelerometer_threshold = 1E-2; // | accel | < 0.01 m/s^2
int total_necessary_accel_time = 5000;  // Make sure it stayed at "0" for 5 seconds

// end parachute thresholds


/**************************************************************************
   Thread safe helpers for setting getting and updating flags and printing
*/
// thread safe copy of src boolean to dest boolean.
// dont use do assign a direct value to dest, use safeAssign for that
void safeUpdate(bool *dest, bool *src, Threads::Mutex *m) {
  // wait forever in 1000 ms intervals to do the assignment
  while ( !m->lock(100) );
  *dest = *src;
  m->unlock();
}
// thread safe assignement of a direct truth val to dest
// pass this function a direct truth value
void safeAssign(bool *dest, bool src, Threads::Mutex *m) {
  // wait forever in 1000 ms intervals to do the assignment
  while ( !m->lock(100) );
  *dest = src;
  m->unlock();
}
// thread safe global read
bool safeRead(bool *src, Threads::Mutex *m) {
  bool ret = false;
  while ( !m->lock(100) );
  ret = *src;
  m->unlock();
  return ret;
}
// thread safe access to the current run clock
unsigned long safeMillis() {
  unsigned long m = 0;
  while ( !millis_lock.lock(5) );
  m = millis();
  millis_lock.unlock();
  return m;
}
void safePrint(String s) {
#if USBSERIAL_DEBUG
  while ( !ser_lock.lock(100) );
  Serial.print(s);
  ser_lock.unlock();
#endif
}
void safePrintln(String s) {
#if USBSERIAL_DEBUG
  while ( !ser_lock.lock(100) );
  Serial.println(s);
  ser_lock.unlock();
#endif
}
// end thread safe helpers
/**********************************************************************************/



/***********************************************************************************
   IMU thread helpers
*/
void imuSleep() {
  while ( !i2c_lock.lock(1000) );
  myICM.lowPower(true);
  myICM.sleep(true);
  i2c_lock.unlock();
}
void imuWake() {
  while ( !i2c_lock.lock(1000) );
  myICM.sleep(false);
  myICM.lowPower(false);
  i2c_lock.unlock();
}
bool imuInit() {
  bool initialized = false;
  while ( !initialized ) {

    // start ICM-20948
    if (USBSERIAL_DEBUG) safePrintln("IMU: trying to start ICM-20948");
    while ( !i2c_lock.lock(100) );
    myICM.begin( Wire, 0 ); // i2c coms, address bit=0

    if ( myICM.status != ICM_20948_Stat_Ok ) {
      i2c_lock.unlock();
      threads.delay(100);
    } else {
      i2c_lock.unlock();
      initialized = true;
    }

    // make sure we are not sleeping
    imuWake();

    // Set full scale range struct for both acc and gyr
    ICM_20948_fss_t myFSS;
    myFSS.a = gpm16; // set +/-16g sensitivity
    myFSS.g = dps2000; // set 2000 dps
    while ( !i2c_lock.lock(100) );
    myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );
    if ( myICM.status != ICM_20948_Stat_Ok) {
      //safePrintln("IMU: failed to set full scale range");
      initialized = false;
    }

    ICM_20948_smplrt_t mySmplrt;
    // this is 4.4 hz output data rate for the imu
    // calculate datarate with 1125 / (1+ODR) in Hz
    mySmplrt.a = 255;
    mySmplrt.g = 255;
    myICM.setSampleRate( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt );
    if ( myICM.status != ICM_20948_Stat_Ok) {
      //safePrintln("IMU:failed to set sample rate");
      initialized = false;
    }

    i2c_lock.unlock();
  }
  return initialized;
}
//void imuISR() {
//  icm_int = true;
//}
// imu thread
void imu_thread(int inc) {
  // IMU SETUP
  if ( imuInit() ) {
    if (USBSERIAL_DEBUG) safePrintln("IMU: INIT OK");
  } else {
    if (USBSERIAL_DEBUG) safePrintln("IMU: INIT FAILED");
  }

  // local copy of sleep status
  bool mNeedSleep = false;

  // dont sleep again until we have logged a measurement
  bool tmss = false;

  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;

  threads.delay(5000);

  // IMU DATA COLLECTION, CONTROL, AND LOGGING
  while (1) {

    // check if we need to prepare to sleep
    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    if ( mNeedSleep ) {
      if (USBSERIAL_DEBUG) safePrintln("IMU: sleeping");
      imuSleep();

      // safely let the sleep thread know we are ready
      while ( !sr_imu_lock.lock(1000) );
      sr_imu = true;
      sr_imu_lock.unlock();

      // now we are waiting to be put to sleep, and checking if we've woken up
      while (1) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        //threads.yield();

        // if we woke up, reconfigure the IMU
        if ( !mNeedSleep ) {
          if (USBSERIAL_DEBUG) safePrintln("IMU: waking up");
          imuWake();

          // clear the sleep ready flag
          while ( !sr_imu_lock.lock(1000) );
          sr_imu = false;
          sr_imu_lock.unlock();
          break;
        }
      }
    }


    // collect IMU data
    bool gotData = false;
    while ( !i2c_lock.lock(100) );
    if ( myICM.dataReady() ) {
      myICM.getAGMT();      // The values are only updated when you call 'getAGMT'
      ax = myICM.accX();
      ay = myICM.accY();
      az = myICM.accZ();
      gx = myICM.gyrX();
      gy = myICM.gyrY();
      gz = myICM.gyrZ();
      mx = myICM.magX();
      my = myICM.magY();
      mz = myICM.magZ();
      gotData = true;
    }
    i2c_lock.unlock();

    // log IMU data
    if ( gotData ) {
      //safePrintln("IMU: got data");
      unsigned long nn = safeMillis();
      // if radio debug log is enabled, copy the packet into the queue to be sent
      // also conditional on whethere IMU logging is enabled in the ism log settings

      while ( !imuq_lock.lock(100) ); // get lock for telem packet log queue
      if ( imuq_count + 1 < MAX_IMUQ_SIZE ) {
        //safePrintln("creating imu telem packet ");
        imudataq[imuq_write] = new IMUPacket(ax, ay, az, gx, gy, gz, mx, my, mz, nn );
        imuq_write = (imuq_write + 1) % MAX_IMUQ_SIZE;
        imuq_count += 1;
      }
      imuq_lock.unlock();
    }
    threads.delay(IMU_SAMPLE_PERIOD);
  }
}

/**********************************************************************************
   SD Logging thread
*/

void syslog(String s) {
  bool mSysLogCreated = false;
  safeUpdate(&mSysLogCreated, &sysLogCreated, &syslog_lock);
  if ( mSysLogCreated ) {
    unsigned long m = safeMillis();
    int tries = 0;
    while (tries < 5 ){
      if ( sd_lock.lock(20) ){
        sdlog.logMsg(LOGID_SYS, String(m) + "\t" + s);
        sd_lock.unlock();
        break;
      }
      tries++;
    }
  }
}

void sd_thread(int inc) {
  bool ret = false; // for return values
  bool mNeedSleep = false;

  Packet *p;
  
  if (USBSERIAL_DEBUG) safePrintln("SD: thread starting...");

  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  sdlog.begin();
  spi_lock.unlock();
  sd_lock.unlock();

  threads.delay(200);

  // create logfile for accelerometer data
  if (USBSERIAL_DEBUG) safePrintln("SD: Creating logfile for accelerometer data...");
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  String logname = LOGNAME_ACC;
  ret = sdlog.createLog(logname, LOGID_ACC);
  spi_lock.unlock();
  sd_lock.unlock();
  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("SD: ACC FILE OK");
  } else {
    if (USBSERIAL_DEBUG) safePrintln("SD: failed to create acc file");
  }

  threads.delay(200);

  // create log file for tc data
  if (USBSERIAL_DEBUG) safePrintln("Creating logfile for thermocouple data...");
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  logname = LOGNAME_TC;
  ret = sdlog.createLog(logname, LOGID_TC);
  spi_lock.unlock();
  sd_lock.unlock();
  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("SD: TC FILE OK");
  } else {
    if (USBSERIAL_DEBUG) safePrintln("SD: failed to create tc file");
  }

  threads.delay(200);

  // create logfile for telemetry data
  if (USBSERIAL_DEBUG) safePrintln("Creating logfile for telemetry data...");
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  logname = LOGNAME_TELEM;
  ret = sdlog.createLog(logname, LOGID_TELEM);
  spi_lock.unlock();
  sd_lock.unlock();
  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("SD: TELEM FILE OK");
  } else {
    if (USBSERIAL_DEBUG) safePrintln("SD: failed to create telem file");
  }

  threads.delay(200);

  // create logfile for imu data
  if (USBSERIAL_DEBUG) safePrintln("Creating logfile for IMU data...");
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  logname = LOGNAME_IMU;
  ret = sdlog.createLog(logname, LOGID_IMU);
  spi_lock.unlock();
  sd_lock.unlock();
  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("SD: IMU FILE OK");
  } else {
    if (USBSERIAL_DEBUG) safePrintln("SD: failed to create IMU file");
  }

  threads.delay(200);

  // create syslog file
  if (USBSERIAL_DEBUG) safePrintln("Creating logfile for syslog messages...");
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  logname = LOGNAME_SYS;
  ret = sdlog.createLog(logname, LOGID_SYS);
  spi_lock.unlock();
  sd_lock.unlock();
  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("SD: SYSLOG FILE OK");
    safeAssign(&sysLogCreated, true, &syslog_lock);
  } else {
    if (USBSERIAL_DEBUG) safePrintln("SD: failed to create SYSLOG file");
  }

  syslog("syslog created");

  while ( 1 ) {
    // SLEEP HANDLING
    // check if we need to prepare to sleep
    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    if ( mNeedSleep ) {
      if (USBSERIAL_DEBUG) safePrintln("SD: sleeping");

      // safely let the sleep thread know we are ready
      while ( !sr_sd_lock.lock(1000) );
      sr_sd = true;
      sr_sd_lock.unlock();

      // now we are waiting to be put to sleep, and checking if we've woken up
      while (1) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        //threads.yield();
        // if we woke up, reconfigure the IMU
        if ( !mNeedSleep ) {
          if (USBSERIAL_DEBUG) safePrintln("SD: waking up");


          // clear the sleep ready flag
          while ( !sr_sd_lock.lock(1000) );
          sr_sd = false;
          sr_sd_lock.unlock();
          break;
        }
      }
    }
    
    
    ///////////////////////
    // MAIN LOGGING

    // telem logging
    // copy packet from the queue and delete its entry in the queue
    // this way we dont hold the telemq_lock unnecessarily while SD ops happen
    if ( telemq_lock.lock() ){
      if ( telemq_count > 0 ) {
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new TelemPacket(telemdataq[telemq_read]->data());
        delete telemdataq[telemq_read];
        telemq_read = (telemq_read + 1) % MAX_TELEMQ_SIZE;
        telemq_count -= 1;
        telemq_lock.unlock();
        // now try logging packet
        while( !sd_lock.lock() );
        byte ret = sdlog.logBin(LOGID_TELEM, p);
        delete p;
        sd_lock.unlock();
      } else {
        telemq_lock.unlock();
      }
    }

    //threads.delay(10);

    // acc packet logging
    // check for accel stat data to log, w muttex
    if ( accq_lock.lock() ){
      if ( accq_count > 0 ) {
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new AccPacket(accdataq[accq_read]->data());
        delete accdataq[accq_read];
        accq_read = (accq_read + 1) % MAX_ACCQ_SIZE;
        accq_count -= 1;
        accq_lock.unlock();
        // now try logging packet
        while( !sd_lock.lock() );
        byte ret = sdlog.logBin(LOGID_ACC, p);
        delete p;
        sd_lock.unlock();
      } else {
        accq_lock.unlock();
      }
    }

    //threads.delay(10);

    // imu packet logging
    if ( imuq_lock.lock() ){
      if ( imuq_count > 0 ) {
        //if (USBSERIAL_DEBUG) safePrintln("got imu packet to log to SD");
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new IMUPacket(imudataq[imuq_read]->data());
        delete imudataq[imuq_read];
        imuq_read = (imuq_read + 1) % MAX_IMUQ_SIZE;
        imuq_count -= 1;
        imuq_lock.unlock();
        // now try logging packet
        while( !sd_lock.lock() );
        byte ret = sdlog.logBin(LOGID_IMU, p);
        delete p;
        sd_lock.unlock();
      } else {
        imuq_lock.unlock();
      }
    }


    //threads.delay(10);

    // thermocouple data logging
    if ( tcq_lock.lock() ){
      if ( tcq_count > 0 ) {
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new TcPacket((uint8_t*)tcdataq[tcq_read]->data());
        delete tcdataq[tcq_read];
        tcq_read = (tcq_read + 1) % MAX_TCQ_SIZE;
        tcq_count -= 1;
        tcq_lock.unlock();
        // now try logging packet
        while( !sd_lock.lock() );
        byte ret = sdlog.logBin(LOGID_TC, p);
        delete p;
        sd_lock.unlock();
      } else {
        tcq_lock.unlock();
      }
    }
  }
}



/**********************************************************************************
   Accelerometer thread

*/
void acc_thread(int inc) {

  bool tmss = false;
  unsigned long t = 0;
  int x, y, z;

  // local copy of sleep status
  bool mNeedSleep = false;

  while (1) {
    /*// check if we need to prepare to sleep
    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    if ( mNeedSleep ) {
      if (USBSERIAL_DEBUG) safePrintln("ACC: sleeping");
      
      // safely let the sleep thread know we are ready
      while ( !sr_acc_lock.lock(1000) );
      sr_acc = true;
      sr_acc_lock.unlock();

      // now we are waiting to be put to sleep, and checking if we've woken up
      while (1) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        // if we woke up, reconfigure the IMU
        if ( !mNeedSleep ) {
          if (USBSERIAL_DEBUG) safePrintln("ACC: waking up");

          // clear the sleep ready flag
          while ( !sr_acc_lock.lock(1000) );
          sr_acc = false;
          sr_acc_lock.unlock();
          break;
        }
      }
    }*/
    
    // get log timestamp
    t = safeMillis();
    x = analogRead(PIN_ACC_X);
    y = analogRead(PIN_ACC_Y);
    z = analogRead(PIN_ACC_Z);
    
    // LOG THE DATA 
    // push packet into acc data queue to be logged
    while ( !accq_lock.lock(100) ); // get lock for telem packet log queue
    if ( accq_count + 1 < MAX_ACCQ_SIZE ) {
      //if (USBSERIAL_DEBUG) safePrintln("creating acc packet for sd log");
      accdataq[accq_write] = new AccPacket((uint16_t)x, (uint16_t)y, (uint16_t)z, t);
      accq_write = (accq_write + 1) % MAX_ACCQ_SIZE;
      accq_count += 1;
    }
    accq_lock.unlock();

    threads.delay(ACC_SAMPLE_PERIOD);
  }
}
// Compression helper function
size_t pack(uint8_t* src, uint8_t* dst, size_t size){
    size_t workmem_size = blz_workmem_size(size);
    uint8_t workmem[workmem_size];
    return blz_pack(src, dst, size, workmem);
}

/**********************************************************************************
   Compression thread

*/
void compress_thread(int inc) {
  unsigned int num_file_ids = 1;
  int log_ids[num_file_ids] = {LOGID_TC};//, LOGID_ACC, LOGID_IMU};
  // Compressed buffer
  uint8_t c_buf[SBD_TX_SZ];
  // Uncompressed buffer
  uint8_t uc_buf[3*SBD_TX_SZ];
  unsigned int id_idx = 0;
  bool mBuildPacket = false;
  bool mAct = false;

  while(1){
    int pack_size = 0;
    int input_size = 0;
    unsigned long actual_read;

    if( !mAct ){
      safeUpdate(&mAct, &activation, &act_lock);
    }
    
    safeUpdate(&mBuildPacket, &buildPacket, &buildPacket_lock);

#ifdef CONFIG_AUTO_BUILD_PACKETS
    mBuildPacket = true;
#endif    
    
    if( mBuildPacket && mAct){
      
      //syslog("COMPRESS: starting to compress data");

      int offset = 0;


      // does compressing this first
      //pack_size = pack(uc_buf, c_buf, input_size);
      //safePrintln("Compressed " + String(input_size) + " bytes into " + String(pack_size));


      // now calculate the size of the packets we will be adding to the uncompressed buffer 
      // based on type char
      int packet_size;

      switch (log_ids[id_idx])
      {
      case LOGID_TC:
        packet_size = TC_T_SIZE;
        break;
      case LOGID_ACC:
        packet_size = ACC_T_SIZE;
        break;
      case LOGID_IMU:
        packet_size = IMU_T_SIZE;
        break;
      default:
        if(USBSERIAL_DEBUG) safePrintln("Invalid file id specified. Defaulting to TC.");
        packet_size = TC_T_SIZE;
        break;
      }

      packet_size += 1; // Account for char denoting which type of packet

      // keep increasing the input_size number of bytes that we request from a given logfile
      // until the compressed size of those packets will not fit into the iridium buf. when the 
      // compressed size exeeds that limit, start decreasing the number of samples requested until 
      // able to send
      // SBD_TX_SZ - 2 to allow for decompressed data size; first two bytes of compressed buffer represent
      // the decompressed size of the data
      while(pack_size < (SBD_TX_SZ - 2) ){
        
        // dont overflow uc_buf
        if( (input_size + packet_size) > (3 * SBD_TX_SZ) ){
          syslog("COMPRESS: input size would overflow uc_buf");
          break;
        }
        
        input_size += packet_size;
        
        // Grab a uniform sample of packets from the log file.
        while( !sd_lock.lock(10) );
        sdlog.sample(log_ids[id_idx], uc_buf+offset, input_size - offset, &actual_read);
        sd_lock.unlock();

        // if there are not enough samples in the specified logfile, just use what is available
        // compress the uncompressed buffer into the compressed buffer to calculate packed size
        // using the correct buffer size if a short read ocurred
        if(actual_read < (input_size - offset) ){
          pack_size = pack(uc_buf, c_buf+2, actual_read + offset);
          
          // There are not enough packets in the logfile
          safePrintln("Not enough packets in logfile. Requested: " + 
            String(input_size - offset) + " bytes, actually read " + String(actual_read));
          break;
        } 
        // too many bytes back
        else if(actual_read > (input_size - offset) ) {
          safePrintln("COMPRESS: got too many bytes back. Requested: " + 
            String(input_size - offset) + " bytes, actually read " + String(actual_read));
        } 
        // got as many bytes as expected
        else {
          pack_size = pack(uc_buf, c_buf+2, input_size);
        }
        
        //safePrint("Input size: " + String(input_size));
        //safePrint(", packet size: " + String(packet_size));
        //safePrintln(", Offset: " + String(offset));
        //safePrintln("Pack size: " + String(pack_size));

        // yield here otherwise file logging and this thread deadlock on sd_lock (?)
        threads.yield();
      }

      // if our packed size won't fit in an iridium buffer, start scaling back
      while (pack_size > (SBD_TX_SZ - 2) ){
        input_size -= packet_size;

        // resample log file
        while( !sd_lock.lock(10) );
        sdlog.sample(log_ids[id_idx], uc_buf+offset, input_size - offset, &actual_read);
        sd_lock.unlock();

        // if we got a short read again, let the pack function know the right buffer size
        if( (input_size - offset) != actual_read ){
          // recalc packed size
          pack_size = pack(uc_buf, c_buf+2, actual_read + offset);         
        } 
        
        // if it wasn't a short read,  use the expected size 
        else {
          // recalc packed size
          pack_size = pack(uc_buf, c_buf+2, input_size);
        }  
      }

      // Get latest telem packet SECOND
      while( !sd_lock.lock(10) );
      int lpbr = sdlog.latest_packet(LOGID_TELEM, uc_buf);
      sd_lock.unlock();

//      #if USBSERIAL_DEBUG
//      safePrint("COMPRESS: got latest packet bytes:\n");
//      for( int ii=0; ii<offset; ii++){
//        safePrint((uint8_t)uc_buf[ii]);
//        safePrint(" ");
//      }
//      safePrintln("");
//      #endif
//      
//      if( offset == (TELEM_T_SIZE + 1) ){
//        syslog("COMPRESS: telem packet for header is correct size, offset=" + String(offset));
//        syslog("COMPRESS: uc_buf[0] = " + String((char)uc_buf[0]));
//        input_size += offset;
//      } else {
//        syslog("COMPRESS: telem packet for header is NOT correct size, offset=" + String(offset));
//        syslog("COMPRESS: uc_buf[0] = " + String((uint8_t)uc_buf[0]) + " in decimal");
//        input_size = TELEM_T_SIZE+1;
//      }
      

      // finally, set the first two bytes of the compressed buffer to the original data size
      *(uint16_t*)(&c_buf[0]) = (uint16_t)(actual_read+offset);
      
      while( !irbuf_lock.lock(10) );
      memcpy(irbuf, c_buf, pack_size);
      irbuf_len = pack_size;
      irbuf_ready = true;
      irbuf_lock.unlock();
      
      if(USBSERIAL_DEBUG) safePrintln("Packed " + String(actual_read + offset)  + " bytes into SBD packet");
      if(USBSERIAL_DEBUG) safePrintln("Compressed size: " + String(pack_size)  + "bytes");
      id_idx = (id_idx+1) % num_file_ids;

      syslog("COMPRESS: filled iridium buffer,comp size: " + String(pack_size)  + " bytes, orig size: " + String(input_size));
      
      mBuildPacket = false;
      safeAssign(&buildPacket, false, &buildPacket_lock);
    }
    threads.delay(15000);
  }
}

/***********************************************************************************
   sleep thread
*/
void sleep_thread(int inc) {
  // CONFIGURE SLEEP PARAMS

  int  who = -1;          // output of sleep function
  bool mAct = false; // our copy of activation status
  bool mTcAct = false; // tc detects activation
  bool mCpAct = false; // cap sense detects activation
  bool mDrp = false; // our copy of debug radio present;
  int  state = 0;    // state for pre-activation sleep routine
  int  sleepCount = 0; // for keeping track of longer periods of time
  bool mNeedSleep = 0; // our copy of the global sleep control flag

  bool ss_radio = false,
       ss_tc    = false,
       ss_acc   = false,
       ss_imu   = false,
       ss_ir    = false;

  if (USBSERIAL_DEBUG) safePrintln("SLEEP: starting sleep thread");

  threads.delay(10000);

  // SLEEP CONTROL prior to activation
  while (1) {

    // delay to allow TC thread to take some measurements before deciding on activation
    // chance to make a TC measurement and update our decicision about activation
    // this should be long enough to create our logs
    threads.delay(SLEEP_TIME_AWAKE);

    // check activation status
    safeUpdate(&mTcAct, &tcAct, &tcAct_lock);
    safeUpdate(&mCpAct, &capAct, &capAct_lock);

    if( CONFIG_USE_ACT_TC && CONFIG_USE_ACT_CAP ){
      mAct = mCpAct && mTcAct;
    } else if( CONFIG_USE_ACT_TC && !CONFIG_USE_ACT_CAP ){
      mAct = mTcAct;
    } else if( !CONFIG_USE_ACT_TC && CONFIG_USE_ACT_CAP ){
      mAct = mCpAct;
    } else {
      safeUpdate(&mAct, &activation, &act_lock);
    }

    // if the cap sensor is activated, but overall activation didn't happen
    //  then we really shouldnt go back to sleep because
    // more than likely the thermocouples just need to catch up
    if( CONFIG_USE_ACT_TC && CONFIG_USE_ACT_CAP && mCpAct && !mAct ){
      // just go up and wait then check activation sources again
      continue;
    }
    
    if ( mAct ) {
      if (USBSERIAL_DEBUG) safePrintln("SLEEP: DETECTED ACTIVATION");
      syslog("SLEEP: activation detected, exiting sleep thread");
      // let the threads know sleeping has finished      
      safeAssign(&needSleep, false, &ns_lock);
      safeAssign(&activation, true, &act_lock);
      //tone(PIN_BUZZER, 1000);
      break;
    }

    switch (state) {
      // not activated, need to sleep
      case 0:

        // for testing, we will just idle in this state until receiving a debug command telling us to sleep
        // this prevents early disconnects in serial logging etc
        if( MISSION_TYPE == MISSION_TEST_JSC || MISSION_TYPE == MISSION_TEST_SHUTTLE ){
          safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
          if( mNeedSleep ){
            syslog("SLEEP: someone else initiated sleep, advancing to state 1");
            state = 1;
          }
        } 

        // the above can be overriden by automatic sleep:
        if( CONFIG_AUTOMATIC_SLEEP ){        
          // for real mission, go to sleep after the initial delay specified above
          safeAssign(&needSleep, true, &ns_lock);
          state = 1;
        }        
        break;

      // needing sleep, have to wait for each thread to be ready
      // need to be kind about checking status of each threads readiness
      case 1:

        if (USBSERIAL_DEBUG) safePrintln("SLEEP: requesting threads prepare for sleep");

        // update our copy of each threads sleep status
        safeUpdate(&ss_radio, &sr_radio, &sr_radio_lock);
        safeUpdate(&ss_tc,    &sr_tc,    &sr_tc_lock);
        safeUpdate(&ss_acc,   &sr_acc,   &sr_acc_lock);
        safeUpdate(&ss_imu,   &sr_imu,   &sr_imu_lock);
        safeUpdate(&ss_ir,    &sr_ir,    &sr_ir_lock);

        // the radio thread is always ready to sleep when no radio is present
        safeUpdate(&mDrp, &debugRadioPresent, &drp_lock);
        if ( !ss_radio && mDrp ) {
          ss_radio = false;
        } else {
          ss_radio = true;
        }

        // if all the threads are ready to sleep
        if ( ss_tc && ss_imu && ss_radio) {
          if (USBSERIAL_DEBUG) safePrintln("SLEEP: all threads acknowledged request for sleep, proceeding");
          syslog("SLEEP: all threads acknowledged request for sleep, proceeding");
          state = 2;
        }
      
      // save power in sleep
//      pinMode(CS_TC1, INPUT);
//      pinMode(CS_TC2, INPUT);
//      pinMode(MUX0, INPUT);
//      pinMode(MUX1, INPUT);
//      pinMode(11, INPUT);
//      pinMode(13, INPUT);  
      break;

      // all threads have prepared for sleep, lets do it
      case 2:
        // now we go to sleep
        SLEEP:
        who = Snooze.hibernate( config_teensy35 ); // return module that woke processor
        
        // if we are woken up by the timer, increment the sleep count
        if( who == 36 ){
          sleepCount += 1;

          // if we should go back to sleep, fall through and do state 2 again
          if( sleepCount < SLEEP_DURATION_MINUTES ){
            goto SLEEP;
          } 
        }
        
        // if we were woken by a digital interrupt from the capacitance sensor
        else if( who == 2 ){
          syslog("SLEEP: woken by capsense");
        }

        else {
          syslog("SLEEP: woken by unknown source: " + String(who));
        }
        
        // otherwise we're awake! let the threads know sleeping has finished... for now        
        safeAssign(&needSleep, false, &ns_lock);

        if (USBSERIAL_DEBUG) safePrintln("SLEEP: WOKE UP BY: " + String(who));

        // normal pin mode
//        pinMode(CS_TC1, OUTPUT);
//        pinMode(CS_TC2, OUTPUT);
//        pinMode(MUX0, OUTPUT);
//        pinMode(MUX1, OUTPUT);
//        pinMode(11, OUTPUT);
//        pinMode(13, OUTPUT);
        

        // back to waiting on threds to sleep
        state = 0;
        sleepCount = 0;
        break;
      
    }

    threads.delay(50);
  }

  // let the activation buzzer run for three seconds
  //threads.delay(3000);
  //noTone(PIN_BUZZER);
  //digitalWrite(PIN_BUZZER, LOW);

  // now we are activated!
  while (1) {
    threads.yield();
    // mybe just end the thread instead of looping here?
  }
}


/***********************************************************************************
   command processing thread
   receive a packet over debug radio, process it here
   only applies to test missions where receiving commands is applicable
*/
#ifdef ISM_DEBUG

void command_thread(int inc) {

  threads.delay(5000);
  
  syslog("CMDTHRD: starting");

  while(1){

    // get access to the command packtet buffer
    if( cmdPacket_lock.lock() ){

      // if there are command packets to parse
      if( cmdPacketCount > 0 ){

        // loop through each packet
        for( int i=0; i < cmdPacketCount; i++ ){

          // decide packet action
          switch( cmdPackets[i]->cmd() ){

            // SIMULATE AN ACTIVATION CONDITION
            case CMDID_ACTIVATE:
              safeAssign(&activation, true, &act_lock);
              if (USBSERIAL_DEBUG) safePrintln("CMDTHRD: manual activation detected");
              syslog("CMDTHRD: manual activation");
              break;

            // TRIGGER A SAMPLE OPERATION AND FILLING THE IRIDIUM BUFFER
            case CMDID_IR_BP:
              syslog("CMDTHRD: building packet");
              safePrintln("CMDTHRD: building packet");
              safeAssign(&buildPacket, true, &buildPacket_lock);
              break;

            // TRIGGER SENDING THE IRIDIUM BUFFER
            case CMDID_IR_SP:
              //syslog("CMDTHRD: sending iridium packet");
              break;

            // PUT THE FLIGHT COMPUTER TO SLEEP
            case CMDID_SLEEP:
              syslog("CMDTHRD: recieved start sleep command");
              safeAssign(&needSleep, true, &ns_lock);
              break;
              
            default:
              break;
          }
          delete cmdPackets[i];
        }
        cmdPacketCount = 0;
      }
      cmdPacket_lock.unlock();
    }


    threads.delay(100);
  }

}
#endif

/***********************************************************************************
   Thermocouple manager thread
   keeps conversions happening as fast as possible
   and updates a shared struct with new data
*/
void tc_thread(int inc) {
  tcv_t mTcReadings; // for communicating with the TcInterface class
  unsigned long lastData = safeMillis();

  bool mNeedSleep = false;
  bool mAct = false;
  bool ret = false;

  if (USBSERIAL_DEBUG) safePrintln("TC: starting thread");

  // don't pass go unless we can init the thermocouples
  while ( !ret ) {
    while ( !spi_lock.lock(1000) );
    ret = tc.enable();
    spi_lock.unlock();

    if ( ret ) {
      if (USBSERIAL_DEBUG) safePrintln("TC: INIT OK");
    } else {
      if (USBSERIAL_DEBUG) safePrintln("TC: INIT FAIL!!");
    }
  }

  // main execution loop
  while (1) {

    // check if we need to prepare to sleep
    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    
    if ( mNeedSleep ) {
      if (USBSERIAL_DEBUG) safePrintln("TC: sleeping");

      //pinMode(MUX0, INPUT);
      //pinMode(MUX1, INPUT);
      //pinMode(MOSI, INPUT);
      //pinMode(SCK, INPUT);

      // safely let the sleep thread know we are ready
      while ( !sr_tc_lock.lock(1000) );
      sr_tc = true;
      sr_tc_lock.unlock();

      // now we are waiting to be put to sleep, and checking if we've woken up
      while (1) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        // if we woke up, enable the TCs again
        //threads.yield();
        if ( !mNeedSleep ) {
          if (USBSERIAL_DEBUG) safePrintln("TC: waking up");

          //pinMode(MUX0, OUTPUT);
          //pinMode(MUX1, OUTPUT);
          //pinMode(MOSI, OUTPUT);
          //pinMode(SCK, OUTPUT);

          // clear the sleep ready flag
          while ( !sr_tc_lock.lock(10) );
          sr_tc = false;
          sr_tc_lock.unlock();
          break;
        }
      }
    }

    // NOW START TAKING  MEASUREMENTS

    // forceStart prevents TC read hangs if we havent gotten any data in 5 seconds
    bool forceStart = safeMillis() - lastData > 5000 ? true : false;
    if ( forceStart ) lastData = safeMillis();

//digitalWrite(LED_ACT, HIGH);

    // make call to TC interface
    while ( !spi_lock.lock(10) );
    //safePrintln(String(safeMillis())+ "  TC: mutex was unlocked, got it ");
    bool gotData = tc.read_all(&mTcReadings, forceStart);
    spi_lock.unlock();
    //safePrintln("TC: read vals");
      
    //digitalWrite(LED_ACT, LOW);
      
    if ( gotData ) {
      if (USBSERIAL_DEBUG) safePrintln("TC: got TC readings");

      

      // this data's timestamp
      unsigned long nn = safeMillis();

      // check activation status
      safeUpdate(&mAct, &activation, &act_lock);
      if ( !mAct ) {
        // check for activation conditions
        int exceeds = 0;
        for ( int i = 0; i < TC_COUNT; i++ ) {
          if ( mTcReadings.data[i] >= TC_THRESHOLD ) {
            exceeds += 1;
          }
        }
        if ( exceeds >= TC_CONSENSUS ) {
          if (USBSERIAL_DEBUG) safePrintln("TC: DETECTED ACTIVATION");
          // log this over radio too
          mAct = true;
          safeAssign(&tcAct, true, &tcAct_lock);
        }
      }

      // quick hack
      // only log data since activation
      if( mAct ){
        // LOG THE DATA 
        // push packet into tc data queue to be logged
        while ( !tcq_lock.lock(100) ); // get lock for telem packet log queue
        if ( tcq_count + 1 < MAX_TCQ_SIZE ) {
          //if (USBSERIAL_DEBUG) safePrintln("creating tc packet ");
          tcdataq[tcq_write] = new TcPacket(mTcReadings.data, nn);
          tcq_write = (tcq_write + 1) % MAX_TCQ_SIZE;
          tcq_count += 1;
        }
        tcq_lock.unlock();

      }

      // radio log
      // push another copy of our data into queue for radio log if enabled
      if( ISM_DEBUG && logen_tc){
        while ( !logtcq_lock.lock(100) ); // get lock for telem packet log queue
        if ( logtcq_count + 1 < MAX_TCQ_SIZE ) {
          //if (USBSERIAL_DEBUG) safePrintln("creating radio tc packet ");
          logtcdataq[logtcq_write] = new TcPacket(mTcReadings.data, nn);
          logtcq_write = (logtcq_write + 1) % MAX_TCQ_SIZE;
          logtcq_count += 1;
        }
        logtcq_lock.unlock();
      }
      
      lastData = nn;

      
    } else {
      // no new tc data
      threads.yield();
    }
    threads.delay(20);
  }
}


/***********************************************************************************
   radio interface thread
   publishes any debug messages and packets generated in other threads
   listens for command packets and dispatches them to command thread
*/
void radio_thread(int inc) {
  bool ret = false;
  Packet *p;

  while ( !spi_lock.lock(1000) );
  ret = logNode.begin();
  logNode.setRetries(1);
  spi_lock.unlock();

  if ( ret ) {
    if (USBSERIAL_DEBUG) safePrintln("ISM: log node started");
    syslog("ISM: log node started");
    if( CONFIG_USE_LEDS) analogWrite(LED_ISM_TX, 5); // dim lit
  } else {
    if (USBSERIAL_DEBUG) safePrintln("ISM: log node failed to start, idling thread");
    syslog("ISM: log node failed to start, idling thread");
    while (1) threads.yield();
  }

  bool mNeedSleep = false;
  bool newData = false;
  bool sent = false;

  while (1) {

    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    if ( mNeedSleep ) {
      if (USBSERIAL_DEBUG) safePrintln("ISM: sleeping");
      safeAssign(&sr_radio, true, &sr_radio_lock);
      //logNode.sleep();
      while ( mNeedSleep ) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        //threads.yield();
        if ( !mNeedSleep ) {
          if (USBSERIAL_DEBUG) safePrintln("ISM: waking up");
          safeAssign(&sr_radio, false, &sr_radio_lock);
        }
        threads.delay(50);
      }
    }
    
    // check for a packet in the logtcqueue
    newData = false;
    if ( logtcq_lock.lock() ){
      if ( logtcq_count > 0 ) {
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new TcPacket((uint8_t*)logtcdataq[logtcq_read]->data());
        delete logtcdataq[logtcq_read];
        logtcq_read = (logtcq_read + 1) % MAX_TCQ_SIZE;
        logtcq_count -= 1;
        logtcq_lock.unlock();
        newData = true;
      } else {
        logtcq_lock.unlock();
      }
    }
    // radio log the tc packet if we have one
    if( newData ){
      // get SPI mutex and send packet over radio
      //if (USBSERIAL_DEBUG) safePrintln("ISM:  about to send ism packet");
      if( CONFIG_USE_LEDS) analogWrite(LED_ISM_TX, 100);

      sent = false;
      while ( !sent ) {
        if (  spi_lock.lock(1000) ) {
          // RH_BROADCAST_ADDRESS
          if ( logNode.send(p, RH_BROADCAST_ADDRESS) ) {
            sent = true;
          }
          spi_lock.unlock();
          if ( sent ) {
            //if (USBSERIAL_DEBUG) safePrintln("sent");
          } else {
            //if (USBSERIAL_DEBUG) safePrintln("failed");
          }
        }
      }
      delete p;
      if( CONFIG_USE_LEDS) analogWrite(LED_ISM_TX, 5);
    }

    // telem radio log
    // check for a telem packet in the logtelemqueue
    newData = false;
    if ( logtelemq_lock.lock() ){
      if ( logtelemq_count > 0 ) {
        // have to get non volatile pointer to the memory or 
        // the Packet constructor complains
        p = new TelemPacket(logtelemdataq[logtelemq_read]->data());
        delete logtelemdataq[logtelemq_read];
        logtelemq_read = (logtelemq_read + 1) % MAX_TELEMQ_SIZE;
        logtelemq_count -= 1;
        logtelemq_lock.unlock();
        newData = true;
      } else {
        logtelemq_lock.unlock();
      }
    }
    
    // send new telem packet if there is one
    if ( newData ) {
      if( CONFIG_USE_LEDS) analogWrite(LED_ISM_TX, 100);
      sent = false;
      while ( !sent ) {
        if (  spi_lock.lock(1000) ) {
          if ( logNode.send(p, RH_BROADCAST_ADDRESS) ) {
            sent = true;
          }
          spi_lock.unlock();
          if ( sent ) {
            //if (USBSERIAL_DEBUG) safePrintln("sent");
          } else {
            //if (USBSERIAL_DEBUG) safePrintln("failed");
          }
        }
      }
      delete p;
      if( CONFIG_USE_LEDS) analogWrite(LED_ISM_TX, 5);
    }

    // check if there is a recevied packet waiting for us
    newData = false;
    if (  spi_lock.lock(20) ) {
      newData = logNode.available();
    }
    //spi_lock.unlock();
    
    // if we have a new packet waiting
    if( newData ) {

      //safePrintln("log node available");
      
      // receive it
      //while(  !spi_lock.lock(20) );
      bool rcvd = logNode.receivePackets();
      //spi_lock.unlock();

      
      // if we received it, decode it into a packet in the logNode packet buffer
      if( rcvd ){

        // build packets from recieved data
        //while(  !spi_lock.lock(10) );
        int np = logNode.decodePackets();
        //spi_lock.unlock();

        syslog("RADIO: recieved" + String(np) + " new packet(s)");
        
        // if it is in the packet buffer, copy it to the command buffer
        if( np > 0 ){
          
          // copy packets from logNode to command buffer, only if they are command packets
          while( !cmdPacket_lock.lock(10) );
          for( int i=0; i<np; i++ ){
            if( logNode.packets()[i]->type() == PTYPE_COMMAND ){
              cmdPackets[cmdPacketCount] = new CommandPacket(logNode.packets()[i]->data());
              cmdPacketCount++;
            }
          }
          cmdPacket_lock.unlock();

          // free memory of packets that were recieved
          //while(  !spi_lock.lock(10) );
          logNode.deletePackets();
          //spi_lock.unlock();
        }
      } else  {
        syslog("RADIO: failed to recieve packet");
      }

      newData = false;
    } 
    spi_lock.unlock();
    
    
    threads.delay(50);
  }
}


/***********************************************************************************
   Telem gathering thread for board info
*/
void telem_thread(int inc) {
  bool mNeedSleep = false;
  unsigned long ts = 0;
  float vbat = 0; 
  int batt = 0;
  int logNum = -1;
  
  if (USBSERIAL_DEBUG) safePrintln("TELEM: starting thread");

  threads.delay(7000);

  // get current logNum
  while ( !sd_lock.lock(1000) );
  while ( !spi_lock.lock(1000) );
  logNum = sdlog.logNum();
  spi_lock.unlock();
  sd_lock.unlock();

  //if (USBSERIAL_DEBUG) safePrintln("TELEM: got logum of "+String(logNum));

  // signal quality of iridium modem
  static uint8_t s = 0;

  while (1) {
    // check for need to sleep and handle properly
    safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
    if ( mNeedSleep ) {
      if (USBSERIAL_DEBUG) safePrintln("TELEM: sleeping");
      safeAssign(&sr_telem, true, &sr_telem_lock);

      while ( mNeedSleep ) {
        safeUpdate(&mNeedSleep, &needSleep, &ns_lock);
        if ( !mNeedSleep ) {
          if (USBSERIAL_DEBUG) safePrintln("TELEM: waking up");
          safeAssign(&sr_telem, false, &sr_telem_lock);
        }
        threads.delay(50);
      }
    }

    if( logNum == -1 ){
      // get current logNum
      while ( !sd_lock.lock(1000) );
      while ( !spi_lock.lock(1000) );
      logNum = sdlog.logNum();
      spi_lock.unlock();
      sd_lock.unlock();
    }

    // process analog conversion to read battery voltage
    // vbat is split through two 100k resistors and connected to PIN_BAT_SENSE
    // resoltution configurable
    float vout = 
    batt = analogRead( PIN_BAT_SENSE ); //PIN_BAT_SENSE
    vbat = ((float)(batt)) / 1023.0;
    vbat = (vbat * 3.3) / VBAT_DIV;

    ts = safeMillis(); // get current as possible timestamp

    if( sq_lock.lock(10) ){
      s = signalQuality;
      sq_lock.unlock();
    }
    int mIrpc = 0;
    if( irpc_lock.lock(10) ){
      mIrpc = irpc;
      irpc_lock.unlock();
    }

    bool mAct = false;
    safeUpdate(&mAct, &activation, &act_lock);

    while ( !telemq_lock.lock(100) ); // get lock for telem packet log queue
    if ( telemq_count + 1 < MAX_TELEMQ_SIZE ) {
      telemdataq[telemq_write] = new TelemPacket(ts, vbat, (float)mIrpc, (float)mAct, s, (uint8_t)logNum);  // TODO: fill in tc1 and tc2 temp and irsig
      telemq_write = (telemq_write + 1) % MAX_TELEMQ_SIZE;
      telemq_count += 1;
    }
    telemq_lock.unlock();

    if( ISM_DEBUG  && logen_telem ){
      while ( !logtelemq_lock.lock(100) ); // get lock for telem packet log queue
      if ( logtelemq_count + 1 < MAX_TELEMQ_SIZE ) {
        logtelemdataq[logtelemq_write] = new TelemPacket(ts, vbat, (float)mIrpc, (float)mAct, s, (uint8_t)logNum);  // TODO: fill in tc1 and tc2 temp and irsig
        logtelemq_write = (logtelemq_write + 1) % MAX_TELEMQ_SIZE;
        logtelemq_count += 1;
      }
      logtelemq_lock.unlock();
      //if (USBSERIAL_DEBUG) safePrintln("TELEM: got logum of "+String(logNum));
    }

    threads.delay(2000);

    // if ism log enabled and  telem log enabled in wireless log settings,
    // create a packet and add it to the outgoing log msg queue
    // the data is copied into that buffer so the


    // its okay to reuse that same packet object and copy it into the logging threads
    // queue of packets to be logged
    // then the packet goes out of scope and is deleted
  }
}


/***********************************************************************************
   Iridium model control thread
*/
void iridium_thread(int inc) {

  bool mAct = false;
  bool mBufReady=false, mirready=false;
  int mBufLen = 0;
  int mSq = 0;
  unsigned long curMillis = 0;
  unsigned long signalCheckInterval = 15000;
  unsigned long lastSignalCheck = 0;
  uint8_t irbuf_internal[SBD_TX_SZ];

  // before activation and during sleep, set serial port pins to iridium to high impedance
  // to limit power consumption of max3232e 
  pinMode(PIN_IRIDIUM_TX, INPUT_PULLUP);
  pinMode(PIN_IRIDIUM_RX, INPUT_PULLUP);

  // pre activation routine
  while (1) {
    // update the activation state
    safeUpdate(&mAct, &activation, &act_lock);

    // if we are activated, break and move on
    if ( mAct ) {
      if (USBSERIAL_DEBUG) safePrintln("IRIDIUM: activation detected");
      // debug this over radio log too
      break;
    }

    threads.delay(100);
  }

  // Start the serial port connected to the satellite modem
  IRIDIUM_SERIAL.begin(9600);

  if (USBSERIAL_DEBUG) safePrintln("IRIDIUM: Powering on modem...");
  digitalWrite(PIN_IR_ENABLE, HIGH);
  if( CONFIG_USE_LEDS) analogWrite(LED_IR_ON, 5); // not blinding
  threads.delay(2000);

  // Begin satellite modem operation
  if (USBSERIAL_DEBUG) safePrintln("IRIDIUM: Starting modem...");
  irerr = modem.begin();
  if (irerr != ISBD_SUCCESS)
  {
    if (USBSERIAL_DEBUG) safePrint("IRIDIUM: Begin failed: error ");
    if (USBSERIAL_DEBUG) safePrintln(irerr);
    if (irerr == ISBD_NO_MODEM_DETECTED)
      if (USBSERIAL_DEBUG) safePrintln("IRIDIUM: No modem detected: check wiring.");
    return;
  }

  // Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  irerr = modem.getSignalQuality(mSq);
  if (irerr != ISBD_SUCCESS)
  {
    if (USBSERIAL_DEBUG) safePrint("IRIDIUM: SignalQuality failed: error ");
    if (USBSERIAL_DEBUG) safePrintln(irerr);
    syslog("IRIDIUM: failed to get first signal strength reading");
    //TODO: error handling
    //return;
  } else {
    syslog("IRIDIUM: first signal strength: " + String(mSq));
  }

  
  while( !sq_lock.lock(10) );
  signalQuality = mSq;
  sq_lock.unlock();

  if( mSq > 0 ){
    safeAssign(&irready, true, &irready_lock);
    syslog("IRIDIUM: ready to send a packet");
  }
  
  if (USBSERIAL_DEBUG) safePrint("IRIDIUM: On a scale of 0 to 5, signal quality is currently ");
  if (USBSERIAL_DEBUG) safePrint(signalQuality);
  if ( signalQuality >= 0 && signalQuality <= 5 ) {
    if( CONFIG_USE_LEDS) analogWrite(LED_IR_SIG, signalBrightness[signalQuality]);
  }

  // start main packet send loop
  while (1) {
    threads.delay(50);


    ///////////////////
    // send data buffer command
    ////////////////////

    while( !sq_lock.lock(10) );
    signalQuality = mSq;
    sq_lock.unlock();
    mirready = mSq > 0;
    
    safeUpdate(&mBufReady, &irbuf_ready, &irbuf_lock);
    
    // if the radio is ready and the buffer is full
    if ( mirready && mBufReady ) {
      // Send the message
      if (USBSERIAL_DEBUG) safePrintln("IRIDIUM: sending data buffer over iridium\r\n");
      syslog("IRIDIUM: sending data buffer");
      if( CONFIG_USE_LEDS) analogWrite(LED_IR_TX, 20);

      // copy the packet locally so that other threads can start filling up another packet
      // while the iridium thread sends the current one.
      while( !irbuf_lock.lock(100) );
      memcpy(irbuf_internal, irbuf, irbuf_len);
      mBufLen = irbuf_len;
      irbuf_lock.unlock();

      irerr = modem.sendSBDBinary(irbuf_internal, mBufLen);
      
      if( CONFIG_USE_LEDS) analogWrite(LED_IR_TX, 0);
      if (irerr != ISBD_SUCCESS)
      {
        if (USBSERIAL_DEBUG) safePrint("IRIDIUM: sendSBDBinary failed: error ");
        if (USBSERIAL_DEBUG) safePrintln(irerr);
        if (irerr == ISBD_SENDRECEIVE_TIMEOUT)
          if (USBSERIAL_DEBUG) safePrintln("IRIDIUM: Try again with a better view of the sky.");

        syslog("IRIDIUM: packet send fail, err="+String(irerr));
      }

      else
      {
        if (USBSERIAL_DEBUG) safePrintln("*****************");
        if (USBSERIAL_DEBUG) safePrintln("*** SENT PACKET *");
        if (USBSERIAL_DEBUG) safePrintln("*****************");
        
        mBufReady = false;
        safeAssign(&irbuf_ready, false, &irbuf_lock);
        while( !irpc_lock.lock(10) );
        irpc++;
        irpc_lock.unlock();
        syslog("IRIDIUM: sent binary packet");
      }
    }

    ///////////////////
    // CHECK THE SINGL QUALITY PERIODICALLY
    ////////////////////
    curMillis = safeMillis();
    if ( curMillis - lastSignalCheck > signalCheckInterval ) {
      irerr = modem.getSignalQuality(mSq);

      
      if (irerr != ISBD_SUCCESS) {
        if (USBSERIAL_DEBUG) safePrint("IRIDIUM: SignalQuality failed: error ");
        if (USBSERIAL_DEBUG) safePrintln(irerr);
        syslog("IRIDIUM: failed to get signal quality");
        //TODO: error handling
        //return;
      } else {
        if (USBSERIAL_DEBUG) safePrint("IRIDIUM: On a scale of 0 to 5, signal quality is currently ");
        if (USBSERIAL_DEBUG) safePrint(signalQuality);
        if (USBSERIAL_DEBUG) safePrintln(".");
        syslog(String("IRIDIUM: signal quality is ") + String(signalQuality));

        if( CONFIG_USE_LEDS ){
          if( signalQuality >= 0 && signalQuality <= 5 ) {
            analogWrite(LED_IR_SIG, signalBrightness[signalQuality]);
          }
        }
        
        while( !sq_lock.lock(10) );
        signalQuality = mSq;
        sq_lock.unlock();
        
        if( mSq > 0 ){
          safeAssign(&irready, true, &irready_lock); 
        }
      }
      lastSignalCheck = curMillis;
    }

  }
}


/***********************************************************************************
   Capacitive sensing thread
*/
void cap_thread(int inc) {
  bool mAct = false;    // local copy of global activation status
  bool mCapVal = false; // current gpio pin state

  // previous readings of capacitive sense value, 
  // must all agree for activation status to be updated
  // for real mission, activation is represented by no sensed capacitance, i.e. capVal=0
  // set all values to 1 initially
  int histLen = 10;
  uint8_t capHistory[histLen];
  memset(capHistory, 1, sizeof(capHistory));

  int pos = 1;

  threads.delay(5000);
  
  while (1) {
    safeUpdate(&mAct, &activation, &act_lock);

    mCapVal = digitalRead(2);
    
    safeAssign(&capReading, mCapVal, &capReading_lock);

    //Serial.println("CAP: capval = " + String(mCapVal));

    capHistory[pos] = mCapVal;
    pos = (pos + 1) % histLen;
    
    if(USBSERIAL_DEBUG) safePrintln("CAP: capval="+String(mCapVal) + ", pos="+String(pos));

    // set capAct to true then logic AND with all of the readings in the buffer,
    // if at the end of the loop capAct is still true then all of the readings 
    // must have been true too, which means activation is detected
    bool mCapAct = true;
    for( int i=0; i<histLen; i++ ){
      mCapAct = capHistory[i] && mCapAct;
    }

    // if we just detected activation and we aren't already activated, activate
    if( pos==0 && mCapAct==true && !mAct ){
      safePrintln("CAPTHRD: detected activation!");
      
      syslog("CAP: detected activation, setting capVal=true");

      safeAssign(&capAct, true, &capAct_lock);
      
      //safeUpdate(&activation, &capAct, &act_lock);

      // idle infinitely now that we are activated, to give other threads more time
      while(1) threads.yield();
    }

    //digitalWrite(LED_ACT, mCapVal);

    threads.delay(200); // take cap sense readings at ~5hz
  }
  
}


/***********************************************************************************
   Parachute deployment thread
*/

void deploy_parachute(){ // Parachute Deployment function
  // TODO figure out how the parachute is deployed
}

void par_thread(int inc){
  
//   bool deployed = false;
//   float x_accel_copy = -20.0;
//   float y_accel_copy = -20.0;
//   float z_accel_copy = -20.0;
  
//   int elapsed_accel_time = 0;
  
//   int initial_accel_time = 0;
  
  while(1){
    
    if (deployed) continue;
    
    int timer = safeMillis();
    
//     x_accel_copy = analogRead(PIN_ACC_X)
//     y_accel_copy = analogRead(PIN_ACC_Y)
//     z_accel_copy = analogRead(PIN_ACC_Z)
    
    // Or 
    
    // x_accel_copy = myICM.accX();
    // y_accel_copy = myICM.accY();
    // z_accel_copy = myICM.accZ();
    
    if (timer > parachute_timer){ // If we're past the time (Define time in the preamble)
//       if (USBSERIAL_DEBUG) safePrintln("Parachute: past deployment time, checking accelerometer data");
//       if ((abs(x_accel_copy) < accelerometer_threshold) && (abs(y_accel_copy) < accelerometer_threshold) && (abs(z_accel_copy) < accelerometer_threshold)) { // check if accelerometer data is correct (Define threshold in the preamble, currently set as 1E-2)
        
//         if (USBSERIAL_DEBUG) safePrintln("Parachute: accelerometer data matches");
        
//         if (initial_accel_time == 0) { // Record time when acceleration first met the parameters
//           initial_accel_time = timer;
//         }
        
//         elapsed_accel_time = timer - initial_accel_time;
        
//         if (elapsed_accel_time > total_necessary_accel_time){ // If enough time has passed with the acceleration meeting the threshold, deploy! (Define time in the preamble)
//           if (USBSERIAL_DEBUG) safePrintln("Parachute: accelerometer data matches and total necessary time has been met");
//           deployed = true;
//         }
        
//       }
//       else if (initial_accel_time != 0) { // Reset timer in case acceleration doesn't continue to meet the threshold
//         initial_accel_time = 0;
//       }
       
       if (USBSERIAL_DEBUG) safePrintln("Parachute: past deployment time, deploying!!!!");
       deployed = true
       
    }
    
    if (deployed){
      if (USBSERIAL_DEBUG) safePrintln("Parachute: deployed!!!!");
      deploy_parachute();
    }
    
  }
  
}

/***********************************************************************************
   Setup
*/
void setup() {
#if USBSERIAL_DEBUG
  Serial.begin(115200);
#endif


  // LED pin setup
  if( CONFIG_USE_LEDS ){
    pinMode(LED_IR_ON, OUTPUT);
    pinMode(LED_IR_SIG, OUTPUT);
    pinMode(LED_IR_TX, OUTPUT);
    pinMode(LED_ISM_TX, OUTPUT);
    pinMode(LED_ACT, OUTPUT);
  }

  // iridium control pin setup
  pinMode(PIN_IR_ENABLE, OUTPUT);
  digitalWrite(PIN_IR_ENABLE, LOW);
  pinMode(PIN_IRIDIUM_TX_ACT, INPUT);
  pinMode(PIN_IRIDIUM_STATUS, INPUT);
  
  // debug radio present (active high)
  pinMode(PIN_ISM_PRESENT, INPUT);

  // high g accel analog input config
  analogReadAveraging(64);
  analogReadResolution(10);
  pinMode(PIN_ACC_X, INPUT);
  pinMode(PIN_ACC_Y, INPUT);
  pinMode(PIN_ACC_Z, INPUT);

  // battery monitoring input config
  // (charging/not charging and level)
  pinMode(PIN_BAT_STAT, INPUT);
  pinMode(PIN_BAT_SENSE, INPUT);

  // startup beep
//  pinMode(PIN_BUZZER, OUTPUT);
//  //digitalWrite(PIN_BUZZER, LOW);
//  tone(PIN_BUZZER, 1000);
//  delay(200);
//  noTone(PIN_BUZZER);
//  delay(100);
//  tone(PIN_BUZZER, 1000);
//  delay(200);
//  noTone(PIN_BUZZER);
//  delay(100);
//  tone(PIN_BUZZER, 1000);
//  delay(200);
//  noTone(PIN_BUZZER);
//  delay(100);
//  tone(PIN_BUZZER, 1000);
//  delay(200);
//  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, INPUT);

  
  // config for IMU, start I2C bus and falling edge data ready interrupt pin
  //attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuISR, FALLING);
  pinMode(PIN_IMU_INT, INPUT);
  pinMode(PIN_IMU_FSYNC, OUTPUT);
  digitalWrite(PIN_IMU_FSYNC, LOW);
  Wire.begin();
  Wire.setClock(400000);

  snoozeSPI.setClockPin(13);

  // configure teensy snooze library classes
  //timer.setTimer(60000); // sleep one minute at a time

  // configure interrupt to wake functionality for the sleep driver
  // on the pin that connects to the capacitive sense pin, we want to wake from sleep
  // when this pin goes low, representing separation of the KREM from the outside of the capsule
  pinMode(2, INPUT_PULLUP);
  snoozeDigital.pinMode(2, INPUT_PULLUP, RISING);//pin, mode, type

  delay(2000);
  

  // start threads, the '1' argument has no purpose, third arg is stack size (default is 1k)
  tid_sd       = threads.addThread(sd_thread,      1, 8192);
  tid_telem    = threads.addThread(telem_thread,   1, 4096);
  tid_tc       = threads.addThread(tc_thread,      1, 4096);
  tid_acc      = threads.addThread(acc_thread,     1, 8192);
  tid_iridium  = threads.addThread(iridium_thread, 1, 16000);
  tid_imu      = threads.addThread(imu_thread,     1, 8192);
  tid_compress = threads.addThread(compress_thread,1, 100000);
  tid_cap      = threads.addThread(cap_thread,     1, 2048);

  tid_sleep    = threads.addThread(sleep_thread,   1, 4096);

  // start the radio thread if its powered on and enabled in config
  // TODO: this thread doesn't like being started first?????
  //   tried increasing the thread memory
  //   all radio spi transactions are done in a mutexf
  //   haven't tested absolute order against TC thread which is
  //   the only other thread using spi...
  if ( ISM_DEBUG ) {
    if ( digitalRead(PIN_ISM_PRESENT) ) {
      if (USBSERIAL_DEBUG) safePrintln("ISM: present");
      debugRadioPresent = true;
      tid_command  = threads.addThread(command_thread, 1, 4096);
      tid_radio = threads.addThread(radio_thread, 1, 20000);
    } else {
      debugRadioPresent = false;
      if (USBSERIAL_DEBUG) safePrintln("ISM: NOT PRESENT");
    }
  } else {
    debugRadioPresent = false;
  }

  threads.setSliceMillis(5);
}

void loop() {

}
