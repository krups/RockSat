/* 
 * Rocksat 2022 Firmware
 * 
 * 
 * Matt Ruffner, University of Kentucky Fall 2021
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_MCP9600.h>
#include <ArduinoNmeaParser.h>
#include <FreeRTOS_SAMD51.h>
#include <SerialTransfer.h>
#include <IridiumSBD.h>
#include <semphr.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include "wiring_private.h"

#include "delay_helpers.h" // rtos delay helpers
#include "config.h"        // project wide defs
#include "packet.h"        // packet definitions
#include "commands.h"      // command spec
#include "pins.h"                  // CDH system pinouts

/* Serial 2
 The GPIO on the SAMD processor support multipler serial protocols
 on various compbinations of pins for more flexibility
 this configures pin 13 as RX and 12 as TX of SERCOM3 in hardware.
 These handlers then in turn trigger the handler for the Serial2 object
 in Arduino
 */
Uart Serial2( &sercom3, 13, 12, SERCOM_RX_PAD_1, UART_TX_PAD_0 ) ;
void SERCOM3_0_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_3_Handler()
{
  Serial2.IrqHandler();
}

// Serial3
Uart Serial3 (&sercom4, A3, A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM4_0_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM4_1_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM4_2_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM4_3_Handler()
{
  Serial3.IrqHandler();
}


// rename serial ports
#define SERIAL      Serial  // debug serial (USB) all uses should be conditional on DEBUG define
#define SERIAL_IRD  Serial2 // to iridium modem
#define SERIAL_LOR  Serial3 // to telemetry radio

// TC to digital objects
Adafruit_MCP9600 mcps[18];

// I2C addresses on MCP breakout board, use this to assign proper channels
// need to specify per breakout if addressing not consistent between boards
const uint8_t MCP_ADDRS[6] = {0x62, 0x61, 0x60, 0x63, 0x64, 0x67};

// freertos task handles
TaskHandle_t Handle_tcTask; // data receive from TPM subsystem task
TaskHandle_t Handle_logTask; // sd card logging task
//TaskHandle_t Handle_gpsTask; // gps data receive task
TaskHandle_t Handle_irdTask; // iridium transmission task
TaskHandle_t Handle_parTask; // parachute deployment task
//TaskHandle_t Handle_barTask; // barometric sensor task
TaskHandle_t Handle_radTask; // telem radio task handle
//TaskHandle_t Handle_monitorTask; // debug running task stats over uart task

// freeRTOS queues
// tasks write direct to SD buffer
/*QueueHandle_t qTmpData; // temperature or heat flux data to be logged
QueueHandle_t qPrsData; // pressure data to be logged
QueueHandle_t qGgaData; // GGA GPS fix data to be logged
QueueHandle_t qRmcData; // RMC GPS data to be logged
QueueHandle_t qBarData; // barometric pressure data
*/


// freeRTOS semaphores
SemaphoreHandle_t tpmSerSem; // data from CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore
SemaphoreHandle_t gpsSerSem; // gps serial port acces
SemaphoreHandle_t irdSerSem; // iridium serial semaphore
SemaphoreHandle_t depSem; // deployment status protector
SemaphoreHandle_t sigSem; // iridium signal protector
SemaphoreHandle_t wbufSem; // SD buffer write semaphore
SemaphoreHandle_t ledSem; // neopixel sepaphore

uint8_t rbuf[RBUF_SIZE];
char sbuf[SBUF_SIZE];
char printbuf[100];


static uint8_t logBuf1[LOGBUF_SIZE];
static uint8_t logBuf2[LOGBUF_SIZE];
uint32_t endWriteOffset1 = 0; // how many bytes short of LOGBUF_SIZE was the data length of the last block write
uint32_t endWriteOffset2 = 0;
uint32_t logBuf1Pos = 0, // current write index in buffer1
         logBuf2Pos = 0; // current write index in buffer2
uint8_t activeLog = 0;   // which buffer should be used fo writing
volatile bool gb1Full = false, gb2Full = false;

bool globalDeploy = false;
bool irSig = 0;

// GPS update callbacks
#ifdef USE_GPS
//void onRmcUpdate(nmea::RmcData const);
//void onGgaUpdate(nmea::GgaData const);

// GPS parser object
//ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);
#endif


#ifdef USE_LEDS

Adafruit_NeoPixel led(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void ledError(int type) {
  switch (type) {
  case ERR_BOOT:
    led.setPixelColor(0, led.Color(150, 0, 0));
    break;
  case ERR_2:
    led.setPixelColor(0, led.Color(150, 0, 150));
    break;
  case ERR_3:
    led.setPixelColor(0, led.Color(150, 150, 0));
    break;
  case ERR_4:
    led.setPixelColor(0, led.Color(0, 150, 150));
    break;
  }

  led.show();
}

void ledOk() {
  led.setPixelColor(0, led.Color(0, 150, 0));
}
#endif


#ifdef USE_SPECTROMETER
uint16_t specData[NUM_SPEC_CHANNELS];
#endif

// IRIDIUM MODEM OBJECT
IridiumSBD modem(SERIAL_IRD);

// Parachute servo setup
Servo linAct1, linAct2;

/*
void onRmcUpdate(nmea::RmcData const rmc)
{
  #ifdef DEBUG_GPS
  //if (rmc.is_valid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeRmc(rmc, SERIAL);  
      xSemaphoreGive( dbSem );
    }
  //}
  #endif

  if( xQueueSend( qRmcData, ( void * ) &rmc, ( TickType_t ) 200 ) != pdTRUE ) {
    // Failed to post the message, even after 100 ticks.
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("ERROR: failed to put rmc data into queue");
      xSemaphoreGive( dbSem );
    }
    #endif
  }
}

void writeRmc(nmea::RmcData const rmc, Stream &pipe)
{
  pipe.print("RMC ");

  if      (rmc.source == nmea::RmcSource::GPS)     pipe.print("GPS");
  else if (rmc.source == nmea::RmcSource::GLONASS) pipe.print("GLONASS");
  else if (rmc.source == nmea::RmcSource::Galileo) pipe.print("Galileo");
  else if (rmc.source == nmea::RmcSource::GNSS)    pipe.print("GNSS");

  pipe.print(" ");
  pipe.print(rmc.time_utc.hour);
  pipe.print(":");
  pipe.print(rmc.time_utc.minute);
  pipe.print(":");
  pipe.print(rmc.time_utc.second);
  pipe.print(".");
  pipe.print(rmc.time_utc.microsecond);

  if (rmc.is_valid)
  {
    pipe.print(" : LON ");
    pipe.print(rmc.longitude, 5);
    pipe.print(" ° | LAT ");
    pipe.print(rmc.latitude, 5);
    pipe.print(" ° | VEL ");
    pipe.print(rmc.speed);
    pipe.print(" m/s | HEADING ");
    pipe.print(rmc.course);
    pipe.print(" °");
  }

  pipe.println();
}

// write formatted gps string to stream i.e. file or serial port
void writeGga(nmea::GgaData const gga, Stream &pipe)
{
  pipe.print("GGA ");

  if      (gga.source == nmea::GgaSource::GPS)     pipe.print("GPS");
  else if (gga.source == nmea::GgaSource::GLONASS) pipe.print("GLONASS");
  else if (gga.source == nmea::GgaSource::Galileo) pipe.print("Galileo");
  else if (gga.source == nmea::GgaSource::GNSS)    pipe.print("GNSS");

  pipe.print(" ");
  pipe.print(gga.time_utc.hour);
  pipe.print(":");
  pipe.print(gga.time_utc.minute);
  pipe.print(":");
  pipe.print(gga.time_utc.second);
  pipe.print(".");
  pipe.print(gga.time_utc.microsecond);

  if (gga.fix_quality != nmea::FixQuality::Invalid)
  {
    pipe.print(" : LON ");
    pipe.print(gga.longitude, 5);
    pipe.print(" ° | LAT ");
    pipe.print(gga.latitude, 5);
    pipe.print(" ° | Num Sat. ");
    pipe.print(gga.num_satellites);
    pipe.print(" | HDOP =  ");
    pipe.print(gga.hdop);
    pipe.print(" m | Altitude ");
    pipe.print(gga.altitude);
    pipe.print(" m | Geoidal Separation ");
    pipe.print(gga.geoidal_separation);
    pipe.print(" m");
  }

  pipe.println();
}

// hopefully not called from an interrup by the GPS parser library
// this function takes the gps fix data and pushes it into the logging queue
void onGgaUpdate(nmea::GgaData const gga)
{
  #ifdef DEBUG_GPS
  if (gga.fix_quality != nmea::FixQuality::Invalid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeGga(gga, SERIAL);
      xSemaphoreGive( dbSem );
    }
  }
  #endif

  if( xQueueSend( qGgaData, ( void * ) &gga, ( TickType_t ) 200 ) != pdTRUE ) {
    // Failed to post the message, even after 100 ticks. 
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      SERIAL.println("ERROR: failed to put gga data into queue");
      xSemaphoreGive( dbSem );
    }
    #endif
  }
}
*/

// thread safe copy of src boolean to dest boolean.
// dont use do assign a direct value to dest, use safeAssign for that
bool safeUpdate(bool &dest, bool &src, SemaphoreHandle_t &m) {
  // wait100 ticks to do the assignment
  if ( xSemaphoreTake( m, ( TickType_t ) 100 ) == pdTRUE ) {
    dest = src;
    xSemaphoreGive( m ); //  free the mutex
    return true; // assignment success
  } else {
    return false;  // if no assignment was made
  }
}

// thread safe assignement of a direct truth val to dest
// pass this function a direct truth value
bool safeAssign(bool &dest, bool src, SemaphoreHandle_t &m) {
  // wait100 ticks to do the assignment
  if ( xSemaphoreTake( m, ( TickType_t ) 100 ) == pdTRUE ) {
    dest = src;
    xSemaphoreGive( m ); //  free the mutex
    return true; // assignment success
  } else {
    return false;  // if no assignment was made
  }
}

// thread safe global read
bool safeRead(bool &src, SemaphoreHandle_t &m) {
  bool ret = false;
  // wait100 ticks to do the assignment
  if ( xSemaphoreTake( m, ( TickType_t ) 100 ) == pdTRUE ) {
    ret = src;
    xSemaphoreGive( m ); //  free the mutex
  }
  return ret;
}



#ifdef USE_GPS
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * GPS serial monitoring thread
*/
/*
static void gpsThread( void *pvParameters )
{
  #ifdef DEBUG_GPS
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("GPS thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  while(1) {
    if ( xSemaphoreTake( gpsSerSem, ( TickType_t ) 100 ) == pdTRUE ) {
      while (SERIAL_GPS.available()) {
        parser.encode((char)SERIAL_GPS.read());
      }
      xSemaphoreGive( gpsSerSem );
    }
    
    myDelayMs(20);
  }
  
  vTaskDelete( NULL );  
}
*/
#endif


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * Iridium thread
*/
static void irdThread( void *pvParameters )
{
  bool fix_valid = false;
  char buf[330];
  int mSq = 0, irerr; // signal quality, modem operation return code
  unsigned long lastSignalCheck = 0, lastPacketSend = 0;
  bool gpsAvailable = false;
  nmea::RmcData rmcData;
  
  sprintf(buf, "No GPS fix yet.");
  
  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Iridium thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  myDelayMs(1000); // give modem time to power up
    
  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Iridium thread: trying to start modem!...");
    xSemaphoreGive( dbSem );
  }
  #endif
    
  // init the iridium library and check signal strength
  irerr = modem.begin();
  
  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("IRIDIUM: called modem.begin()");
    xSemaphoreGive( dbSem );
  }
  #endif
    
  
  while( irerr != ISBD_SUCCESS ){
    #ifdef DEBUG_IRD
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: Begin failed: error " + String(irerr));
      xSemaphoreGive( dbSem );
    }
    #endif
    
    if( irerr == ISBD_NO_MODEM_DETECTED ){
      #ifdef DEBUG_IRD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.println("IRIDIUM: No modem detected: check wiring. ");
        xSemaphoreGive( dbSem );
      }
      #endif
    }
    
    irerr = modem.begin();
    
    myDelayMs(1000);
  }
  
  #ifdef DEBUG_IRD
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("IRIDIUM: modem initialized!");
    xSemaphoreGive( dbSem );
  }
  #endif
    
  
  // Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  irerr = modem.getSignalQuality(mSq);
  if( irerr != ISBD_SUCCESS ){
    
    #ifdef DEBUG_IRD
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: SignalQuality failed: error " + String(irerr));
      //syslog("IRIDIUM: failed to get first signal strength reading");
      //TODO: error handling
      //return;
      xSemaphoreGive( dbSem );
    }
    #endif
    
    
  } else {
    #ifdef DEBUG_IRD
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: first signal strength: " + String(mSq));
      //syslog("IRIDIUM: failed to get first signal strength reading");
      //TODO: error handling
      //return;
      xSemaphoreGive( dbSem );
    }
    #endif
  }
  
  //
  // MAIN TASK LOOP
  //
  while(1) {
    // handle a thread asking to send a packet, also periodically check the signal strength
    
    // periodically check the signal strength
    if( xTaskGetTickCount() - lastSignalCheck > CHECK_SIGNAL_PERIOD ){
      irerr = modem.getSignalQuality(mSq);
      if( irerr != ISBD_SUCCESS ){
        
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: get SignalQuality failed: error " + String(irerr));
          //syslog("IRIDIUM: failed to get first signal strength reading");
          //TODO: error handling
          //return;
          xSemaphoreGive( dbSem );
        }
        #endif 
      } else {
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: signal strength: " + String(mSq));
          //syslog("IRIDIUM: failed to get first signal strength reading");
          //TODO: error handling
          //return;
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      
      if ( xSemaphoreTake( sigSem, ( TickType_t ) 100 ) == pdTRUE ) {
        irSig = mSq;
        xSemaphoreGive( sigSem );
      }
      
      lastSignalCheck = xTaskGetTickCount();
    }
    
    
    // 
    // IS IT TIME TO SEND A PACKKAGE??
    if( xTaskGetTickCount() - lastPacketSend > IRIDIUM_PACKET_PERIOD && 
        (mSq > 0) && 
        SEND_PACKETS ){
      
      #ifdef DEBUG_IRD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: sending packet");
      xSemaphoreGive( dbSem );
      }
      #endif

      // TODO: fill the buff with compressed data
      
      irerr = modem.sendSBDText(buf);
            
      if (irerr != ISBD_SUCCESS) { // sending failed
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: failed to send packet :( error " + String(irerr));
          xSemaphoreGive( dbSem );
        }
        #endif 
      }
      else { // send success
        #ifdef DEBUG_IRD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("IRIDIUM: successfully sent a packet!!!!!!");
          xSemaphoreGive( dbSem );
        }
        #endif 
        // only update lastPacketSned timestamp if we were successful so that
        // the modem will try again asap
        lastPacketSend = xTaskGetTickCount();
      }
    }
    
    myDelayMs(50);
    
  } // end task loop
  
  vTaskDelete( NULL );  
}

/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * TPM Serial monitoring thread and helpers
*/

/* safely read from an MCP device
 *
*/
float readMCP(int id) {
  float hot=0.0;//ambient=0.0,adcval=0.0;

  if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {

      hot = mcps[id].readThermocouple();

      xSemaphoreGive( i2c1Sem );
  }

  return hot;
}

// initialize an MCP device
// made to run before task scheduler is started
bool initMCP(int id) {
  bool ok = true;
  uint8_t addr = MCP_ADDRS[id];
  uint8_t tries = 0;
  uint8_t max_tries = 100;

  #if DEBUG
  SERIAL.print("Starting MCP #");SERIAL.print(id);
  #endif

  delay(10);

  while ( !mcps[id].begin(addr) && tries < max_tries) {
    delay(10);
    tries++;
  }

  if( tries == max_tries) {
    ok = false;
    #if DEBUG
    SERIAL.println("Sensor not found. Check wiring!");
    #endif
  } else {
    #if DEBUG
    SERIAL.println("  Found MCP9600!");
    #endif
  }

  mcps[id].setADCresolution(MCP9600_ADCRESOLUTION_14);
  /*
  SERIAL.print("ADC resolution set to ");
  switch (mcps[id].getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   SERIAL.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   SERIAL.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   SERIAL.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   SERIAL.print("12"); break;
  }
  SERIAL.println(" bits");
  */

  mcps[id].setThermocoupleType(MCP9600_TYPE_K);
  /*SERIAL.print("Thermocouple type set to ");
  switch (mcps[id].getThermocoupleType()) {
    case MCP9600_TYPE_K:  SERIAL.print("K"); break;
    case MCP9600_TYPE_J:  SERIAL.print("J"); break;
    case MCP9600_TYPE_T:  SERIAL.print("T"); break;
    case MCP9600_TYPE_N:  SERIAL.print("N"); break;
    case MCP9600_TYPE_S:  SERIAL.print("S"); break;
    case MCP9600_TYPE_E:  SERIAL.print("E"); break;
    case MCP9600_TYPE_B:  SERIAL.print("B"); break;
    case MCP9600_TYPE_R:  SERIAL.print("R"); break;
  }
  //SERIAL.println(" type");
  */

  // TODO: what is the default filter coefficient
  // https://cdn.sparkfun.com/assets/9/0/b/0/3/MCP9600_Datasheet.pdf
  mcps[id].setFilterCoefficient(1);
  //SERIAL.print("Filter coefficient value set to: ");
  //SERIAL.println(mcps[id].getFilterCoefficient());

  #ifdef DEBUG
  SERIAL.println("started mcp");
  #endif

  mcps[id].enable(true);

  return ok;
}

/*
void safeKick() {
  if ( xSemaphoreTake( dogSem, ( TickType_t )  0) == pdTRUE ) {
   Watchdog.reset();
   xSemaphoreGive( dogSem );
  }
}*/

// thread definition
static void tcThread( void *pvParameters )
{

  float current_temps[NUM_TC_CHANNELS];
  unsigned long lastDebug = 0;
  unsigned long lastRead = 0;

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("MCP thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {
    if( xTaskGetTickCount() - lastRead > 500 ){
      lastRead = xTaskGetTickCount();
      //safeKick();

      // assign tc temps from MCP objects to local vars
      for( int i=0; i<NUM_TC_CHANNELS; i++ ){
        current_temps[i] = readMCP(i);
        //myDelayMs(5);
        //safeKick();
      }

      #ifdef DEBUG_TC
      if( xTaskGetTickCount() - lastDebug > 1000 ){
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          // print tc temps over serial
          SERIAL.print("TC temps: ");
          for( int i=0; i<NUM_TC_CHANNELS; i++ ){
            SERIAL.print(current_temps[i]); if(i<NUM_TC_CHANNELS-1) SERIAL.print(", ");
          }
          SERIAL.println();
          xSemaphoreGive( dbSem );
        }
        lastDebug = xTaskGetTickCount();
      }
      #endif

      // build data struct to send over serial
      tc_t data;
      data.t = xTaskGetTickCount();
      for( int i=0; i<NUM_TC_CHANNELS; i++ ){
        data.data[i] = current_temps[i];
      }

      // try to write the tc data to the SD log buffer that is available
    }

    taskYIELD();
  }

  vTaskDelete( NULL );
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * sd card logging thread
*/
static void logThread( void *pvParameters )
{
  static bool ready = false;
  bool b1full = false, b2full = false;
  uint32_t written = 0;
  File logfile;
  
  #ifdef DEBUG_LOG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("sd logging thread thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  char* filename = LOGFILE_NAME;
  
  // INIT CARD
  while (!SD.begin(PIN_SD_CS)) {
    #if DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.println("ERROR: sd logging thread couldn't init sd card");
      xSemaphoreGive( dbSem );
    }
    #endif
    ready = false;
    myDelayMs(1000);
  } 
  //else {
    #if DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.println("SD CARD INIT OK");
      xSemaphoreGive( dbSem );
    }
    #endif
    ready = true;
  //}
  
  
  // CREATE UNIQUE FILE NAMES (UP TO 100)
  for( uint8_t i=0; i < 100; i++) {
    filename[3] = '0' + i/10;
    filename[4] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  // main thread loop
  // check if a buffer is full and if it is switch the active buffer that the threads dump data to to
  // be the other buffern and start writing this one (ping pong system)
  while(1) {

    if( !gb1Full && !gb2Full ){
      taskYIELD();
      continue;
    }

    // open log file for all telem (single log file configuration)
    logfile = SD.open(filename, FILE_WRITE);

    // if we could not open the file
    if( ! logfile ) {
      #if DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.print("ERROR: sd logging thread couldn't open logfile for writing: ");
        Serial.println(filenames[0]);
        xSemaphoreGive( dbSem );
      }
      #endif
    }

    // we assume we can open it and start writing the buffer
    // check if one of the buffers is full and lets write it
    if( gb1Full ){

      logfile.seek(endWriteOffset1);

      written = logfile.write(logBuf1, LOGBUF_SIZE);

      #if DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.print("SD: wrote  ");
        Serial.print(written);
        Serial.print("/");
        Serial.print(LOGBUF_SIZE);
        Serial.println(" bytes from logBuf1");
        xSemaphoreGive( dbSem );
      }
      #endif

      // this would omit leaving gaps in the log file, but then we won't be writing
      // on nice block boundaries within the SD card
      //endWriteOffset1 += logBuf1Pos;

      // this method leaves blocks of zeros at the end of each
      endWriteOffset1 += LOGBUF_SIZE;

      // clean the buffer so we are gauranteed to see zeros where there hasn't
      // been recetn data written to
      memset(logBuf1, 0, LOGBUF_SIZE);

    }

    // check the other buffer
    if( gb2Full ){

      logfile.seek(endWriteOffset2);

      written = logfile.write(logBuf2, LOGBUF_SIZE);

      #if DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.print("SD: wrote  ");
        Serial.print(written);
        Serial.print("/");
        Serial.print(LOGBUF_SIZE);
        Serial.println(" bytes from logBuf2");
        xSemaphoreGive( dbSem );
      }
      #endif

      // this would omit leaving gaps in the log file, but then we won't be writing
      // on nice block boundaries within the SD card
      //endWriteOffset2 += logBuf2Pos;

      // this method leaves blocks of zeros at the end of each
      endWriteOffset2 += LOGBUF_SIZE;

      // clean the buffer so we are gauranteed to see zeros where there hasn't
      // been recetn data written to
      memset(logBuf2, 0, LOGBUF_SIZE);

    }

    // done writing to this file
    logfile.close();

    taskYIELD();
    //myDelayMs(10);
  }
  
  vTaskDelete( NULL );  
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * parachute deployment thread
 * monitor gps fixes and more importantly pressure data
*/
static void parThread( void *pvParameters )
{
  bool deployed = false;

  #ifdef DEBUG_PAR
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("parachute deployment thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  while(1) {
    // peek at gga and pressure queues to see if activation criteria have been met
    
    myDelayMs(50);
    
    if (deployed) continue;
    
    // TODO: implement parachute logic

    if ( /* received deploy over radio */ false ) {
      deployed = true;
    }
    
    if( deployed ){
      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("{INFO} !!! parachute deployment !!!");
        Serial.println("{INFO} !!! parachute deployment !!!");
        Serial.println("{INFO} !!! parachute deployment !!!");
        xSemaphoreGive( dbSem );
      }
      #endif
      
      if ( xSemaphoreTake( depSem, ( TickType_t ) portMAX_DELAY ) == pdTRUE ) {
        globalDeploy = true;
        xSemaphoreGive( depSem );
      }
      
      // now start paracture deployment sequence
      // first trigger c02 servo
      
      linAct1.write(ACT_POS_ACT);
      linAct2.write(ACT_POS_ACT);
    }
    
  }
  
  vTaskDelete( NULL );  
}



/**********************************************************************************
 *  Radio task
*/
static void radThread( void *pvParameters )
{

  unsigned long lastSendTime = 0;
  tlm_t dataOut;
  nmea::GgaData ggaData;
  nmea::RmcData rmcData;
  tc_t tmpData;
  spec_t specData;
  
    /* 0: waiting for +OK from reset
     1: configuring address
     2:   waiting for address config +OK
     3: configuring network id
     4:   waiting for network id config +OK
     5: configuring rf params
     6:   waiting for rf param config +OK
     7: ready
  */
  int state = 0;  
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("Radio thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  digitalWrite(PIN_LORA_RST, LOW);
  myDelayMs(100);
  digitalWrite(PIN_LORA_RST, HIGH);
  
    
  //SERIAL_LOR.print("AT+RESET\r\n");
  //SERIAL_LOR.flush();
  
  #ifdef DEBUG_RAD_VERBOSE
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("LORA: Reset radio");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  
  memset(rbuf, 0, RBUF_SIZE);
  memset(sbuf, 0, SBUF_SIZE);
  
  #ifdef DEBUG_RAD_VERBOSE
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("LORA: zerod buffers");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  while(1) {
    // check the data source queue and see if there is something to send
    // in capsule firmware this will need to peek at many other queus to aggregate the needed data,
    // here we just spoof values in the struct and send it
    // only send once per second
    
    /*#ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.print("STATE = ");
      SERIAL.print(state);
      SERIAL.print(", LOR_RADIO.peek()= ");
      SERIAL.println(SERIAL_LOR.peek());
      xSemaphoreGive( dbSem );
    }
    #endif*/
    
    if( state == 1 ){
      int len = sprintf(sbuf, "AT+ADDRESS=1\r\n");
      SERIAL_LOR.write(sbuf, len);
      state = 2;
      #ifdef DEBUG_RAD_VERBOSE
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("LORA: sent at+address, setting state to 2");
        xSemaphoreGive( dbSem );
      }
      #endif
      //taskYIELD();
      myDelayMs(10);
    }
    else if( state == 3 ){
      int len = sprintf(sbuf, "AT+NETWORKID=1\r\n");
      SERIAL_LOR.write(sbuf, len);
      state = 4;
      #ifdef DEBUG_RAD_VERBOSE
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("LORA: sent at+networkid, setting state to 4");
        xSemaphoreGive( dbSem );
      }
      #endif
      myDelayMs(10);
      //taskYIELD();
    }
    else if( state == 5 ){
      int len = sprintf(sbuf, "AT+PARAMETER=10,7,1,7\r\n"); // less than 3km
      SERIAL_LOR.write(sbuf, len);
      state = 6;
      #ifdef DEBUG_RAD_VERBOSE
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.println("LORA: sent at+param, setting state to 6");
        xSemaphoreGive( dbSem );
      }
      #endif
      myDelayMs(10);
      //taskYIELD();
    }
    
    if( xTaskGetTickCount() - lastSendTime > TLM_SEND_PERIOD && state == 7){
      // fill up dataout structure with data to send over telem radio
      
      dataOut.t = xTaskGetTickCount(); 
      
      const int dataSize = sizeof(tlm_t);
      sprintf(sbuf, "AT+SEND=2,%d,", dataSize); // where 2 is the address
      int pre = strlen(sbuf);
         
      // TODO: can we embed binary data ?????
      // now copy binary data from struct to send buffer
      memcpy(&sbuf[pre], (void*)&dataOut, dataSize);
      
      sbuf[pre+dataSize] = '\r';
      sbuf[pre+dataSize+1] = '\n';
      sbuf[pre+dataSize+2] = 0;
      
      
      #ifdef DEBUG_RAD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.print("sending radio binary packet of size ");
        SERIAL.println(pre+dataSize+2);
        SERIAL.print("actual data was (bytes): ");
        SERIAL.println(dataSize);
        //SERIAL.println("DONE");
        xSemaphoreGive( dbSem );
      }
      #endif
      
      // send to lora module
      SERIAL_LOR.write(sbuf, pre+dataSize+2);
      //SERIAL_LOR.write(sbuf, pre);
      
      // update timestamp, not including data fill time from above lines
      lastSendTime = dataOut.t; 
      
      // go to state 8 so that we wait for a response
      state = 8;
      myDelayMs(100);
      //taskYIELD();
    }
    
  
    // handle incoming message from LORA radio
    // AT message from module
    bool eol = false;
    int pos = 0;
    bool timeout = false;
    unsigned long timeoutStart = 0;
    if( SERIAL_LOR.peek() == '+' ){
      
      #ifdef DEBUG_RAD_VERBOSE
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.println("saw a +");
        xSemaphoreGive( dbSem );
      }
      #endif

    
      unsigned long timeoutStart = xTaskGetTickCount();
      while(!eol && !timeout && pos < RBUF_SIZE-1) {
        if( SERIAL_LOR.available() ){
          rbuf[pos] = SERIAL_LOR.read();
          if( pos > 1 ){
            if( rbuf[pos]=='\n' && rbuf[pos-1]=='\r' ){
              memset(&rbuf[pos+1], 0, RBUF_SIZE-(pos+1));
              eol = true;
            }
          }
          
          if( pos++ >= RBUF_SIZE ){
            break;
          }
        }
        if( xTaskGetTickCount() - timeoutStart > RX_TIMEOUT_PERIOD ){
          memset(rbuf, 0, RBUF_SIZE);
          timeout = true;
        }
      }
    
    
      if( timeout ){
        #ifdef DEBUG_RAD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio rx timed out");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if (!timeout && eol) {
        #ifdef DEBUG_RAD_VERBOSE
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio got message!");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if( !timeout && !eol) {
        #ifdef DEBUG_RAD
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio receive buffer overrun!");
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      
      // if first byte is non-zero then we received data ( also check timeout and eol vars to be safe)
      // now process the data line received
      if( (rbuf[0] == '+') && !timeout && eol){
        
        
        int eqpos = 1;
        while( ( rbuf[eqpos] != '=' ) &&  ( eqpos < pos) ){
          eqpos++;
        }
        
        if( eqpos == pos ){
          #ifdef DEBUG_RAD
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            SERIAL.print("Received ");
            SERIAL.write(rbuf, pos);
            xSemaphoreGive( dbSem );
          }
          #endif
          
          if( state < 7 ){
            state ++;
            if( state == 7 ){
              #ifdef DEBUG_RAD_VERBOSE
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
                SERIAL.println("STATE = 7, successfully configured radio!");
                xSemaphoreGive( dbSem );
              }
              #endif
              taskYIELD();
            }
          } else if( state > 7 ){
            state = 7;
            #ifdef DEBUG_RAD_VERBOSE
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              SERIAL.println("STATE was 8, received +OK from a data send operation!");
              xSemaphoreGive( dbSem );
            }
            #endif
            taskYIELD(); 
          }
          
        } else {
          // found an '=', parse rest of message
          //
          // TODO: parse the packet and fill the rxtlm_t struct with data
          //       and send to the logging and display queues
          // 
          
          // check if its a receive message
          if( rbuf[0]=='+' &&
              rbuf[1]=='R' &&
              rbuf[2]=='C' &&
              rbuf[3]=='V'){
            
            // parse data
            // example rx string: +RCV=50,5,HELLO,-99,40
            const char *comma = ",";
            char *token;
            char *data;
            int rssi;
            int snr;
            int addr = -1;
            int datalen = -1;
            
            // parse target address
            token = strtok((char *) &rbuf[5], comma);
            addr = atoi(token);
            
            // extract data length
            token = strtok(NULL, comma);
            datalen = atoi(token);
            
            data = (char *)&rbuf[5];
            while( *(++data) != ',');
            while( *(++data) != ',');
            
            // get pointer to start of data 
            //data = strtok(NULL, comma);
            
            // get the rssi
            token = strtok((char *) &rbuf[8+datalen], comma);
            token = strtok(NULL, comma);
            rssi = atoi(token);
            
            // get the SNR
            token = strtok(NULL, comma);
            snr = atoi(token);
            
            
            #ifdef DEBUG
            int pblen = sprintf(printbuf, "Received %d bytes from address %d\n  rssi: %d, snr: %d\n", datalen, addr, rssi, snr);
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              SERIAL.write(printbuf, pblen);
              //SERIAL.write(data, datalen);
              xSemaphoreGive( dbSem );
            }
            #endif
            
            // check what the data was
            switch (data[0]) {
              case CMDID_DEPLOY_CHUTE:
                #if DEBUG
                if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
                  SERIAL.println("Got deploy drogue command");
                  xSemaphoreGive( dbSem );
                }
                #endif
                break;
            }
            
             
          }
        }
      }
    } 
    //taskYIELD();
    myDelayMs(100);
  }
  
  vTaskDelete( NULL );  
}




/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
void setup() {

  // servo control
  pinMode(PIN_CHUTE_ACT1, OUTPUT);
  pinMode(PIN_CHUTE_ACT2, OUTPUT);

  // attach servo
  linAct1.attach(PIN_CHUTE_ACT1);
  linAct2.attach(PIN_CHUTE_ACT2);
  linAct1.write(ACT_POS_HOME);
  linAct2.write(ACT_POS_HOME);

  #if DEBUG
  SERIAL.begin(115200); // init debug serial
  #endif
  
  delay(100);
  
  //delay(10);
  //SERIAL_TPM.begin(TPM_SERIAL_BAUD); // init serial to tpm subsystem
  //delay(10);
  //SERIAL_GPS.begin(9600); // init gps serial
  delay(10);
  SERIAL_IRD.begin(9600); // init iridium serial
  delay(10);
  SERIAL_LOR.begin(115200); // init LORA telemetry radio serial
  
  
  // Assign SERCOM functionality to enable 3 more UARTs
  pinPeripheral(A1, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);
  pinPeripheral(A2, PIO_SERCOM_ALT);
  pinPeripheral(A3, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);
  
  // reset pin for lora radio
  pinMode(PIN_LORA_RST, OUTPUT);
  
  // battery voltage divider 
  pinMode(PIN_VBAT, INPUT);
  
  delay(3000);
  
  #ifdef DEBUG
  SERIAL.println("Starting...");
  #endif

  // zero out log buffers
  memset(logBuf1, 0, LOGBUF_SIZE);
  memset(logBuf2, 0, LOGBUF_SIZE);


  // initialize all MCP9600 chips on the TC to digital breakout
  bool ok = true;
  for( int i=0; i<NUM_TC_CHANNELS; i++) {
    ok &= initMCP(i);
    delay(100);
    ledError(ERR_2);
    //Watchdog.reset();
  }
  if (!ok) {
    #ifdef USELEDS
    ledError();
    #endif

    #ifdef DEBUG
    SERIAL.println("failed to start all MCP devices");
    #endif
  }
  
  #ifdef DEBUG
  SERIAL.println("Created queues...");
  #endif
  
  // SETUP RTOS SEMAPHORES
  // setup cdh serial port smphr
  if ( tpmSerSem == NULL ) {
    tpmSerSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( tpmSerSem ) != NULL )
      xSemaphoreGive( ( tpmSerSem ) );  // make available
  }
  // setup debug serial log semaphore
  if ( dbSem == NULL ) {
    dbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( dbSem ) != NULL )
      xSemaphoreGive( ( dbSem ) );  // make available
  }
  // setup i2c port semaphore
  if ( i2c1Sem == NULL ) {
    i2c1Sem = xSemaphoreCreateMutex();  // create mutex
    if ( ( i2c1Sem ) != NULL )
      xSemaphoreGive( ( i2c1Sem ) );  // make available
  }
  // setup gps serial semaphore
  if ( gpsSerSem == NULL ) {
    gpsSerSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( gpsSerSem ) != NULL )
      xSemaphoreGive( ( gpsSerSem ) );  // make available
  }
  // setup iridium serial semaphore
  if ( irdSerSem == NULL ) {
    irdSerSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( irdSerSem ) != NULL )
      xSemaphoreGive( ( irdSerSem ) );  // make available
  }
  // setup DEPLOYMENT bool protector
  if ( depSem == NULL ) {
    depSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( depSem ) != NULL )
      xSemaphoreGive( ( depSem ) );  // make available
  }
  // setup iridium signal protector
  if ( sigSem == NULL ) {
    sigSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( sigSem ) != NULL )
      xSemaphoreGive( ( sigSem ) );  // make available
  }
  // setup write buffer sem
  if ( wbufSem == NULL ) {
    wbufSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( wbufSem ) != NULL )
      xSemaphoreGive( ( wbufSem ) );  // make available
  }
  // setup led sem
  if ( ledSem == NULL ) {
    ledSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( ledSem ) != NULL )
      xSemaphoreGive( ( ledSem ) );  // make available
  }

  #ifdef DEBUG
  SERIAL.println("Created semaphores...");
  #endif
  
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(tcThread, "TC Measurement", 512, NULL, tskIDLE_PRIORITY, &Handle_tcTask);
  xTaskCreate(logThread, "SD Logging", 1024, NULL, tskIDLE_PRIORITY, &Handle_logTask);
  //xTaskCreate(gpsThread, "GPS Reception", 512, NULL, tskIDLE_PRIORITY, &Handle_gpsTask);
  xTaskCreate(irdThread, "Iridium thread", 512, NULL, tskIDLE_PRIORITY, &Handle_irdTask);
  xTaskCreate(parThread, "Parachute Deployment", 512, NULL, tskIDLE_PRIORITY+2, &Handle_parTask);
  //xTaskCreate(barThread, "Capsule internals", 512, NULL, tskIDLE_PRIORITY, &Handle_barTask);
  xTaskCreate(radThread, "Telem radio", 1000, NULL, tskIDLE_PRIORITY, &Handle_radTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);
  
  ledOk();

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();
  
  // error scheduler failed to start
  while(1)
  {
	  SERIAL.println("Scheduler Failed! \n");
	  delay(1000);
  }
  
  
}

void loop() {
  // tasks!
}
