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
#include <Adafruit_BNO055.h>
#include <Adafruit_MCP9600.h>
#include <ArduinoNmeaParser.h>
#include <FreeRTOS_SAMD51.h>
#include <IridiumSBD.h>
#include <semphr.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include "wiring_private.h"

#include "H3LIS100.h"      // high g accel driver
#include "delay_helpers.h" // rtos delay helpers
#include "config.h"        // project wide defs
#include "packet.h"        // packet definitions
#include "commands.h"      // command spec
#include "pins.h"          // system pinouts
#include "serial_headers.h" // Headers for serial print
#include "brieflz.h"

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
#define SERIAL_IRD  Serial1 // to iridium modem
//#define SERIAL_LOR  Serial2 // lora debug
#define SERIAL_GPS  Serial2 // to GPS

// TC to digital objects
Adafruit_MCP9600 mcps[6];

// I2C addresses on MCP breakout board, use this to assign proper channels
// need to specify per breakout if addressing not consistent between boards
const uint8_t MCP_ADDRS[6] = {0x63, 0x64, 0x62, 0x60, 0x61, 0x67};

// freertos task handles
TaskHandle_t Handle_compTask; // compression task handle
TaskHandle_t Handle_tcTask; // data receive from TPM subsystem task
TaskHandle_t Handle_logTask; // sd card logging task
TaskHandle_t Handle_gpsTask; // gps data receive task
TaskHandle_t Handle_irdTask; // iridium transmission task
TaskHandle_t Handle_parTask; // parachute deployment task
TaskHandle_t Handle_imuTask; // imu and acc task
TaskHandle_t Handle_barTask; // barometric sensor task
TaskHandle_t Handle_radTask; // telem radio task handle
//TaskHandle_t Handle_monitorTask; // debug running task stats over uart task
TaskHandle_t Handle_specTask; // Spectrometer task

// freeRTOS semaphores
SemaphoreHandle_t tpmSerSem; // data from CDH semaphore
SemaphoreHandle_t dbSem; // serial debug logging (Serial)
SemaphoreHandle_t i2c1Sem; // i2c port 1 access semaphore
SemaphoreHandle_t gpsSerSem; // gps serial port acces
SemaphoreHandle_t irdSerSem; // iridium serial semaphore
SemaphoreHandle_t depSem; // deployment status protector
SemaphoreHandle_t irbSem; // iridium buffer protector
SemaphoreHandle_t sigSem; // iridium signal quality protector
SemaphoreHandle_t wbufSem; // SD buffer write semaphore
SemaphoreHandle_t ledSem; // neopixel sepaphore
SemaphoreHandle_t sdSem;

uint8_t rbuf[RBUF_SIZE];
char sbuf[SBUF_SIZE];

static uint8_t logBuf1[LOGBUF_BLOCK_SIZE];
static uint8_t logBuf2[LOGBUF_BLOCK_SIZE];
volatile uint32_t logBuf1Pos = 0, // current write index in buffer1
         logBuf2Pos = 0; // current write index in buffer2
volatile uint8_t activeLog = 1;   // which buffer should be used fo writing, 1 or 2
volatile bool gb1Full = false, gb2Full = false;

volatile bool globalDeploy = false;
volatile bool irSig = 0;
volatile bool packetReady = 0;
int gPacketSize = 0;
char gIrdBuf[SBD_TX_SZ];

// for compression thread
uint8_t c_buf[SBD_TX_SZ];      // Compressed buffer
uint8_t uc_buf[10*SBD_TX_SZ];  // Uncompressed buffer
uint32_t workmem[(1UL << (BLZ_HASH_BITS))]; // lookup table for compression

char filename[LOGFILE_NAME_LENGTH] = LOGFILE_NAME;

// include here to things are already defined
#include "sample_datfile.h"

// GPS update callbacks
void onRmcUpdate(nmea::RmcData const);
void onGgaUpdate(nmea::GgaData const);

// GPS parser object
ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);

// low g acc and gyro
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// capsule internal barometric pressure object
Adafruit_MPL3115A2 baro;

//  high g accel
H3LIS100 highg = H3LIS100(0x1234); // 0x1234 is an arbitrary sensor ID, not an I2C address

uint16_t data_1[NUM_SPEC_CHANNELS]; // Define an array for the data read by the spectrometer
uint16_t data_2[NUM_SPEC_CHANNELS]; // Define an array for the data read by the spectrometer

int multipliers_1[NUM_SPEC_CHANNELS] = {0}; // Define an array for the coefficients of the simpson's rule
int multipliers_2[NUM_SPEC_CHANNELS] = {0}; // Define an array for the coefficients of the simpson's rule

float const coeff = ((850.0 - 340.0) / (NUM_SPEC_CHANNELS - 1) / 3); // Initial coefficient for the simpson's rule

void readSpectrometer(uint8_t SPEC_TRG, uint8_t SPEC_ST, uint8_t SPEC_CLK, uint8_t SPEC_VIDEO, uint16_t *data)
{ // This is from the spec sheet of the spectrometer

    int delayTime = 1; // delay time

    // Start clock cycle and set start pulse to signal start
    digitalWrite(SPEC_CLK, LOW);
    myDelayUs(delayTime);
    digitalWrite(SPEC_CLK, HIGH);
    myDelayUs(delayTime);
    digitalWrite(SPEC_CLK, LOW);
    digitalWrite(SPEC_ST, HIGH);
    myDelayUs(delayTime);

    // Sample for a period of time
    for (int i = 0; i < 15; i++)
    {

        digitalWrite(SPEC_CLK, HIGH);
        myDelayUs(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        myDelayUs(delayTime);
    }

    // Set SPEC_ST to low
    digitalWrite(SPEC_ST, LOW);

    // Sample for a period of time
    for (int i = 0; i < 85; i++)
    {

        digitalWrite(SPEC_CLK, HIGH);
        myDelayUs(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        myDelayUs(delayTime);
    }

    // One more clock pulse before the actual read
    digitalWrite(SPEC_CLK, HIGH);
    myDelayUs(delayTime);
    digitalWrite(SPEC_CLK, LOW);
    myDelayUs(delayTime);

    // Read from SPEC_VIDEO
    for (int i = 0; i < NUM_SPEC_CHANNELS; i++)
    {

        data[i] = analogRead(SPEC_VIDEO);

        digitalWrite(SPEC_CLK, HIGH);
        myDelayUs(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        myDelayUs(delayTime);
    }

    // Set SPEC_ST to high
    digitalWrite(SPEC_ST, HIGH);

    // Sample for a small amount of time
    for (int i = 0; i < 7; i++)
    {

        digitalWrite(SPEC_CLK, HIGH);
        myDelayUs(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        myDelayUs(delayTime);
    }

    digitalWrite(SPEC_CLK, HIGH);
    myDelayUs(delayTime);
}

void printData(uint16_t *data, float result[2], int id)
{ // Print the NUM_SPEC_CHANNELS data, then print the current time, the current color, and the number of channels.
    Serial.print(id);
    Serial.print(',');
    for (int i = 0; i < NUM_SPEC_CHANNELS; i++)
    {
        //    data_matrix(i) = data[i];
        Serial.print(data[i]);
        Serial.print(',');
    }
    Serial.print(result[0] + result[1]);
    Serial.print(',');
    Serial.print(xTaskGetTickCount());
    Serial.print(',');
    Serial.print(NUM_SPEC_CHANNELS);

    Serial.print("\n");
}

void calcIntLoop(uint16_t *data, int *multipliers, float* result)
{
    for (int i = 0; i < 88; i++)
    { // Calculate each value for the simpson's rule.
        result[0] += multipliers[i] * data[i];
    }
    result[0] *= coeff;

    for (int i = 88; i < NUM_SPEC_CHANNELS; i++)
    { // Calculate each value for the simpson's rule.
        result[1] += multipliers[i] * data[i];
    }
    result[1] *= coeff;
}

static void specThread(void *pvParameters)
{
  float res1[2];
  float res2[2];
    while (1)
    {
      // take reading from spectrometer, disabling interrupts to ensure proper timing
      taskENTER_CRITICAL();
      readSpectrometer(PIN_SPEC1_TRIG, PIN_SPEC1_START, PIN_SPEC1_CLK, PIN_SPEC1_VIDEO, data_1);
      taskEXIT_CRITICAL();

      calcIntLoop(data_1, multipliers_1, res1);

      #ifdef DEBUG_SPEC
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        //printData(data_1, res1, SPECTROMETER_1_SERIAL);
        Serial.print("Spectro 1 Res: ");
        Serial.print(res1[0]);
        Serial.print("/");
        Serial.println(res1[1]);
        xSemaphoreGive( dbSem );
      }
      #endif

      // take reading from second spectrometer
      taskENTER_CRITICAL();
      readSpectrometer(PIN_SPEC2_TRIG, PIN_SPEC2_START, PIN_SPEC2_CLK, PIN_SPEC2_VIDEO, data_2);
      taskEXIT_CRITICAL();

      calcIntLoop(data_2, multipliers_2, res2);

      #ifdef DEBUG_SPEC
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        printData(data_2, res2, SPECTROMETER_2_SERIAL);
        Serial.print("Spectro 2 Res: ");
        Serial.print(res2[0]);
        Serial.print("/");
        Serial.println(res2[1]);
        xSemaphoreGive( dbSem );
      }
      #endif

      myDelayMs(1000);
    }
}

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
  default:
    led.setPixelColor(0, led.Color(100, 100, 100));
  }
  led.show();
}

void ledOk() {
  led.setPixelColor(0, led.Color(0, 150, 0));
  led.show();
}

// IRIDIUM MODEM OBJECT
IridiumSBD modem(SERIAL_IRD);

// Parachute servo setup
Servo linAct;


void onRmcUpdate(nmea::RmcData const rmc)
{
  rmc_t data;

  #ifdef DEBUG_GPS
  //if (rmc.is_valid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeRmc(rmc, SERIAL);
      xSemaphoreGive( dbSem );
    }
  //}
  #endif

  // TODO: write data to log buffer
  data.t = xTaskGetTickCount();
  data.time[0] = rmc.time_utc.hour;
  data.time[1] = rmc.time_utc.minute;
  data.time[2] = rmc.time_utc.second;
  data.time[3] = rmc.time_utc.microsecond;
  data.lat = rmc.latitude;
  data.lon = rmc.longitude;
  data.speed = rmc.speed;
  data.course = rmc.course;

  // try to write the gps rmc data to the SD log buffer
  if ( xSemaphoreTake( wbufSem, ( TickType_t ) 10 ) == pdTRUE ) {

    if( activeLog == 1 ){
      // is this the last data we will put in before considering the
      // buffer full?
      logBuf1[logBuf1Pos++] = PTYPE_RMC; // set packet type byte
      memcpy(&logBuf1[logBuf1Pos], &data, sizeof (rmc_t));
      logBuf1Pos += sizeof (rmc_t);
      if( logBuf1Pos >= LOGBUF_FULL_SIZE ){
        activeLog = 2;
        logBuf1Pos = 0;
        gb1Full = true;
      }
    } else if( activeLog == 2 ){
      // is this the last data we will put in before considering the
      // buffer full?
      logBuf2[logBuf2Pos++] = PTYPE_RMC; // set packet type byte
      memcpy(&logBuf2[logBuf2Pos], &data, sizeof (rmc_t));
      logBuf2Pos += sizeof (rmc_t);
      if( logBuf2Pos >= LOGBUF_FULL_SIZE ){
        activeLog = 1;
        logBuf2Pos = 0;
        gb2Full = true;
      }
    }

    xSemaphoreGive( wbufSem );
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
  gga_t data;

  #ifdef DEBUG_GPS
  //if (gga.fix_quality != nmea::FixQuality::Invalid) {
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 200 ) == pdTRUE ) {
      writeGga(gga, Serial);
      xSemaphoreGive( dbSem );
    }
  //}
  #endif

  // TODO: write data to log buffer
  data.t = xTaskGetTickCount();
  data.time[0] = gga.time_utc.hour;
  data.time[1] = gga.time_utc.minute;
  data.time[2] = gga.time_utc.second;
  data.time[3] = gga.time_utc.microsecond;
  data.lat = gga.latitude;
  data.lon = gga.longitude;
  data.hdop = gga.hdop;
  data.alt = gga.altitude;

  // try to write the gps data to the SD log buffer
  if ( xSemaphoreTake( wbufSem, ( TickType_t ) 10 ) == pdTRUE ) {

    if( activeLog == 1 ){
      // is this the last data we will put in before considering the
      // buffer full?
      logBuf1[logBuf1Pos++] = PTYPE_GGA; // set packet type byte
      memcpy(&logBuf1[logBuf1Pos], &data, sizeof (gga_t));
      logBuf1Pos += sizeof (gga_t);
      if( logBuf1Pos >= LOGBUF_FULL_SIZE ){
        activeLog = 2;
        logBuf1Pos = 0;
        gb1Full = true;
      }
    } else if( activeLog == 2 ){
      // is this the last data we will put in before considering the
      // buffer full?
      logBuf2[logBuf2Pos++] = PTYPE_GGA; // set packet type byte
      memcpy(&logBuf2[logBuf2Pos], &data, sizeof (gga_t));
      logBuf2Pos += sizeof (gga_t);
      if( logBuf2Pos >= LOGBUF_FULL_SIZE ){
        activeLog = 1;
        logBuf2Pos = 0;
        gb2Full = true;
      }
    }

    xSemaphoreGive( wbufSem );
  }

}

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


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * GPS serial monitoring thread
*/

static void gpsThread( void *pvParameters )
{
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("GPS thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {
    //if ( xSemaphoreTake( gpsSerSem, ( TickType_t ) 100 ) == pdTRUE ) {
      while (SERIAL_GPS.available()) {
        parser.encode((char)SERIAL_GPS.read());
      }
    //  xSemaphoreGive( gpsSerSem );
    //}

      vTaskDelay(10);
  }

  vTaskDelete( NULL );
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * IMU Monitoring thread
 *
 * Collect IMU data and store statistics on it to the log file
 *
*/
static void imuThread( void *pvParameters )
{
  bool bno_ok = false, highg_ok = false;
  imu::Vector<3> rawAcc;
  imu::Vector<3> rawGyr;
  sensors_event_t event;
  acc_t accData;
  imu_t imuData;
  // TODO: implement
  // unsigned long sample_period_ms = 20;
  unsigned long last_sample_time = 0;


  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("IMU thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
    // init BNO055 IMU
    if( bno.begin() ) {
      bno_ok = true;

      // now configure the bno to be +/-16g (4g is default)
      // gyr setting of +/-2000 dps is default
      //bno.setAccFSR(Adafruit_BNO055::ACC_FSR_16G);
    }

    // init H3LIS100 accelerometer
    if( highg.begin() ){
      highg_ok = true;
    }

    xSemaphoreGive( i2c1Sem );
  }

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.print("IMU: init BNO: ");
    Serial.print(bno_ok);
    Serial.print(", init H3LIS: ");
    Serial.println(highg_ok);
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {
    myDelayMs(500);

    // try to acquire lock on i2c bus and take measurements
    // from the IMU and high g accelerometer
    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      // get low g acceleration and gyro measurements
      rawAcc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      rawGyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

      // get high g accel measurements
      highg.getEvent(&event);

      xSemaphoreGive( i2c1Sem );
    }

    last_sample_time = xTaskGetTickCount();

    // copy high g acc data (h3lis100) into logging structs
    accData.t = last_sample_time;
    accData.data[0] = event.acceleration.x;
    accData.data[1] = event.acceleration.y;
    accData.data[2] = event.acceleration.z;

    // copy low g and gyro data (bno055 imu)
    imuData.t = last_sample_time;
    imuData.data[0] = rawAcc.x();
    imuData.data[1] = rawAcc.y();
    imuData.data[2] = rawAcc.z();
    imuData.data[3] = rawGyr.x();
    imuData.data[4] = rawGyr.y();
    imuData.data[5] = rawGyr.z();

    // try to write the imu and acc structs to the SD log buffer
    if ( xSemaphoreTake( wbufSem, ( TickType_t ) 10 ) == pdTRUE ) {
      if( activeLog == 1 ){
        // write imu data
        logBuf1[logBuf1Pos++] = PTYPE_IMU; // set packet type byte
        memcpy(&logBuf1[logBuf1Pos], &imuData, sizeof (imu_t));
        logBuf1Pos += sizeof (imu_t);

        // write acc data
        logBuf1[logBuf1Pos++] = PTYPE_ACC; // set packet type byte
        memcpy(&logBuf1[logBuf1Pos], &accData, sizeof (acc_t));
        logBuf1Pos += sizeof (acc_t);

        // is this the last data we will put in before considering the
        // buffer full?
        if( logBuf1Pos >= LOGBUF_FULL_SIZE ){
          activeLog = 2;
          logBuf1Pos = 0;
          gb1Full = true;
        }
      } else if( activeLog == 2 ){
        // write imu data
        logBuf2[logBuf2Pos++] = PTYPE_IMU; // set packet type byte
        memcpy(&logBuf2[logBuf2Pos], &imuData, sizeof (imu_t));
        logBuf2Pos += sizeof (imu_t);

        // write acc data
        logBuf2[logBuf2Pos++] = PTYPE_ACC; // set packet type byte
        memcpy(&logBuf2[logBuf2Pos], &accData, sizeof (acc_t));
        logBuf2Pos += sizeof (acc_t);

        // is this the last data we will put in before considering the
        // buffer full?
        if( logBuf2Pos >= LOGBUF_FULL_SIZE ){
          activeLog = 1;
          logBuf2Pos = 0;
          gb2Full = true;
        }
      }

      xSemaphoreGive( wbufSem );
    }



    #ifdef DEBUG_IMU
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("BNO055 aX: ");
      Serial.print(rawAcc.x(), 4);
      Serial.print("\taY: ");
      Serial.print(rawAcc.y(), 4);
      Serial.print("\taZ: ");
      Serial.println(rawAcc.z(), 4);

      Serial.print("BNO055 gX: ");
      Serial.print(rawGyr.x(), 4);
      Serial.print("\tgY: ");
      Serial.print(rawGyr.y(), 4);
      Serial.print("\tgZ: ");
      Serial.println(rawGyr.z(), 4);

      Serial.print("H3LIS100 aX: ");
      Serial.print(event.acceleration.x, 4);
      Serial.print("\taY: ");
      Serial.print(event.acceleration.y, 4);
      Serial.print("\taZ: ");
      Serial.println(event.acceleration.z, 4);

      xSemaphoreGive( dbSem );
    }
    #endif

  }

  vTaskDelete( NULL );
}


/**********************************************************************************
 * barometric pressure monitoring thread
*/
static void barThread( void *pvParameters )
{
  bool init = false;
  bar_t data;

  #ifdef DEBUG_BARO
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Barometer thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  while (!init) {
    myDelayMs(1000);

    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      // initialize barometer
      if (baro.begin()) {
        init = true;
        baro.setSeaPressure(BAR_SEA_PRESSURE);

      }
      xSemaphoreGive( i2c1Sem );
    }
  }

  #ifdef DEBUG_BARO
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Barometer initialized!");
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {

    // acquire lock on i2c bus and get barometer readings
    if ( xSemaphoreTake( i2c1Sem, ( TickType_t ) 100 ) == pdTRUE ) {
      data.prs = baro.getPressure();
      data.alt = baro.getAltitude();
      data.tmp = baro.getTemperature();
      xSemaphoreGive( i2c1Sem );
    }

    // assign sample time right after acquisition
    data.t = xTaskGetTickCount();

    // try to write the gps rmc data to the SD log buffer
    if ( xSemaphoreTake( wbufSem, ( TickType_t ) 10 ) == pdTRUE ) {

      if( activeLog == 1 ){
        // is this the last data we will put in before considering the
        // buffer full?
        logBuf1[logBuf1Pos++] = PTYPE_BAR; // set packet type byte
        memcpy(&logBuf1[logBuf1Pos], &data, sizeof (bar_t));
        logBuf1Pos += sizeof (bar_t);
        if( logBuf1Pos >= LOGBUF_FULL_SIZE ){
          activeLog = 2;
          logBuf1Pos = 0;
          gb1Full = true;
        }
      } else if( activeLog == 2 ){
        // is this the last data we will put in before considering the
        // buffer full?
        logBuf2[logBuf2Pos++] = PTYPE_BAR; // set packet type byte
        memcpy(&logBuf2[logBuf2Pos], &data, sizeof (bar_t)); // copy data to buffer
        logBuf2Pos += sizeof (bar_t); // add full offset even if we had a short copy
        if( logBuf2Pos >= LOGBUF_FULL_SIZE ){
          activeLog = 1;
          logBuf2Pos = 0;
          gb2Full = true;
        }
      }

      xSemaphoreGive( wbufSem );
    }



    #ifdef DEBUG_BARO
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("pressure: "); Serial.print(sensorData.prs); Serial.print, (" hPa, ");
      Serial.print("altitude: "); Serial.print(sensorData.alt); Serial.println(" m, ");
      Serial.print("internal temp: "); Serial.print(sensorData.tmp); Serial.println(" C");

      xSemaphoreGive( dbSem );
    }
    #endif

    myDelayMs(1000);
  }
}


/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
/**********************************************************************************
 * Iridium thread
*/
static void irdThread( void *pvParameters )
{
  bool fix_valid = false;
  uint8_t buf[SBD_TX_SZ];
  int bufLen = 0;
  bool pready = false;
  int mSq = 0, irerr; // signal quality, modem operation return code
  unsigned long lastSignalCheck = 0, lastPacketSend = 0;

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

    // check the global flag to see if there is a packet ready
    if ( xSemaphoreTake( irbSem, ( TickType_t ) 10 ) == pdTRUE ) {
      if( packetReady ){
       pready = true;
      }
      xSemaphoreGive( irbSem );
    }

    // IS IT TIME TO SEND A PACKKAGE??
    if( xTaskGetTickCount() - lastPacketSend > IRIDIUM_PACKET_PERIOD &&
        (mSq > 0) &&
        SEND_PACKETS &&
        pready){

      #ifdef DEBUG_IRD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      SERIAL.println("IRIDIUM: sending packet");
      xSemaphoreGive( dbSem );
      }
      #endif


      // fill the buff with compressed data if the global flag
      // is set telling us there is one available from the compression thread
      if ( xSemaphoreTake( irbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
        if( packetReady ){
          memcpy(buf, gIrdBuf, gPacketSize);
          bufLen = gPacketSize;

          // tell the compression thread to go ahead and refill the buffer
          // with the next compressed packet
          packetReady = false;
        }
        xSemaphoreGive( irbSem );
      }

      // try to send the packet
      irerr = modem.sendSBDBinary(buf, bufLen);

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

        pready = false; // only set this if the send succeeds
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

  #ifdef DEBUG_MCP_STARTUP
  SERIAL.print("ADC resolution set to ");
  switch (mcps[id].getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   SERIAL.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   SERIAL.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   SERIAL.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   SERIAL.print("12"); break;
  }
  SERIAL.println(" bits");
  #endif

  mcps[id].setThermocoupleType(MCP9600_TYPE_K);

  #ifdef DEBUG_MCP_STARTUP
  SERIAL.print("Thermocouple type set to ");
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
  #endif


  // TODO: what is the default filter coefficient
  // https://cdn.sparkfun.com/assets/9/0/b/0/3/MCP9600_Datasheet.pdf
  //mcps[id].setFilterCoefficient(1);
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

static void compressionThread( void * pvParameters )
{
  int pack_size = 0;
  int input_size = 0;
  unsigned long actual_read;
  char filename[LOGFILE_NAME_LENGTH] = LOGFILE_NAME;
  int timesCompSame = 0, lastCompressSize = 0;
  bool acceptShort = false;
  int temp = 0;
  int bytesRead = 0;
  packet_t packet;

  // choose based on expected size of packets you are sampling
  // and the assumption that compressing wont make the data bigger
  int packetsToSample = 1;

  // packet types are powers of two so you can OR them together
  // to get bit mask of packet types to sample
  uint16_t ptypeToSample = 0;

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Compression thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  myDelayMs(25000);

  while(1) {
    pack_size = 0;
    input_size = 0;
    actual_read = 0;
    packetsToSample = 1;
    acceptShort = false;
    lastCompressSize = 0;
    timesCompSame = 0;
    bytesRead = 0;


    // prefill with the most recent data from several sources before adding in
    // as much TC data as possible so we dont start from empty buffer each time
    if( ( temp = sample_datfile(PTYPE_GGA, 1, uc_buf) ) != ERR_SD_BUSY)
      bytesRead = temp;
    if( ( temp = sample_datfile(PTYPE_RMC, 1, &uc_buf[bytesRead] )) != ERR_SD_BUSY )
      bytesRead += temp;
    if( ( temp = sample_datfile(PTYPE_IMU, 1, &uc_buf[bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;
    if( ( temp = sample_datfile(PTYPE_ACC, 1, &uc_buf[bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;
    if( ( temp = sample_datfile(PTYPE_BAR, 1, &uc_buf[bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;
    if( ( temp = sample_datfile(PTYPE_SPEC, 1, &uc_buf[bytesRead])) != ERR_SD_BUSY )
      bytesRead += temp;

    input_size = bytesRead;

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("COMP: starting compression, input_size = ");
      Serial.println(input_size);
      xSemaphoreGive( dbSem );
    }
    #endif

    // keep sampling and compressing data until compressed data
    // is larger than 338 bytes
    while(pack_size < (SBD_TX_SZ - 2) && !acceptShort ){

      packetsToSample++;

//      #ifdef DEBUG
//      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//        Serial.print("calling sample_datfile(), asking for ");
//        Serial.print(packetsToSample);
//        Serial.println(" packets.");
//        xSemaphoreGive( dbSem );
//      }
//      #endif


      byte
      // TODO: need to sample spec data too, first just do TC
      ptypeToSample = PTYPE_TC;
      // sample the datfile, requesting packetsToSample samples of type ptypeToSample
      //taskENTER_CRITICAL();
      bytesRead = sample_datfile(ptypeToSample, packetsToSample, &uc_buf[input_size]);
      //taskEXIT_CRITICAL();

      // sample some IMU data, placing it in the uc buffer after the previous reading
      //ptypeToSample = PTYPE_IMU;
      //bytesRead += sample_datfile(ptypeToSample, packetsToSample, &uc_buf[bytesRead]);
      //ptypeToSample = PTYPE_ACC;
      //bytesRead += sample_datfile(ptypeToSample, packetsToSample, &uc_buf[bytesRead]);




//      #ifdef DEBUG
//      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//        Serial.print("Got ");
//        Serial.print(bytesRead);
//        Serial.println(" bytes back in buffer, compressing...");
//        xSemaphoreGive( dbSem );
//      }
//      #endif

      if( bytesRead == ERR_SD_BUSY ){
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.println(" ERR SD BUSY");
          xSemaphoreGive( dbSem );
        }
        #endif

        break;
      }

      // compress the contents of uc_buf and place them into memory starting at
      // c_buf+2 address, the number of bytes in the input is bytesRead, according to
      // the return value of the sample_datfile() function

      //taskENTER_CRITICAL();
      pack_size = blz_pack(uc_buf, c_buf+2, bytesRead + input_size, workmem);
      //taskEXIT_CRITICAL();

      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.print(bytesRead+input_size);
        Serial.print(" compressed (");
        Serial.print(packetsToSample);
        Serial.print(") down to ");
        Serial.print(pack_size);
        Serial.println(" bytes.");
        xSemaphoreGive( dbSem );
      }
      #endif

      if ( lastCompressSize == 0 ){
        lastCompressSize = pack_size;
      } else {
        if( pack_size == lastCompressSize ) {
          timesCompSame++;
        } else {
          lastCompressSize = pack_size;
        }
        if( timesCompSame > 10 ) acceptShort = true;
      }

      //myDelayMs(5); // wait 5ms between compression runs
    }

    if( pack_size > (SBD_TX_SZ - 2) ){
      packetsToSample--;

//      #ifdef DEBUG
//      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//        Serial.print("calling sample_datfile(), asking for ");
//        Serial.print(packetsToSample);
//        Serial.println(" packets.");
//        xSemaphoreGive( dbSem );
//      }
//      #endif

      // TODO: need to sample spec data too, first just do TC
      ptypeToSample = PTYPE_TC;

      // sample the datfile, requesting packetsToSample samples of type ptypeToSample
      //taskENTER_CRITICAL();
      bytesRead = sample_datfile(ptypeToSample, packetsToSample, &uc_buf[input_size]);
      //taskEXIT_CRITICAL();

//      #ifdef DEBUG
//      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
//        Serial.print("Got ");
//        Serial.print(bytesRead);
//        Serial.println(" bytes back in buffer, compressing...");
//        xSemaphoreGive( dbSem );
//      }
//      #endif

      if( bytesRead == ERR_SD_BUSY ){
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.println(" ERR SD BUSY");
          xSemaphoreGive( dbSem );
        }
        #endif

        break;
      }

      // compress the contents of uc_buf and place them into memory starting at
      // c_buf+2 address, the number of bytes in the input is bytesRead, according to
      // the return value of the sample_datfile() function

      //taskENTER_CRITICAL();
      pack_size = blz_pack(uc_buf, c_buf+2, bytesRead + input_size, workmem);
      //taskEXIT_CRITICAL();

      #ifdef DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.print(bytesRead+input_size);
        Serial.print(" compressed (");
        Serial.print(packetsToSample);
        Serial.print(") down to  ");
        Serial.print(pack_size+2);
        Serial.println(" bytes.");
        xSemaphoreGive( dbSem );
      }
      #endif

      /*
      if ( lastCompressSize == 0 ){
        lastCompressSize = pack_size;
      } else {
        if( pack_size == lastCompressSize ) {
          timesCompSame++;
        } else {
          lastCompressSize = pack_size;
        }
        if( timesCompSame > 10 ) acceptShort = true;
      }
      */

    }

    // fill in size of compressed buffer into the first 2 bytes
    *(uint16_t*)(&c_buf[0]) = (uint16_t)(bytesRead + input_size);

    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      Serial.print("COMP: ready to send compressed packet of size ");
      Serial.print(pack_size+2);
      Serial.println(" bytes.");
      xSemaphoreGive( dbSem );
    }
    #endif

    // copy into packet structure
    packet.t = xTaskGetTickCount();
    packet.size = pack_size+2;
    memset(packet.data, 0, SBD_TX_SZ);
    memcpy(packet.data, c_buf, pack_size+2);

    // if there is not already data in it, copy this compressed packet
    // to the global iridium buffer to be sent asap
    if ( xSemaphoreTake( irbSem, ( TickType_t ) 10 ) == pdTRUE ) {
      if( !packetReady ){
        packetReady = true;
        memcpy(gIrdBuf, c_buf, pack_size+2);
        gPacketSize = pack_size+2;
      }
      xSemaphoreGive( irbSem );
    }

    //  write packet to logfile for debugging whether we were
    // able to copy it to the iridium buffer or not
    if ( xSemaphoreTake( wbufSem, ( TickType_t ) 10 ) == pdTRUE ) {
      if( activeLog == 1 ){
        logBuf1[logBuf1Pos++] = PTYPE_PACKET; // set packet type byte
        memcpy(&logBuf1[logBuf1Pos], &packet, sizeof (packet_t));
        logBuf1Pos += sizeof (packet_t);
        if( logBuf1Pos >= LOGBUF_FULL_SIZE ){
          activeLog = 2;
          logBuf1Pos = 0;
          gb1Full = true;
        }
      } else if( activeLog == 2 ){
        logBuf2[logBuf2Pos++] = PTYPE_PACKET; // set packet type byte
        memcpy(&logBuf2[logBuf2Pos], &packet, sizeof (packet_t));
        logBuf2Pos += sizeof (packet_t);
        if( logBuf2Pos >= LOGBUF_FULL_SIZE ){
          activeLog = 1;
          logBuf2Pos = 0;
          gb2Full = true;
        }
      }
      xSemaphoreGive( wbufSem );
    }

    // TESTING: build a packet every 15 seconds
    myDelayMs(PACKET_BUILD_PERIOD);
  }

  vTaskDelete( NULL );
}

// thread definition
static void tcThread( void *pvParameters )
{
  tc_t data;
  float current_temps[NUM_TC_CHANNELS];
  unsigned long lastDebug = 0;
  unsigned long lastRead = 0;
  uint8_t  ledState = 0;

  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("TC thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  while(1) {
    if( xTaskGetTickCount() - lastRead > 500 ){
      lastRead = xTaskGetTickCount();
      //safeKick();

      if ( xSemaphoreTake( ledSem, ( TickType_t ) 100 ) == pdTRUE ) {
        ledState = (ledState + 1) % 4;
        ledError( ledState );
        xSemaphoreGive( ledSem );
      }

      // assign tc temps from MCP objects to local vars
      for( int i=0; i<NUM_TC_CHANNELS; i++ ){
        current_temps[i] = readMCP(i);
        myDelayMs(1);
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
      data.t = xTaskGetTickCount();
      for( int i=0; i<NUM_TC_CHANNELS; i++ ){
        data.data[i] = current_temps[i];
      }

      // try to write the tc data to the SD log buffer that is available
      if ( xSemaphoreTake( wbufSem, ( TickType_t ) 10 ) == pdTRUE ) {

        if( activeLog == 1 ){
          // is this the last data we will put in before considering the
          // buffer full?
          logBuf1[logBuf1Pos++] = PTYPE_TC; // set packet type byte
          memcpy(&logBuf1[logBuf1Pos], &data, sizeof (tc_t));
          logBuf1Pos += sizeof (tc_t);
          if( logBuf1Pos >= LOGBUF_FULL_SIZE ){
            activeLog = 2;
            logBuf1Pos = 0;
            gb1Full = true;
          }
        } else if( activeLog == 2 ){
          // is this the last data we will put in before considering the
          // buffer full?
          logBuf2[logBuf2Pos++] = PTYPE_TC; // set packet type byte
          memcpy(&logBuf2[logBuf2Pos], &data, sizeof (tc_t));
          logBuf2Pos += sizeof (tc_t);
          if( logBuf2Pos >= LOGBUF_FULL_SIZE ){
            activeLog = 1;
            logBuf2Pos = 0;
            gb2Full = true;
          }
        }

        xSemaphoreGive( wbufSem );
      }
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
  uint32_t filesize = 0;
  File logfile;

  #ifdef DEBUG_LOG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("sd logging thread thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  // wait indefinitely for the SD mutex
  while( xSemaphoreTake( sdSem, portMAX_DELAY ) != pdPASS );

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


#if DEBUG
if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
  Serial.print("About to decide on filename, init is: ");
  Serial.println(filename);
  xSemaphoreGive( dbSem );
}
#endif

  // CREATE UNIQUE FILE NAMES (UP TO 100)
  for( uint8_t i=0; i < 100; i++) {
    filename[3] = '0' + i/10;
    filename[4] = '0' + i%10;

    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  // write a header block to the file to start
  // the first two bytes are the block size
  logfile = SD.open(filename, FILE_WRITE);
  *(uint16_t*)(&logBuf1[0]) = (uint16_t)(LOGBUF_BLOCK_SIZE);
  logfile.write(logBuf1, LOGBUF_BLOCK_SIZE);
  logfile.close();

  xSemaphoreGive( sdSem );

  #if DEBUG_LOG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.print("Wrote header to filename: ");
    Serial.println(filename);
    xSemaphoreGive( dbSem );
  }
  #endif

  // main thread loop
  // check if a buffer is full and if it is switch the active buffer that the threads dump data to to
  // be the other buffern and start writing this one (ping pong system)
  while(1) {

    if( !gb1Full && !gb2Full ){
      myDelayMs(10);
      taskYIELD();
      continue;
    }

    // wait indefinitely for the SD mutex
    //while( xSemaphoreTake( sdSem, portMAX_DELAY ) != pdPASS );
    if ( xSemaphoreTake( sdSem, ( TickType_t ) 1000 ) == pdTRUE ) {

      // open log file for all telem (single log file configuration)
      logfile = SD.open(filename, FILE_WRITE);

      filesize = logfile.size();

      // if we could not open the file
      if( ! logfile ) {
        #if DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("ERROR: sd logging thread couldn't open logfile for writing: ");
          Serial.println(filename);
          xSemaphoreGive( dbSem );
        }
        #endif
      } else {
        #if DEBUG_LOG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("SDLOG: logfile size is ");
          Serial.print(filesize);
          Serial.println(" bytes");
          xSemaphoreGive( dbSem );
        }
        #endif
      }

      // we assume we can open it and start writing the buffer
      // check if one of the buffers is full and lets write it
      if( gb1Full ){

        logfile.seek(logfile.size());

        written = logfile.write(logBuf1, LOGBUF_BLOCK_SIZE);

        // clean the buffer so we are gauranteed to see zeros where there hasn't
        // been recetn data written to
        memset(logBuf1, 0, LOGBUF_BLOCK_SIZE);

        #if DEBUG_LOG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("SD: wrote  ");
          Serial.print(written);
          Serial.print("/");
          Serial.print(LOGBUF_BLOCK_SIZE);
          Serial.println(" bytes from logBuf1");
          xSemaphoreGive( dbSem );
        }
        #endif

        gb1Full = false;
      }

      // check the other buffer
      if( gb2Full ){

        logfile.seek(logfile.size());

        written = logfile.write(logBuf2, LOGBUF_BLOCK_SIZE);

        // clean the buffer so we are gauranteed to see zeros where there hasn't
        // been recetn data written to
        memset(logBuf2, 0, LOGBUF_BLOCK_SIZE);

        #if DEBUG_LOG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          Serial.print("SD: wrote  ");
          Serial.print(written);
          Serial.print("/");
          Serial.print(LOGBUF_BLOCK_SIZE);
          Serial.println(" bytes from logBuf2");
          xSemaphoreGive( dbSem );
        }
        #endif

        gb2Full = false;
      }

      // done writing to this file
      logfile.close();

      xSemaphoreGive( sdSem );
    }

    myDelayMs(10);
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

    myDelayMs(500);
    #ifdef DEBUG
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
      if( Serial.available() ){
        if( Serial.read() == 'p' ){
          deployed = true;
        }

      }
      xSemaphoreGive( dbSem );
    }
    #endif


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

      digitalWrite(PIN_CHUTE_ACT, HIGH);

      //linAct.write(ACT_POS_ACT);
    }

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

  // servo control using servo trigger board
  pinMode(PIN_CHUTE_ACT, OUTPUT);
  digitalWrite(PIN_CHUTE_ACT, LOW);

  // attach servo
  // need 3.3v -> 5v level shifter for linear actuators
  //  linAct.attach(PIN_CHUTE_ACT);
  //  linAct.write(ACT_POS_HOME);

  #if DEBUG
  SERIAL.begin(115200); // init debug serial
  #endif

  delay(100);

  int num = 0;

  for (int i = 0; i < NUM_SPEC_CHANNELS; i++)
  { // Create the coefficients for the simpson's rule integral
      if ((i == 0) || (i == NUM_SPEC_CHANNELS))
      { // I = Delta x/3 * ( 1 f(x_0) + 4 f(x_1) + 2 f(x_2) + ... + 2 f(x_{n-2}) + 4 f(x_{n-1}) + f(x_n) )
          num = 1;
      }
      else if (i % 2 == 1)
      {
          num = 4;
      }
      else if (i % 2 == 0)
      {
          num = 2;
      }
      multipliers_1[i] = num;
      multipliers_2[i] = num;
  }

  pinMode(PIN_SPEC1_CLK, OUTPUT);
  pinMode(PIN_SPEC1_START, OUTPUT);
  pinMode(PIN_SPEC2_CLK, OUTPUT);
  pinMode(PIN_SPEC2_START, OUTPUT);

  digitalWrite(PIN_SPEC1_CLK, HIGH); // Set SPEC_CLK High
  digitalWrite(PIN_SPEC1_START, LOW);   // Set SPEC_ST Low
  digitalWrite(PIN_SPEC2_CLK, HIGH); // Set SPEC_CLK High
  digitalWrite(PIN_SPEC2_START, LOW);   // Set SPEC_ST Low

  delay(100);

  delay(100);
  SERIAL_GPS.begin(9600); // init gps serial
  delay(10);
  SERIAL_IRD.begin(19200); // init iridium serial
  //delay(10);
  //SERIAL_LOR.begin(115200); // init LORA telemetry radio serial

  // Assign SERCOM functionality to enable 3 more UARTs
  //pinPeripheral(A1, PIO_SERCOM_ALT);
  //pinPeripheral(A4, PIO_SERCOM_ALT);
  //pinPeripheral(A2, PIO_SERCOM_ALT);
  //pinPeripheral(A3, PIO_SERCOM_ALT);
  pinPeripheral(13, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);


  // reset pin for lora radio
  //pinMode(PIN_LORA_RST, OUTPUT);

  // battery voltage divider
  pinMode(PIN_VBAT, INPUT);

  Wire.begin();
  Wire.setClock(100000);

  delay(3000);

  #ifdef DEBUG
  SERIAL.println("Starting...");
  #endif

  // zero out log buffers
  memset(logBuf1, 0, LOGBUF_BLOCK_SIZE);
  memset(logBuf2, 0, LOGBUF_BLOCK_SIZE);

  led.begin();
  led.show();

  ledError(ERR_BOOT);

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
  // setup iridium buffer protector
  if ( irbSem == NULL ) {
    irbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( irbSem ) != NULL )
      xSemaphoreGive( ( irbSem ) );  // make available
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
  // setup sd sem
  if ( sdSem == NULL ) {
    sdSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( sdSem ) != NULL )
      xSemaphoreGive( ( sdSem ) );  // make available
  }

  #ifdef DEBUG
  SERIAL.println("Created semaphores...");
  #endif

  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(tcThread,   "TC Measurement", 512, NULL, tskIDLE_PRIORITY, &Handle_tcTask);
  xTaskCreate(logThread,  "SD Logging", 512, NULL, tskIDLE_PRIORITY-1, &Handle_logTask);
  xTaskCreate(specThread, "Spectrometer", 512, NULL, tskIDLE_PRIORITY, &Handle_specTask);
  xTaskCreate(irdThread,  "Iridium", 512, NULL, tskIDLE_PRIORITY, &Handle_irdTask);
  xTaskCreate(parThread,  "Parachute Deployment", 512, NULL, tskIDLE_PRIORITY, &Handle_parTask);
  xTaskCreate(barThread,  "Capsule internals", 512, NULL, tskIDLE_PRIORITY, &Handle_barTask);
  xTaskCreate(imuThread,  "IMU reading", 512, NULL, tskIDLE_PRIORITY, &Handle_imuTask);
  xTaskCreate(gpsThread,  "GPS Reception", 512, NULL, tskIDLE_PRIORITY, &Handle_gpsTask);
  xTaskCreate(compressionThread, "Data Compression", 512, NULL, tskIDLE_PRIORITY, &Handle_compTask);

  //xTaskCreate(radThread, "Telem radio", 1000, NULL, tskIDLE_PRIORITY, &Handle_radTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);

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
