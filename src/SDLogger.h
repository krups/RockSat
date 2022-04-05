/*
* KREPE ISS Mission
* Matt Ruffner Dec 2020
*
* SDLogger supports writing to multiple files, referenced by a file ID.
* 
*/

#ifndef SDLOGGER
#define SDLOGGER

#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <TeensyThreads.h>

#include "config.h"
#include "packets.h"

#define PIN_SD_CS  BUILTIN_SDCARD

#define MAX_OPEN_FILES 5

extern void safePrintln(String s);
extern void safePrint(String s);

class SDLogger {

public:
  SDLogger() : _ready(false), _logNum(0) {}
  ~SDLogger() {}
  
  /***********************
  * init the SD card and make sure we have one
  * read in the log count from EEPROM, increment and store
  */
  void begin() {
    if (!SD.begin(PIN_SD_CS)) {
      if(USBSERIAL_DEBUG) safePrintln("Card failed, or not present");
      // don't do anything more:
      _ready = false;
      return;
    }
    if(USBSERIAL_DEBUG) safePrintln("card initialized.");
    _ready = true;
    
    for( int i=0; i<MAX_OPEN_FILES; i++ ){
      _open[i] = false;
    }
    
    // read in last log id from memory , sizeof(int)
    EEPROM.get(0, _logNum);
    EEPROM.update(0, (_logNum+1) % 100);

    if(USBSERIAL_DEBUG) safePrint("-----> log # "); safePrintln(_logNum);
  }
  
  /*******************************
  * create an entry in the current file list, stored by filename at index 'id'
  * param logtag: start of filename (appends a rolling number and .txt to the end)
  * returns true if the file was successfully created
  */  
  bool createLog(String logtag, int id) {
    if( !_ready ){
      if(USBSERIAL_DEBUG) safePrintln("SDLOG: not ready, please call begin()");
      return false;
    }
  
    // only allow a few files open
    if( id >= MAX_OPEN_FILES ){
      return false;
    }
    
    // create filename and try to open file
    String fname;
    fname += (String(_logNum) + logtag);
    if( fname.length() > 8 ){
      safePrint("SDLOG: need shorter name");
    }
    fname +=".dat";

    if(USBSERIAL_DEBUG) {
      //safePrint("SDLOG: filename is '");
      //safePrint(fname);safePrintln("'");
      //safePrint(("SDLOG: storing logfile entry: " + fname));
      //safePrintln(", file not yet created");
    }
    _fh = SD.open(fname.c_str(), FILE_WRITE);
    if( _fh ) {
      _fnames[id] = fname;
      //if(USBSERIAL_DEBUG) safePrintln("SDLOG:  ok. opened log file: " + fname);
      _fh.close();
      return true;
    } else {
      //if(USBSERIAL_DEBUG) safePrintln("SDLOG:  err. couldnt open file: " + fname);
      return false;
    }
  }
  
  /******************************
  * for logging text
  */
  byte logMsg(int id, String s) {
    // the bouncer
    if( !_idValid(id) ) return 0;
    
    //if(USBSERIAL_DEBUG) safePrint("trying to open logfile #"); safePrint(String(id)); safePrint(" with filename "); safePrintln(_fnames[id]);
  
    _fh = SD.open(_fnames[id].c_str(), FILE_WRITE);
  
    if( _fh ){
      //safePrint("ok. openend file "); safePrintln(_fnames[id]);
      //safePrint("  file has "); safePrint(_fh.size()); safePrint(" bytes in it, seeking to end");
      
      _fh.seek(_fh.size());
      byte ret = _fh.println(s);
      _fh.close();
      return ret;
  
    } else {
      if(USBSERIAL_DEBUG) safePrint("error: could not re-open file "); safePrint(_fnames[id]); safePrintln(" for logging");
      return 0;
    }      
  }
  
  /*******************************
  * for logging packets in binary format:
  * [ packet type (1 byte)     ]
  * [ packet len  (2 bytes)    ]
  * [ packet data (len bytes) ]
  */
  byte logBin(int id, Packet *p) {
    // bounce
    if( !_idValid(id) ) return 0;
    
    //if(USBSERIAL_DEBUG) safePrint("trying to open logfile #"); safePrint(String(id)); safePrint(" with filename "); safePrintln(_fnames[id]);
  
    // open file referenced by id
    _fh = SD.open(_fnames[id].c_str(), FILE_WRITE);
  
    // if file opened
    if( _fh ){
      //safePrint("ok. openend file "); safePrintln(_fnames[id]);
      //safePrint("  file has "); safePrint(_fh.size()); safePrint(" bytes in it, seeking to end");
      
      // seek to end of file
      _fh.seek(_fh.size());
      
      // write packet data, including type char
      uint8_t b = p->type();
      byte ret = _fh.write(&b, 1);
      ret += _fh.write(p->data(), p->size());
      _fh.close(); //close file
      return ret; // return the number of bytes written. TODO: make sure this is equal to the number attempted to write.
  
    } else {
      if(USBSERIAL_DEBUG) safePrint("error: could not re-open file "); safePrint(_fnames[id]); safePrintln(" for logging");
      return 0;
    }   
  
  }
  
  // Take uniform sample with number of bytes = `size` from file with id=`id`
  // Put result in sam_buf
  void sample(int id, byte* sam_buf, unsigned long sizeRequest, unsigned long *sizeActual){
    if( !_idValid(id) ){
      safePrintln("attempt to sample file with ID that doesn't exist");
      return;
    }
    // open file referenced by id
    _fh = SD.open(_fnames[id].c_str(), FILE_READ);
    if( !_fh ) {
      safePrintln("error opening file");
      return;
    }
    //safePrint("wanting "); safePrint(sizeRequest); safePrintln("bytes");
    char type_buf[1];
    _fh.read(type_buf, 1);
    _fh.seek(0);
    unsigned long packet_size;
    
    switch (type_buf[0])
    {
    case PTYPE_TELEM:
      packet_size = TELEM_T_SIZE;
      break;
    case PTYPE_ACCELSTATS:
      packet_size = ACC_STAT_T_SIZE;
      break;
    case PTYPE_TC:
      packet_size = TC_T_SIZE;
      break;
    case PTYPE_ACCEL:
      packet_size = ACC_T_SIZE;
      break;
    case PTYPE_IMU:
      packet_size = IMU_T_SIZE;
      break;
    default:
      if(USBSERIAL_DEBUG) safePrint(_fnames[id]); safePrintln(" has an unexpected layout.");
      return;
      break;
    }
    packet_size += 1; // for packet ID char
    
    //safePrint("packet size is "); safePrint(packet_size); safePrintln(" bytes");
    
    unsigned long num_samples_needed = floor((float)sizeRequest/((float)packet_size));
    unsigned long num_packets_available = (unsigned long)floor(((float)_fh.size())/((float)packet_size));
    
    if( num_packets_available < num_samples_needed ){
      num_samples_needed = num_packets_available;
    }
    
    unsigned long interval = (unsigned long)floor(((float)num_packets_available)/((float)num_samples_needed));
    
    //safePrint("num_samples_needed:    "); safePrintln(num_samples_needed);
    //safePrint("num_packets_available: "); safePrintln(num_packets_available);
    //safePrint("interval:              "); safePrintln(interval);
    int i=0;
    *sizeActual = 0;
    for( i = 0; i < num_samples_needed; i++ ){
      int loc = i*packet_size*interval;
      //safePrint("  seeking to "); safePrintln(loc);
      _fh.seek(loc);
      //safePrint("and reading "); safePrint(packet_size); safePrintln("bytes");
      *sizeActual += _fh.read(&sam_buf[i*(packet_size)], packet_size);
    }
    
    //*sizeActual = (i * packet_size);
    
    _fh.close();

  }

  // Get the latest packet from the file specified by `id`. 
  // `sam_buf` must have enough space allocated for the given packet type.
  int latest_packet(int id, byte* sam_buf){
    int bytes_read = 0;
    if( !_idValid(id) ){
      safePrintln("attempt to sample file with ID that doesn't exist");
      return;
    }
    // open file referenced by id
    _fh = SD.open(_fnames[id].c_str(), FILE_READ);
    if( !_fh ) {
      safePrintln("error opening file");
      return -1;
    }
   
    int packet_size;
    switch (id)
    {
    case LOGID_TELEM:
      packet_size = TELEM_T_SIZE;
      break;
/*    case PTYPE_ACCELSTATS:
      packet_size = ACC_STAT_T_SIZE;
      break;
    case PTYPE_TC:
      packet_size = TC_T_SIZE;
      break;
    case PTYPE_ACCEL:
      packet_size = ACC_T_SIZE;
      break;
    case PTYPE_IMU:
      packet_size = IMU_T_SIZE;
      break;*/
    default:
      if(USBSERIAL_DEBUG) {safePrint(_fnames[id]); safePrintln(" has an unexpected layout.");}
      return -1;
      break;
    }
    packet_size += 1;
    unsigned int start_loc = _fh.size() -  packet_size;
    //safePrint("latest_packet:  seeking to "); safePrint(start_loc);
    _fh.seek(start_loc);

    //safePrint("and reading "); safePrint(packet_size); safePrintln("bytes");
    bytes_read += _fh.read(sam_buf, packet_size);
    _fh.close();
    
    return bytes_read;
} 

  bool isReady() { return _ready; }

  uint8_t logNum() { return _logNum; }

private:
  // parameter validation for the ID corresponding to the the file to write to
  bool _idValid(int id) {
    if( !_ready ){
      if(USBSERIAL_DEBUG) safePrintln("SD card not initialized");
      return false;
    }
    if( (id >= MAX_OPEN_FILES) || (id < 0) ){
      if(USBSERIAL_DEBUG) safePrintln("invalid file index");
      return false;
    }
    return true;
  }

  bool _ready, _open[MAX_OPEN_FILES];
  uint8_t _logNum;
  
  String _fnames[MAX_OPEN_FILES];
  File _fh;

};

#endif
