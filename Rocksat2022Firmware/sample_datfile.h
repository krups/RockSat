#ifndef SAMPLE_DATFILE_H
#define SAMPLE_DATFILE_H

#include "config.h"
#include "packet.h"

// sample a data file taking numPackets of each type ptype and put them
// sequenctially into the output buffer
int sample_datfile(char *filename, int fnameLen, uint16_t ptype, int numPackets, unsigned char *output)
{
  File logfile;
  int numSampled = 0;
  int fileSize = 0;
  int numBlocks = 0;
  int blockStride = 0;
  unsigned long lastTime;
  int i, offset;
  char type;


  if( gb1Full || gb2Full ){
    return ERR_SD_BUSY;
  }

  uint16_t packet_size = 0;
  if( ptype && PTYPE_TC )
    packet_size += sizeof (tc_t);
  if( ptype && PTYPE_SPEC )
    packet_size += sizeof (spec_t);

  #if DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.print("SAMPLE: chunk size is ");
    Serial.println(packet_size);
    xSemaphoreGive( dbSem );
  }
  #endif

  if ( xSemaphoreTake( sdSem, ( TickType_t ) 100 ) == pdTRUE ) {

    // make sure file exists
    if (! SD.exists(filename)) {
      return ERR_SD_BUSY;
    }

    // open log file and read file info
    logfile = SD.open(filename, FILE_WRITE);
    fileSize = logfile.size();
    numBlocks = fileSize / LOGBUF_SIZE;

    logfile.seek( fileSize - LOGBUF_SIZE );

    // go until the end of data in this last block
    // and find the most recent timestamp recorded by a packet
    // first read the packet type which should be nonzero otherwise
    // we have found the end of the file
    while( (type = logfile.read()) != 0 ){
      // timestamp is always first in packet (after 1 byte type preamble)
      logfile.read((unsigned char *)(&lastTime), 4);

      if( type==PTYPE_TC )        offset = sizeof (tc_t) - 4;
      else if( type==PTYPE_SPEC ) offset = sizeof (spec_t) - 4;

      // slurp the rest of the packet depending on what type it is
      #if DEBUG
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        Serial.print("  SAMPLE: seeking to:  ");
        Serial.println(logfile.position()+offset);
        xSemaphoreGive( dbSem );
      }
      #endif

      logfile.seek( logfile.position() + offset );
    }

    // close the file
    logfile.close();

    xSemaphoreGive( sdSem );
  } else {
    return ERR_SD_BUSY;
  }

  #if DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.print("SAMPLE: file size: ");
    Serial.print(fileSize);
    Serial.print(", numBlocks: ");
    Serial.print(numBlocks);
    Serial.print(", lastTime: ");
    Serial.println(lastTime);
    xSemaphoreGive( dbSem );
  }
  #endif

  for(i=0; i<numBlocks; i++){


  }

  ledOk();
  return 0;
}



#endif
