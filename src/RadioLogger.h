#ifndef RADIO_LOGGER_H
#define RADIO_LOGGER_H

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define CAPSULE_ADDRESS               2
#define STATION_ADDRESS               1
#define RF69_FREQ                     915.0
#define PACKET_BUFFER_NUM_PACKETS     5
#define SEND_BUF_SIZE                 RH_RF69_MAX_MESSAGE_LEN
#define RECV_BUF_SIZE                 RH_RF69_MAX_MESSAGE_LEN
#define RECV_QUEUE_SIZE               180
#define TEMPBUF_SIZE                  60 

extern void safePrintln(String s);
extern void safePrint(String s);

class RadioLogger {
public:
  RadioLogger() : sbuflen(0), rbuflen(0), rssi(0), lastFrom(0), pcount(0), rq_read(0), rq_write(0), rq_size(0) {};
  
  bool begin() {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    
    // manual reset
    //digitalWrite(RFM69_RST, HIGH);
    //delay(10);
    //digitalWrite(RFM69_RST, LOW);
    //delay(10);
    
    //safePrintln("initialziing");
    
    if (!rf69_manager.init()) {
      //safePrintln("RFM69 radio init failed");
      return false;
    }
    
    if (USBSERIAL_DEBUG) safePrintln("Setting frequency");
    if (!rf69.setFrequency(RF69_FREQ)) {
      //safePrintln("setFrequency failed");
      return false;
    }
    
    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    // The encryption key has to be the same as the one in the server
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);
    
    //safePrintln("Setup done");
    return true;
  };

  bool send(Packet *p, uint8_t addr) {
    // Send a message to the DESTINATION!
    
    // if the packet will fit in the send buffer
    if( p->size() >= RH_RF69_MAX_MESSAGE_LEN ){
      //safePrintln("sending super long messages not supported yet");
      return false;
    }
    
    // reset the send buffer size
    sbuflen = 0;
    
    // set the packet type field
    sendbuf[0] = p->type(); 
    
    // copy over the packet data
    memcpy( &sendbuf[1], p->data(), p->size() );
    
    // set the buffer size, accounting for the type byte
    sbuflen = p->size() + 1;
    
    //safePrint("about to send");
    //safePrint(String(sbuflen));
    //safePrintln(" bytes");
    
    if (rf69_manager.sendtoWait(sendbuf, sbuflen, addr)) {
      //safePrintln(" -> sent");
      return true;      
    } else {
      //safePrintln(" -> failed (no ack)");
      return false;
    }    
  }
  
  bool receivePackets() {
    uint8_t to, id, flags, len;
    bool ok = false;

    // receive any new data into the buffer, offset by what has yet to be written into the queue
    ok = rf69_manager.recvfromAck(recvbuf + rbuflen, &len, &lastFrom, &to, &id, &flags);
    
    // if we received a valid packet
    if( ok ){
    
      //safePrintln("** just finished receiving a packet");
      //safePrint("rq_read = "); safePrint(String(rq_read)); safePrint(", ");
      //safePrint("rq_write = "); safePrintln(String(rq_write));
      //safePrint("rbuflen = "); safePrintln(String(len));
      // the new number of bytes in the buffer
      rbuflen = len;
      
      // now copy the received data into the rec queue for later processing
      // incrementing the write index and queue size as we go and also
      // making sure to not overfill the queue
      int i=0;
      for( i=0; i<rbuflen; i++ ){
        writeQueueByte(recvbuf[i]);
        
        // check if the receive queue is full
        // TODO: do something about it if it is
        if( rq_size >= RECV_QUEUE_SIZE ){
           if (USBSERIAL_DEBUG) safePrint("ERROR: recvque full!");
           break; 
        }
      }
      
      // double check if we copied all data into recvque
      if( i != rbuflen ){
        if (USBSERIAL_DEBUG) safePrint("ERROR: didn't copy all data into recvque!");
        rbuflen = rbuflen - i;
      } else {
        rbuflen = 0;
      }
      
      //safePrintln("** done copying received data to queue");
      //safePrint("rq_read = "); safePrint(String(rq_read)); safePrint(", ");
      //safePrint("rq_write = "); safePrint(String(rq_write));
      //safePrint("rq_size = "); safePrintln(String(rq_size));

      //rssi = rf69.lastRssi();
      
      // return true indicating we successfully received data
      return true;
    } else {
      // no packet received
      return false;
    }
  }
  
  int decodePackets() {    
    //if( rbuflen == 0) return 0;
  
    // detect number of packets containted in buffer
    int num_packets = 0;
    
    uint8_t tempbuf[TEMPBUF_SIZE];
    
    int i = 0;
    int i_orig = 0;
    int rq_read_orig = rq_read;
    
    //safePrintln("** starting to decode packets");
    //safePrint("rq_read = "); safePrint(String(rq_read)); safePrint(", ");
    //safePrint("rq_write = "); safePrintln(String(rq_write));
    
    // loop through the recieve queue to find how many COMPLETE packets there are
    for( i=0; i<rq_size; i++ ){
    
      // save the current size in case we increment too far
      i_orig = i;
    
      // decide packet type based on type byte
      switch( recvque[rq_read] ){
      
        case PTYPE_TELEM:
          num_packets++;
          i += TELEM_T_SIZE;
          rq_read = (rq_read + TELEM_T_SIZE) % RECV_QUEUE_SIZE;
          break;
          
        case PTYPE_ACCELSTATS:
          num_packets++;
          i += ACC_STAT_T_SIZE;
          rq_read = (rq_read + ACC_STAT_T_SIZE) % RECV_QUEUE_SIZE;
          break;
          
        case PTYPE_TC:
          num_packets++;
          i += TC_T_SIZE;
          rq_read = (rq_read + TC_T_SIZE) % RECV_QUEUE_SIZE;
          break;
          
        case PTYPE_ACCEL:
          num_packets++;
          i += ACC_T_SIZE;
          rq_read = (rq_read + ACC_T_SIZE) % RECV_QUEUE_SIZE;
          break;
          
        case PTYPE_COMMAND:
          num_packets++;
          i += CMD_T_SIZE;
          rq_read = (rq_read + CMD_T_SIZE) % RECV_QUEUE_SIZE;
          break;
          
        case PTYPE_IMU:
          num_packets++;
          i += IMU_T_SIZE;
          rq_read = (rq_read + IMU_T_SIZE) % RECV_QUEUE_SIZE;
          break;
          
        default:
          if (USBSERIAL_DEBUG) safePrintln("ERAWR: lost track of what packet was which n where n what not");
          break;
      }
      
      // we have received the beginning of a packet without the complete data
      // in this case, leave that partial data in the queue (including the type byte) 
      // so that future received data (presumable the rest of the partial packet) is appended 
      // to the correct place in the queue
      if( i > rq_size ){
        if (USBSERIAL_DEBUG) {
          safePrint("leaving "); 
          safePrint( String(i-rq_size) );
          safePrintln(" bytes in queue");
        }
        
        // set the number of bytes to process to be those of complete packets only
        i = i_orig;
        
        // decrement packet count to not include the partial packet
        num_packets--;
        
        // done counting packets in receive queue
        break;
      }
    }
    
    // reset the receive queue read index to be where it was before we looped through to count packets
    // because now we need to read that same data again to actually create the packets.
    rq_read = rq_read_orig;
    
    
  
        
    //safePrint("detected ");
    //safePrint(String(num_packets));
    //safePrintln(" whole packets");  
    

    // NOW BUILD PACKETS
    // for each packet we just identified, copy its data from the queue to a temp buffer and create 
    // a packet from it to go in the packet buffer
    // assumes the packet buffer is empty (i.e. by printing and deleting packets after decoding packets)
    
    
    // save initial queue size before reading 
    int rq_size_orig = rq_size;
    
    // force empty packet buffer
    pcount = 0;
    
    while( pcount < num_packets ){  
      
      // telemetry packet
      if( recvque[rq_read] == PTYPE_TELEM ){
        //if (USBSERIAL_DEBUG) safePrintln("got TELEM packet");
        
        // advance past type byte, discarding result
        readQueueByte();

        // copy packet data to temp buffer
        for( int i=0; i<TELEM_T_SIZE; i++){
          tempbuf[i] = readQueueByte();
        }
        pbuf[pcount] = new TelemPacket(tempbuf);
        pcount++;
      } 
      
      // min/max stats over a period of time from the high g accelerometer
      else if( recvque[rq_read] == PTYPE_ACCELSTATS ){
        //if (USBSERIAL_DEBUG) safePrintln("got ACEL_STATS packet");
        
        // advance past type byte, discarding result
        readQueueByte();
        
        // copy packet data to temp buffer
        for( int i=0; i<ACC_STAT_T_SIZE; i++){
          tempbuf[i] = readQueueByte();
        }
        
        pbuf[pcount] = new AccStatsPacket(tempbuf);
        pcount++;
      } 
      
      // timestamped thermocouple packet (containing TC_COUNT readings)
      else if( recvque[rq_read] == PTYPE_TC ){
        //if (USBSERIAL_DEBUG) safePrintln("got TC packet");
        
        // advance past type byte, discarding result
        readQueueByte();
        
        // copy packet data to temp buffer
        for( int i=0; i<TC_T_SIZE; i++){
          tempbuf[i] = readQueueByte();
        }
        
        pbuf[pcount] = new TcPacket(tempbuf);
        pcount++;
      } 
      
      // singe timestamped high g accelerometer reading
      else if( recvque[rq_read] == PTYPE_ACCEL ){
        //if (USBSERIAL_DEBUG) safePrintln("got ACCEL packet");
        
        // advance past type byte, discarding result
        readQueueByte();
        
        // copy packet data to temp buffer
        for( int i=0; i<ACC_T_SIZE; i++){
          tempbuf[i] = readQueueByte();
        }
        
        pbuf[pcount] = new AccPacket(tempbuf);
        pcount++;
      } 
      
      // singe timestamped imu reading
      else if( recvque[rq_read] == PTYPE_IMU ){
        //if (USBSERIAL_DEBUG) safePrintln("got IMU packet");
        
        // advance past type byte, discarding result
        readQueueByte();
        
        // copy packet data to temp buffer
        for( int i=0; i<IMU_T_SIZE; i++){
          tempbuf[i] = readQueueByte();
        }
        
        pbuf[pcount] = new IMUPacket(tempbuf);
        pcount++;
      } 
      
      // COMMAND PACKET WOOHOO
      else if( recvque[rq_read] == PTYPE_COMMAND ){
        //if (USBSERIAL_DEBUG) safePrintln("got command packet oh yuh");
        
        // advance past type byte, discarding result
        readQueueByte();
        
        // copy packet data to temp buffer
        for( int i=0; i<CMD_T_SIZE; i++){
          tempbuf[i] = readQueueByte();
        }
        
        pbuf[pcount] = new CommandPacket(tempbuf);
        pcount++;
      }
      
      // unrecognized packet     
      else {
        if(USBSERIAL_DEBUG) safePrintln(" OUT OF WACHK ON THE PACKET DECODE, discarding entire buffer (for now)");
        break;
      }
    }
    
    // at this point, 'i' bytes should have been read from the recvque, leaving 'rq_size_orig - i' bytes
    // left in the queue, make sure this is the case
    
    //safePrint("after parsing, rq_size = "); safePrint(String(rq_size)); safePrint(", rq_size_orig = "); safePrint(String(rq_size_orig)); 
    //safePrint(", i = "); safePrintln(String(i));
    //safePrint("pcount = "); safePrintln(String(pcount));
    
    if( rq_size == rq_size_orig - i ){
      //safePrintln(" --> queue length sanity check OK!");
    } else {
      //safePrintln(" --> queue length sanity check FAIL!!");
    }
    
    // also check that we decoded as many packets as expected
    if( pcount == num_packets ){
      //safePrintln(" --> packet count sanity check OK!");
      //rbuflen = 0;
      return pcount;
    } else {
      //safePrintln(" --> packet count sanity check FAIL!");
      return 0;
    }
    
  }

  void printPackets() {
    //safePrintln("*** printing packet...");
    if( pcount == 0) return;
    for( int i=0; i<pcount; i++ ){
      if (USBSERIAL_DEBUG) safePrintln(pbuf[i]->toString(true));
    }
    //safePrintln("done");
  }
  
  void deletePackets() {
  if( pcount == 0 ) return;
    for( int i=0; i<pcount; i++ ){
      delete pbuf[i];
    }
    pcount = 0;
  }

  uint8_t readQueueByte() {
    uint8_t ret = 0;
    ret = recvque[rq_read];
    rq_read = (rq_read + 1) % RECV_QUEUE_SIZE;
    rq_size = rq_size - 1;
    return ret; 
  }
  
  void writeQueueByte(uint8_t b) {
    recvque[rq_write] = b;
    rq_write = (rq_write + 1) % RECV_QUEUE_SIZE;
    rq_size = rq_size + 1;
  }

  int numPackets() { return pcount; } 
  
  Packet** packets() { return pbuf; }

  bool available() { return rf69_manager.available(); }
  
  bool sleep() { return rf69.sleep(); }

  void setRetries(uint8_t re) { rf69_manager.setRetries(re); }
  uint8_t getRetries() { return rf69_manager.retries(); }

  uint8_t sbuflen;                          // current number of bytes in send buffer
  uint8_t rbuflen;                          // current number of bytes in receive buffer
  uint8_t sendbuf[SEND_BUF_SIZE];           // send buffer
  uint8_t recvbuf[RECV_BUF_SIZE];           // receive buffer
  uint8_t recvque[RECV_QUEUE_SIZE];         // circular buffer to hold received data for processing
  uint8_t rssi;                             // signal strength of last received message
  uint8_t lastFrom;                         // address last message was from
  int rq_read, rq_write;                   // read and write indices for the circular buffer
  int rq_size;                              // number of bytes in circular buffer
  
  int pcount;                               // how many packets we are holding
  Packet *pbuf[PACKET_BUFFER_NUM_PACKETS];  // the packets we are holding
  
  static RH_RF69 rf69;
  static RHReliableDatagram rf69_manager;
};

RH_RF69 RadioLogger::rf69 = RH_RF69(RFM69_CS, RFM69_INT);
RHReliableDatagram RadioLogger::rf69_manager = RHReliableDatagram(RadioLogger::rf69, NODE_ADDRESS);

#endif
