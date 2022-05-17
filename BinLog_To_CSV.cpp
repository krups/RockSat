// parser program for krepe
// convert binary log files of packets into csv files for post flight analysis

#include <iostream>
#include <fstream>
#include <cstring>
#include <string>


typedef uint8_t byte;

#include "packets.h"

#define DEBUG 0

void usage(char** argv);

int main(int argc, char** argv)
{
  
  if( argc < 2 ){
    usage(argv);
    return 0;
  }
  
  std::string inFileName(argv[1]);
  std::ifstream inFile(inFileName);
  //std::vector<std::string> outFileNames;
  
  if( !inFile.is_open() ){
    std::cout << "Could not open input file " << argv[1] << std::endl;
    return 1;
  }

  if( DEBUG ) std::cout << "INFO: Opened input file " << argv[1] << std::endl;

  // detect file length and read in entire file to memory
  inFile.seekg(0, inFile.end);
  unsigned long length = inFile.tellg();
  inFile.seekg(0, inFile.beg);
  
  if( DEBUG ) std::cout << "INFO: File length is " << length << " bytes" << std::endl;
  
  // buffer to hold data as we parse it
  char *buffer = new char[length];
  // keep track of current position (cp) in buffer
  unsigned long cp = 0;

  if( DEBUG ) std::cout << "INFO: Reading in data...";
  
  // read data as a block and close input file
  inFile.read (buffer, length);
  inFile.close();
  
  if( DEBUG ) std::cout << "done" << std::endl;
  
  
  if( DEBUG ) std::cout << "counting packets...";
  int num_packets = 0;
    
  unsigned long i = 0;
  
  for( i=0; i<length; i++ ){
    if( DEBUG ) std::cout << "i=" << i << std::endl;
    //std::cout << "reading in byte" << buffer[i] << ", i=" << i << std::endl;
    switch((char)buffer[i]){
      case PTYPE_TELEM:
        num_packets++;
        i += TELEM_T_SIZE ;
        break;
      case PTYPE_ACCELSTATS:
        num_packets++;
        i += ACC_STAT_T_SIZE;
        break;
      case PTYPE_TC:
        num_packets++;
        i += TC_T_SIZE;
        break;
      case PTYPE_ACCEL:
        num_packets++;
        i += ACC_T_SIZE;
        break;
      case PTYPE_COMMAND:
        num_packets++;
        i += CMD_T_SIZE;
        break;
      case PTYPE_IMU:
        num_packets++;
        i += IMU_T_SIZE;
        break;
      default:
        std::cout << "[ERROR]: lost track of what packet was which n where n what not, stopping" << std::endl;
        i=length;
    }
    
    if( i > length ){
      std::cout <<  "expecting another " << (i-length) << " bytes" << std::endl;
    }
  }
  
  //std::cout << "counted " << num_packets << " packets in file" << std::endl;
  
  
  
  
  unsigned long pcount = 0;   // current count of instantiated packets
  unsigned long idx = 0;  // current position within buffer
  
  while( pcount < num_packets ){  
    
    // telemetry packet
    if( buffer[idx] == PTYPE_TELEM ){
      TelemPacket *p = new TelemPacket((uint8_t*)&buffer[idx+1]);
      pcount++;
      idx += TELEM_T_SIZE+1;
      
      // now print this packet
      std::cout << p->t() << ", " << p->batt() << ", " << p->tc1_temp() << ", " << p->tc2_temp() << ", " << (int)p->ir_sig() << ", " <<  (int)p->log_num() << std::endl;
      
      delete p;
    } 
    
    // min/max stats over a period of time from the high g accelerometer
    else if( buffer[idx] == PTYPE_ACCELSTATS ){
      AccStatsPacket *p = new AccStatsPacket((uint8_t*)&buffer[idx+1]);
      pcount++;
      idx += ACC_STAT_T_SIZE+1;
      
      // print
      
      delete p;
    } 
    
    // timestamped thermocouple packet (containing TC_COUNT readings)
    else if( buffer[idx] == PTYPE_TC ){
      TcPacket *p = new TcPacket((uint8_t*)&buffer[idx+1]);
      pcount++;
      idx += TC_T_SIZE+1;

      // print the tc data
      std::cout << p->time() << ", ";
      for( int i=0; i<TC_COUNT; i++ ){
        std::cout << p->data()[i] << (i==TC_COUNT-1 ? "" : ", ");
      }
      std::cout << std::endl;
      
      delete p;
    } 
    
    // singe timestamped high g accelerometer reading
    else if( buffer[idx] == PTYPE_ACCEL ){
      AccPacket *p = new AccPacket((uint8_t*)&buffer[idx+1]);
      pcount++;
      idx += ACC_T_SIZE+1;
      
      // print the acc packet

      std::cout << p->t() << ", " << p->x() << ", " << p->y() << ", " << p->z() << std::endl;
      
      delete p;
    } 
    
    // singe timestamped imu reading
    else if( buffer[idx] == PTYPE_IMU ){
      IMUPacket *p = new IMUPacket((uint8_t*)&buffer[idx+1]);
      pcount++;
      idx += IMU_T_SIZE+1;
      
      // print the imu data 
      std::cout <<  *(float*)(&p->data()[0])  << ", " <<  *(float*)(&p->data()[4])  << ", " <<  *(float*)(&p->data()[8]) << ", ";
      std::cout <<  *(float*)(&p->data()[12]) << ", " <<  *(float*)(&p->data()[16]) << ", " <<  *(float*)(&p->data()[20]) << ", ";
      std::cout <<  *(float*)(&p->data()[24]) << ", " <<  *(float*)(&p->data()[28]) << ", " <<  *(float*)(&p->data()[32]) << ", " << std::endl;

      delete p;
    } 
    
    // COMMAND PACKET WOOHOO
    else if( buffer[idx] == PTYPE_COMMAND ){
      CommandPacket *p = new CommandPacket((uint8_t*)&buffer[idx+1]);
      pcount++;
      idx += CMD_T_SIZE+1;
      
      // print
      
      delete p;
    }
    
    // unrecognized packet     
    else {
      std::cout << "[ERROR] OUT OF WACHK ON THE PACKET DECODE, discarding entire buffer (for now)" << std::endl;
      break;
    }
  }
  
  if( pcount == num_packets ){
    if (DEBUG) std::cout << "parsed all packets" << std::endl;
    //safePrintln(" --> parsed all packets");
  } else {
  
  }
  
  
  
  
  
  //safePrint("received "); safePrint(String(num_packets)); safePrint(" packets, ");
  //safePrint("   i="); safePrint(String(i)); safePrint(", len="); safePrintln(String(len));
  
    
  
  
  return 0;
}

void usage(char** argv)
{
  std::cout << "USAGE:" << std::endl;
  std::cout << "  " << argv[0] << " logfile.dat" << std::endl;
}
