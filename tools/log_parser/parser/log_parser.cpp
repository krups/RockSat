// parser program for krepe
// convert binary log files of packets into csv files for post flight analysis

#include <iostream>
#include <ostream>
#include <fstream>
#include <cstring>
#include <string>


#include "packet.h"

#define DEBUG 1
#define DEBUG_SIZEOF 0
#define DEBUG_PACKET_SIZE 1

void usage(char** argv);
void printRmc(rmc_t data, std::ostream &stream);
void printGga(gga_t data, std::ostream &stream);
void printTc(tc_t data, std::ostream &stream);
void printAcc(acc_t data, std::ostream &stream);
void printImu(imu_t data, std::ostream &stream);
void printSpec(spec_t data, std::ostream &stream);
void printBar(bar_t data, std::ostream &stream);
void printPacket(packet_t data, std::ostream &stream);


int main(int argc, char** argv)
{
  int num_packets = 0;
  unsigned long i = 0;
  int block_id = 0;

  if( argc < 3 ){
    usage(argv);
    return 0;
  }

  //for( int i=0; i<argc; i++ )
  //  std::cout << i << " '" << argv[i] << "'" << std::endl;
  //std::cout << "strcmp(argv[1], \"log\") = " << strcmp(argv[1], "log") << std::endl;
  //std::cout << "strcmp(argv[1], \"packet\") = " << strcmp(argv[1], "packet") << std::endl;

  if( !( strcmp(argv[1], "log")==0 || strcmp(argv[1],"packet")==0) ){
    usage(argv);
    return 0;
  }

  std::string inFileName(argv[2]);
  std::ifstream inFile(inFileName);
  std::ofstream outFile;
  //std::vector<std::string> outFileNames;

  if( DEBUG_SIZEOF ){
    int s = sizeof(float);
    std::cout << "sizeof(float) = " << sizeof (float) << std::endl;
    std::cout << "sizeof(unsigned long) = " << sizeof (unsigned long) << std::endl;
    std::cout << "sizeof(int) = " << sizeof (int) << std::endl;
  }

  if( !inFile.is_open() ){
    std::cout << "Could not open input file " << argv[2] << std::endl;
    return 1;
  }

  if( DEBUG ) std::cout << "INFO: Opened input file " << argv[2] << std::endl;

  // detect file length and read in entire file to memory
  inFile.seekg(0, inFile.end);
  unsigned long length = inFile.tellg();
  inFile.seekg(0, inFile.beg);

  if( DEBUG ) std::cout << "INFO: File length is " << length << " bytes" << std::endl;

  // buffer to hold data as we parse it
  char *buffer = new char[length];
  // keep track of current position (cp) in buffer
  unsigned long cp = 0;
  int offset = 0;

  if( DEBUG ) std::cout << "INFO: Reading in data...";

  // read data as a block and close input file
  inFile.read (buffer, length);
  inFile.close();

  if( DEBUG ) std::cout << "done" << std::endl;


  unsigned long block_size = 0;

  // only read a start offset if this is a logfile and not a packet
  if( strcmp(argv[1],"log")==0 ){
    // read first two bytes to determine the block size
    block_size = *(uint16_t*)(&buffer[0]);
  }

  if( DEBUG ) std::cout << "block size is " << block_size << std::endl;

  // loop through the file (which is now in memory)
  for( i=block_size; i<length; i++ ){

    if( buffer[i] == PTYPE_GGA ){
      offset = sizeof(gga_t);
      //if( DEBUG_PACKET_SIZE ) std::cout << "sizeof(gga_t) is " << offset << std::endl;
      gga_t data;
      memcpy(&data, &buffer[i+1], offset);
      i += offset;

      if( DEBUG ) printGga(data, std::cout);

    } else if( buffer[i] == PTYPE_RMC ){
      offset = sizeof(rmc_t);
      rmc_t data;
      memcpy(&data, &buffer[i+1], offset);
      i += offset;

      if( DEBUG ) printRmc(data, std::cout);

    } else if( buffer[i] == PTYPE_ACC ){
      offset = sizeof(acc_t);
      acc_t data;
      memcpy(&data, &buffer[i+1], offset);
      i += offset;

      if( DEBUG ) printAcc(data, std::cout);

    } else if( buffer[i] == PTYPE_IMU ){
      offset = sizeof(imu_t);
      imu_t data;
      memcpy(&data, &buffer[i+1], offset);
      i += offset;

      if( DEBUG ) printImu(data, std::cout);

    } else if( buffer[i] == PTYPE_TC ){
      offset = sizeof(tc_t);
      tc_t data;
      memcpy(&data, &buffer[i+1], offset);
      i += offset;

      if( DEBUG ) printTc(data, std::cout);

    } else if( buffer[i] == PTYPE_SPEC){
      offset = sizeof(spec_t);
      spec_t data;
      memcpy(&data, &buffer[i+1], offset);
      i += offset;

      if( DEBUG ) printSpec(data, std::cout);

    } else if( buffer[i] == PTYPE_BAR ){
      offset = sizeof(bar_t);
      bar_t data;
      memcpy(&data, &buffer[i+1], offset);
      i += offset;

      if( DEBUG ) printBar(data, std::cout);

    } else if( buffer[i] == PTYPE_PACKET ){
      offset = sizeof(packet_t);
      packet_t data;
      memcpy(&data, &buffer[i+1], offset);
      i += offset;

      if( DEBUG ) printPacket(data, std::cout);

      if( data.size < 100000 ){ // iridium packet should not be this big
        std::string name = inFileName + "_packet_" + std::to_string(num_packets) + ".bin";
        outFile.open (name, std::ios::out | std::ios::binary);
        outFile.write(data.data, data.size);
        outFile.close();
        std::cout << "writing iridium packet to " << name << std::endl;
      }

      num_packets++;
    }

  }

  //std::cout << "counted " << num_packets << " packets in file" << std::endl;




  unsigned long pcount = 0;   // current count of instantiated packets
  unsigned long idx = 0;  // current position within buffer


  return 0;
}

void usage(char** argv)
{
  std::cout << "USAGE:" << std::endl;
  std::cout << "  " << argv[0] << " [log | packet] input_file" << std::endl;
}

void printRmc(rmc_t data, std::ostream &stream) {
  stream << "RMC: " << data.t / (1000.0) << ", " << data.time[0] << ":" << data.time[1] << ":" <<
            data.time[2] << "." << data.time[3] << ", " << data.lat << ", " <<
            data.lon << ", " << data.speed << ", " << data.course << std::endl;
}

void printGga(gga_t data, std::ostream &stream) {
  stream << "GGA: " << data.t / (1000.0) << ", " << data.time[0] << ":" << data.time[1] << ":" <<
            data.time[2] << "." << data.time[3] << ", " << data.lat << ", " <<
            data.lon << ", " << data.hdop << ", " << data.alt << std::endl;
}

void printTc(tc_t data, std::ostream &stream) {
  stream << "TC: " << ((float)data.t) / (1000.0 / (float)TIME_SCALE) << ", " << ((float)data.data[0])/((float)UNIT_SCALE) << ", " << ((float)data.data[1])/((float)UNIT_SCALE) << ", " <<
            ((float)data.data[2])/((float)UNIT_SCALE) << ", " << ((float)data.data[3])/((float)UNIT_SCALE) << ", " << ((float)data.data[4])/((float)UNIT_SCALE) << ", " <<
            ((float)data.data[5])/((float)UNIT_SCALE) <<  std::endl;
}

void printAcc(acc_t data, std::ostream &stream) {
  stream << "ACC: " << ((float)data.t) / (1000.0 / (float)TIME_SCALE) << ", " << ((float)data.data[0])/((float)UNIT_SCALE) << ", " << ((float)data.data[1]) / ((float)UNIT_SCALE) << ", " <<
            ((float)data.data[2])/((float)UNIT_SCALE) << std::endl;
}

void printImu(imu_t data, std::ostream &stream) {
  stream << "IMU: " << ((float)data.t)/ (1000.0 / (float)TIME_SCALE) << ", " << ((float)data.data[0])/((float)UNIT_SCALE) << ", " << ((float)data.data[1])/((float)UNIT_SCALE) << ", " <<
            ((float)data.data[2])/((float)UNIT_SCALE) << ", " << ((float)data.data[3])/((float)UNIT_SCALE) << "," << ((float)data.data[4])/((float)UNIT_SCALE) << ", " <<
            ((float)data.data[5])/((float)UNIT_SCALE) <<  std::endl;
}

void printSpec(spec_t data, std::ostream &stream) {
  stream << "SPEC: " << ((float)data.t) / (1000.0 )  << ", " << data.ch1 << ", " << data.ch2 << std::endl;
}

void printBar(bar_t data, std::ostream &stream) {
  stream << "BAR: " << ((float)data.t) / (1000.0 / (float)TIME_SCALE) << ", " << ((float)data.prs)/((float)UNIT_SCALE) << ", " << ((float)data.tmp)/((float)UNIT_SCALE) << ", " << ((float)data.alt)/((float)UNIT_SCALE) << std::endl;
}

void printPacket(packet_t data, std::ostream &stream) {
  stream << "COMP: " << data.t << ", compressed iridium packet, size: " << data.size << std::endl;
}

