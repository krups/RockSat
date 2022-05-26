// decompress helper for iridium packets
// krepe 2020 matt ruffner

#include <iostream>
#include <fstream>
#include <cstring>
#include <string>

#include "brieflz.h"

#define MAX_DEPACKED_SIZE 5000

int main(int argc, char ** argv)
{

  if( argc < 3 ){
    std::cout << "need an input file and output file" << std::endl;
    return 1;
  }

  std::cout << "starting" << std::endl;

  std::string inFileName(argv[1]);
  std::string outFileName(argv[2]);
  std::ifstream inFile(inFileName);

  if( !inFile.is_open() ){
    std::cout << "Could not open input file : " << inFileName << std::endl;
    return 1;
  }

  // detect file length and read in entire file to memory   
  inFile.seekg(0, inFile.end);
  unsigned long length = inFile.tellg();
  inFile.seekg(0, inFile.beg);

  std::cout << "INFO: File length is " << length << " bytes" << std::endl;
  
  // buffer to hold data as we parse 
  char *buffer = new char[length];
  // keep track of current position (cp) in 
  unsigned long cp = 0;

  std::cout << "INFO: Reading in data...";

  // read data as a block and close input 
  inFile.read (buffer, length);
  inFile.close();

  std::cout << "done" << std::endl;

  uint16_t dclen = *(uint16_t*)(&buffer[0]);

  std::cout << "decompressed data size expected to be: " << dclen << " bytes." << std::endl;
  
  unsigned char  depacked[dclen];

  std::cout << "trying to depack...";

  int len = blz_depack_safe(buffer+2, length, depacked, dclen);

  std::cout << "done" << std::endl;

  std::ofstream outFile(outFileName, std::ofstream::binary);

  if( !outFile.is_open() ){
    std::cout << "Could not open output file : " << inFileName << std::endl;
    return 1;
  }

  outFile.write((const char *)depacked, len);

  outFile.close();

  std::cout << "wrote " << len << " bytes to output file" << std::endl;
  
}
