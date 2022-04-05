#ifndef COMMANDS_H
#define COMMANDS_H

/************************************************************************************************
* Commands
*/
// typically sent from station -> capsule
#define CMDID_IR_ON             1       // power on iridium
#define CMDID_IR_OFF            2       // power off iridium
#define CMDID_IR_BP             3       // build packet
#define CMDID_IR_SP             4       // send packet
#define CMDID_IR_TEST           5       // send a "hello world" packet
#define CMDID_ACTIVATE          6       // simulate activation
#define CMDID_SLEEP             7       // for testing, initiate sleeep

// logging
#define CMD_ID_START_ISMLOG     51
#define CMD_ID_STOP_ISMLOG      52

// create a new set of logfiles on the SD card
#define CMD_ID_RESTART_SD       53

// bidirectional paramaterless commands
#define CMDID_OK                101       // acknowledge command for an ack packet or keepalive 
#define CMDID_ACT_DETECTED      102       // temp and cap sense have detected activation 


// setting log sources
#define CMDID_LOG_SOURCE        200       // set log sources
                                        // each byte of the payload indicates sources to enable
                                        // ex data[] = {1,2,3,4} enables all 4 log sources
#define CMDID_LOG_PERIOD        201       // set log period
                                        // arg byte specifies which log source's interval to set. 
                                        // data[] is interval in milliseconds.


// queue length for command receiving thread
#define MAX_CMDQ_SIZE           20      // max command packets waiting to be processed by command thread


#endif
