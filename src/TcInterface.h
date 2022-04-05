#ifndef TcInterface_h
#define TcInterface_h


#include <Adafruit_MAX31856.h>
#include "config.h"
#include "packets.h"

// Encapsulate selection and reading of thermocouple values
class TcInterface
{
    public:
        TcInterface( char* tc_types, // String of thermocouple types (B, E, J, K, N, R, S, or T)
            int cs_tc1 = CS_TC1, // Chip select pin for "TC1" MAX31856 
            int cs_tc2 = CS_TC2, // Chip select pin for "TC2" MAX31856 
            int mux0 = MUX0,  // Input for mux to select thermocouple output
            int mux1 = MUX1,  // Input for mux to select thermocouple output
            int tc1_fault = TC1_FAULT, // Fault pin for "TC1" MAX31856
            int tc2_fault = TC2_FAULT // Fault pin for "TC2" MAX31856
        );
        bool enable(void);
        void disable(void);
        bool read_all(tcv_t *dest, bool force);

        // Checks if either sensor has a fault.
        // Returns a uint16_t where the high byte is the fault in TC1
        // and the low byte is the fault in TC2
        uint16_t check_faults(void);

        // Sets appropriate mux outputs to select given tc according to the following mapping:
        //  MUX0 | MUX1 | TC
        //--------------------
        //    0  |  0   | 1, 5
        //    0  |  1   | 2, 6
        //    1  |  0   | 3, 7
        //    1  |  1   | 4, 8
        void select_tc(int tc);

        
    private:
        bool enabled;
        char* tc_types;
        int cs_tc1; 
        int cs_tc2;
        Adafruit_MAX31856 max1;// U13 TC to Digital - connected to IC1 mux output of TC 1-4
        Adafruit_MAX31856 max2;// U12 TC to Digital - connected to IC3 mux output of TC 5-8
        int mux0;// Input for mux to select thermocouple output
        int mux1;// Input for mux to select thermocouple output
        int tc1_fault; // Fault pin for "TC1" MAX31856
        int tc2_fault;         
        char tc_type_lookup[TC_COUNT];
        void set_types_from_chars(char* tc_types);
        Adafruit_MAX31856& get_max_from_tc(int tc);
};

#endif
