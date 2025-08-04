#ifndef MODBUS_M_CLIENT_H
#define MODBUS_M_CLIENT_H

#include "modbus/modbus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <string>


#define BUG_REPORT(_cond, _format, _args...) \
    printf(                                  \
        "\nLine %d: assertion error for '%s': " _format "\n", __LINE__, #_cond, ##_args)

#define ASSERT_TRUE(_cond, _format, __args...)    \
    {                                             \
        if (_cond) {                              \
            printf("OK\n");                       \
        } else {                                  \
            BUG_REPORT(_cond, _format, ##__args); \
        }                                         \
    };
    


class modbusClient
{
public:
    modbusClient(std::string _ip, int _port);
    ~modbusClient();
		
    bool modbusConnect();
    bool modbusDisConnect();
    bool writeBits(const uint8_t M_BITS[], uint8_t BIT_SIZE);
    void writeRegisters();
	
private:
    std::string ip;
    int port;
    modbus_t* ctx;
    int nb_points;
    uint8_t *tab_rp_bits;
    uint32_t old_response_to_sec;
    uint32_t old_response_to_usec;
    uint32_t new_response_to_sec;
    uint32_t new_response_to_usec;	
    
    const uint16_t UT_BITS_ADDRESS = 0x130;
    const uint16_t UT_BITS_NB = 0x25;
    const uint8_t UT_BITS_TAB[5] = { 0xCD, 0x6B, 0xB2, 0x0E, 0x1B };
    /*const uint16_t UT_BITS_NB = 0x08;
    const uint8_t UT_BITS_TAB[] = { 0x01 };*/
    const uint16_t UT_BITS_ADDRESS_INVALID_REQUEST_LENGTH = UT_BITS_ADDRESS + 2;

    const uint16_t UT_INPUT_BITS_ADDRESS = 0x1C4;
    const uint16_t UT_INPUT_BITS_NB = 0x16;
    const uint8_t UT_INPUT_BITS_TAB[3] = { 0xAC, 0xDB, 0x35 };
};


#endif
