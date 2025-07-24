#include "modbus/modbus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <string>

class modbusServer{

public:
    modbusServer();
    ~modbusServer();

    bool modbusConnect();
    bool modbusDisConnect();
    bool writeBits(const uint8_t M_BITS[], uint8_t BIT_SIZE);

private:
    modbus_t* ctx;
    int PORT;
    std::string IP;

    const uint16_t UT_BITS_ADDRESS = 0x130;
    const uint16_t UT_BITS_NB = 0x25;
    const uint8_t UT_BITS_TAB[] = { 0xCD, 0x6B, 0xB2, 0x0E, 0x1B };
    const uint16_t UT_BITS_ADDRESS_INVALID_REQUEST_LENGTH = UT_BITS_ADDRESS + 2;

    const uint16_t UT_INPUT_BITS_ADDRESS = 0x1C4;
    const uint16_t UT_INPUT_BITS_NB = 0x16;
    const uint8_t UT_INPUT_BITS_TAB[] = { 0xAC, 0xDB, 0x35 };

    const uint16_t UT_REGISTERS_ADDRESS = 0x160;
    const uint16_t UT_REGISTERS_NB = 0x3;
    const uint16_t UT_REGISTERS_NB_MAX = 0x20;
    const uint16_t UT_REGISTERS_TAB[] = { 0x022B, 0x0001, 0x0064 };

    /* Raise a manual exception when this address is used for the first byte */
    const uint16_t UT_REGISTERS_ADDRESS_SPECIAL = 0x170;
    /* The response of the server will contains an invalid TID or slave */
    const uint16_t UT_REGISTERS_ADDRESS_INVALID_TID_OR_SLAVE = 0x171;
    /* The server will wait for 1 second before replying to test timeout */
    const uint16_t UT_REGISTERS_ADDRESS_SLEEP_500_MS = 0x172;
    /* The server will wait for 5 ms before sending each byte */
    const uint16_t UT_REGISTERS_ADDRESS_BYTE_SLEEP_5_MS = 0x173;

    /* If the following value is used, a bad response is sent.
    It's better to test with a lower value than
    UT_REGISTERS_NB_POINTS to try to raise a segfault. */
    const uint16_t UT_REGISTERS_NB_SPECIAL = 0x2;

    const uint16_t UT_INPUT_REGISTERS_ADDRESS = 0x108;
    const uint16_t UT_INPUT_REGISTERS_NB = 0x1;
    const uint16_t UT_INPUT_REGISTERS_TAB[] = { 0x000A };

};