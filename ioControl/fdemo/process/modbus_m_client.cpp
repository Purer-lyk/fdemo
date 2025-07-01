#include "modbus_m_client.h"
#include <iostream>

modbusClient::modbusClient(std::string _ip, int _port):
ip(_ip),
port(_port),
tab_rp_bits(nullptr),
ctx(nullptr)
{
    ip.erase(ip.size()-1);
}

modbusClient::~modbusClient(){
	
}

bool modbusClient::modbusConnect(){
    ctx = modbus_new_tcp(ip.c_str(), port);
    modbus_set_debug(ctx, TRUE);
    modbus_set_error_recovery(ctx, MODBUS_ERROR_RECOVERY_LINK);
    modbus_get_response_timeout(ctx, &old_response_to_sec, &old_response_to_usec);
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        ctx = nullptr;
        return false;
    }
    /* Allocate and initialize the memory to store the bits */
    nb_points = (UT_BITS_NB > UT_INPUT_BITS_NB) ? UT_BITS_NB : UT_INPUT_BITS_NB;
    tab_rp_bits = (uint8_t *) malloc(nb_points * sizeof(uint8_t));
    memset(tab_rp_bits, 0, nb_points * sizeof(uint8_t));
    
    printf("** UNIT TESTING **\n");

    printf("1/1 No response timeout modification on connect: ");
    modbus_get_response_timeout(ctx, &new_response_to_sec, &new_response_to_usec);
    ASSERT_TRUE(old_response_to_sec == new_response_to_sec &&
                    old_response_to_usec == new_response_to_usec,
                "");

    printf("\nTEST WRITE/READ:\n");
    return true;
}

bool modbusClient::modbusDisConnect(){
    /* Free the memory */
    free(tab_rp_bits);

    /* Close the connection */
    modbus_close(ctx);
    modbus_free(ctx);
    return true;
}

void modbusClient::writeBits(const uint8_t M_BITS[], uint8_t BIT_SIZE){
	if(ctx==nullptr) return;
	//BIT_SIZE = UT_BITS_NB;
	
	int rc;
	uint8_t value;
	uint8_t tab_value[BIT_SIZE];
	modbus_set_bits_from_bytes(tab_value, 0, BIT_SIZE, M_BITS);
	rc = modbus_write_bits(ctx, UT_BITS_ADDRESS, BIT_SIZE, tab_value);
	printf("1/2 modbus_write_bits: ");
	ASSERT_TRUE(rc == BIT_SIZE, "");
	
	
	rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, BIT_SIZE, tab_rp_bits);
    printf("2/2 modbus_read_bits: ");
    ASSERT_TRUE(rc == BIT_SIZE, "FAILED (nb points %d)\n", rc);

    int i = 0;
    nb_points = BIT_SIZE;
    while (nb_points > 0) {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;
        printf("nb_bits:%d\n", nb_bits);

        value = modbus_get_byte_from_bits(tab_rp_bits, i * 8, nb_bits);
        ASSERT_TRUE(
            value == M_BITS[i], "FAILED (%0X != %0X)\n", value, M_BITS[i]);

        nb_points -= nb_bits;
        i++;
    }
}



void modbusClient::writeRegisters(){
}
