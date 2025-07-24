#include "modbus_m_server.h"

modbusServer::modbusServer(std::string _ip, int _port):
IP(_ip),
PORT(_port),
ctx(nullptr)
{

}

modbusServer::~modbusServer(){

}

bool modbusServer::modbusConnect(){
    ctx = modbus_new_tcp(IP.c_str(), PORT);
    modbus_set_debug(ctx, TRUE);
    modbus_set_error_recovery(ctx, MODBUS_ERROR_RECOVERY_LINK);
    modbus_get_response_timeout(ctx, &old_response_to_sec, &old_response_to_usec);
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        ctx = nullptr;
        return false;
    }

    
    
}

bool modbusServer::modbusDisConnect(){
     /* Free the memory */
    free(tab_rp_bits);

    /* Close the connection */
    modbus_close(ctx);
    modbus_free(ctx);
    return true;
}

bool modbusServer::writeBits(const uint8_t M_BITS[], uint8_t BIT_SIZE){
    int s = -1;
    s = modbus_tcp_listen(ctx, 1);
    modbus_tcp_accept(ctx, &s);

    int rc;
    rc = modbus_receive(ctx, query);

    modbus_mapping_t *mb_mapping;
    mb_mapping = modbus_mapping_new_start_address(UT_BITS_ADDRESS,
                                                  UT_BITS_NB,
                                                  UT_INPUT_BITS_ADDRESS,
                                                  UT_INPUT_BITS_NB,
                                                  UT_REGISTERS_ADDRESS,
                                                  UT_REGISTERS_NB_MAX,
                                                  UT_INPUT_REGISTERS_ADDRESS,
                                                  UT_INPUT_REGISTERS_NB);

    rc = modbus_reply(ctx, query, rc, mb_mapping);
}