#include "modbus_m_server.h"

modbusServer::modbusServer(std::string _ip, int _port):
IP(_ip),
PORT(_port),
ctx(nullptr),
s(-1)
{

}

modbusServer::~modbusServer(){

}

bool modbusServer::modbusConnect(){
    ctx = modbus_new_tcp(IP.c_str(), PORT);
    if(ctx==nullptr){
        printf("connect error!");
        return false;
    }
    modbus_set_debug(ctx, TRUE);
    modbus_set_error_recovery(ctx, MODBUS_ERROR_RECOVERY_LINK);

    s = modbus_tcp_listen(ctx, 1);

    //阻塞模式，等待主站连接请求，没有则继续等待
	// modbus_tcp_accept(mb, &socketMb);
	
    if(s==-1){
        printf("listen error!");
        modbus_free(ctx);
		ctx = nullptr;
        return false;
    }
    else
	{
		// 设置为非阻塞
		int flags = fcntl(socketMb, F_GETFL, 0);
		fcntl(socketMb, F_SETFL, flags | O_NONBLOCK);
	}

	// 设置服务端等待客户端请求超时时间
	modbus_set_indication_timeout(mb, 3, 0);
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
    if(ctx==nullptr){
        printf("cannot write until connect!")
        return false;
    }
    //MODBUS_TCP_MAX_ADU_LENGTH在标准库里面定义
	uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH] = {0};
    mbMapping = modbus_mapping_new_start_address(UT_BITS_ADDRESS,
                                                  UT_BITS_NB,
                                                  UT_INPUT_BITS_ADDRESS,
                                                  UT_INPUT_BITS_NB,
                                                  UT_REGISTERS_ADDRESS,
                                                  UT_REGISTERS_NB_MAX,
                                                  UT_INPUT_REGISTERS_ADDRESS,
                                                  UT_INPUT_REGISTERS_NB);
                                                 
    /* Initialize input values that's can be only done server side. */
    modbus_set_bits_from_bytes(
        mb_mapping->tab_input_bits, 0, UT_INPUT_BITS_NB, UT_INPUT_BITS_TAB);

    /* Initialize values of INPUT REGISTERS */
    for (i = 0; i < UT_INPUT_REGISTERS_NB; i++) {
        mb_mapping->tab_input_registers[i] = UT_INPUT_REGISTERS_TAB[i];
    }

    int rc = -1;
    rc = modbus_receive(ctx, query);
    if(rc==-1){
        printf("receive error!");
    }
    modbus_reply(ctx, query, rc, mbMapping);
    // rc = modbus_reply(ctx, query, rc, mb_mapping);
}