//
// Created by liyankuan on 2025/7/10.
//

#ifndef FIREDEMO_TCP_CLIENT_H
#define FIREDEMO_TCP_CLIENT_H

#include <cstdio>
#inlcude <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <cstring>

class tcpClient{
public:
	tcpClient(std::string IP, int PORT);
	~tcpClient();
	bool connectServer();
	bool disconnectServer();
	bool writeBits(const uint8_t M_BITS[], uint8_t BIT_SIZE);
	
private:
	int sockfd;
	struct  sockaddr_in addr;
	std::string ip;
	int port;
	
};


#endif //FIREDEMO_TCP_CLIENT_H
