#include "tcp_client.h"

tcpClient::tcpClient(std::string IP, int PORT):
ip(IP),
port(PORT)
{
	sockfd = socket(PE_INET, SOCK_STREAM,0);
}

tcpClient::~tcpClient(){

}

bool tcpClient::connectServer(){
	memset(&addr, 0, sizeof(addr));
	addr.sin_family=AF_INET;
	addr.sin_addr.s_addr=inet_addr(ip.c_str());
	addr.sin_port=htons(port);
	
	if(connect(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr))<0){
		return false;
	}
	return true;
}

bool tcpClient::disconnectServer(){
	close(sockfd);
	return true;
}

bool tcpClient::writeBits(const uint8_t *M_BITS, uint8_t BIT_SIZE){
	int SEND_SIZE = BIT_SIZE+3;//帧头起始符,数据长度,帧头结束符
	uint8_t buf[SEND_SIZE];
	buf[0] = 0x7b;
	buf[1] = BIT_SIZE;
	buf[2] = 0x7d;
	for(int i=0;i<BIT_SIZE;i++){
		buf[i+3] = M_BITS[i];
	}
	send(sockfd, M_BITS,SEND_SIZE,0);
}
