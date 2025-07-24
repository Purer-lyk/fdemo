#ifndef FIREDEMO_MAIN_SYSTEM_H
#define FIREDEMO_MAIN_SYSTEM_H

#include "control.h"
#include "paddle_detection.h"
#include "modbus_m_client.h"
#include "modbus_m_server.h"
#include "tcp_client.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>


using namespace cv;

class mainSystem{
public:
	mainSystem();
	~mainSystem();
	void run();
	
private:
	int srcW, srcH;
	int rangePosx, rangePosy;
	int accumulateTrace;
	
	int modbusReconnect;
	int tcpReconnect;
	int serverReconnect;

	bool modbusTcpStatus;
	bool tcpStatus;
	bool serverStatus;

	int modbusInterval;
	int tcpInterval;
	int serverInterval;

	double currentPos56, currentPos2324;
	int fireStatus;//0-no,1-small,2-middle,3-big
	int triggerCount;
	int scanOrTrace;//0-nothing,1-scaning,2-tracing
	int scanUD, scanLR;
	bool lastTrigger;
	
	std::string modelFile;
	bool rstOrNot;
	double imgLight;
	int ycOffset;
	int yawLimit, pitchLimit;
	float gthreshold;
	int leftDirect, upDirect, rightDirect, downDirect;
	int yawInitPos, pitchInitPos;
	std::vector<float> distinct;
	std::string modbusIP;
	int modbusPORT;
	bool flipFlag;
	int device;
	
	bool feedbackControlpp(const std::vector<Object>& objs, const int& uv);
	void cameraScan();
	void upAndDownTrigger(int randomCurrent);

	bool modbusTransfer();
	bool tcpTransfer();
	bool modbusReply();

	void checkModbus();
	void checkTcp();
	void checkServer();

	void obtainPos();
	void readParams();
	void judgeStatus(int uvOut, int smokeOut, bool ppOut);
	void loseTarget();
	
	Mat frame;
	Mat dst;
	WiringControl* controller;
	paddleDetector* detector;
	modbusClient* modbuser;
	modbusServer* server;
	Object lastTrace;
	tcpClient* tcper;

};



#endif //FIREDEMO_MAIN_SYSTEM_H
