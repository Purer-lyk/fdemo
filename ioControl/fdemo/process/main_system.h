#ifndef FIREDEMO_MAIN_SYSTEM_H
#define FIREDEMO_MAIN_SYSTEM_H

#include "main.h"
#include "control.h"
#include "paddle_detection.h"
#include "modbus_m_client.h"
#include "modbus_m_server.h"
#include "tcp_client.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>

using namespace cv;

enum status{
    RESET=1,
    STILL,
    SCAN,
    CONTROL,
    TRIGGER
};

class mainSystem{
public:
	mainSystem();
	~mainSystem();
	void run();
	void stop();
	
private:
	enum status systemStatus;

	int srcW, srcH;
	int rangePosx, rangePosy;
	int accumulateTrace;
	int noTraceOffset;
	
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
	double triggerPos56, triggerPos2324;
	int fireStatus;//0-no,1-small,2-middle,3-big
	int triggerCount;
	int scanOrTrace;//0-nothing,1-scaning,2-tracing
	int scanUD, scanLR;
	bool lastTrigger;
	bool lastScan;
	float scanState;
	float scanOffset;
	int triggerDirect;
	
	std::string modelFile, reModelFile;
	bool rstOrNot;
	double imgLight;
	int ycOffset, xcOffset;
	float yawLimit, pitchLimit;
	float gthreshold, rethreshold;
	int leftDirect, upDirect, rightDirect, downDirect;
	bool uvInit;
	std::vector<float> distinct;
	std::string modbusIP;
	int modbusPORT;
	bool flipFlag;
	int device;
	
	bool findTarget(const std::vector<Object>& objs, const int& uv);
	void tinyModify56(int diffCx);
	bool feedbackControlpp(const std::vector<Object>& objs);
	void cameraScan();
	void stdScan();
	void singleScan();
	void upAndDownTrigger();

	bool modbusTransfer();
	bool tcpTransfer();
	bool modbusReply();

	void checkModbus();
	void checkTcp();
	void checkServer();

	void obtainPos();
	void printPos();
	void readParams();
	void judgeStatus(int uvOut, int smokeOut, bool ppOut);
	void loseTarget();
	void resetStatus();
	void unTrigger();
	
	Mat frame;
	Mat dst;
	WiringControl* controller;
	paddleDetector* detector;
	modbusClient* modbuser;
	modbusServer* server;
	tcpClient* tcper;
	
	Object lastTrace;

};



#endif //FIREDEMO_MAIN_SYSTEM_H
