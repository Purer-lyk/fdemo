#ifndef FIREDEMO_MAIN_SYSTEM_H
#define FIREDEMO_MAIN_SYSTEM_H

#include "control.h"
#include "paddle_detection.h"
#include "modbus_m_client.h"
#include "tcp_client.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>


using namespace cv;

class mainSystem{
public:
	mainSystem(std::string model_file);
	~mainSystem();
	void run();
	
private:
	int srcW, srcH;
	int rangePosx, rangePosy;
	int direct_56, direct_2324;
	int accumulateTrace;
	
	int modbusReconnect;
	int tcpReconnect;
	bool modbusTcpStatus;
	bool tcpStatus;
	int modbusInterval;
	int tcpInterval;
	
	double currentPos;
	int fireStatus;//0-no,1-small,2-middle,3-big
	int triggerCount;
	int rstTick;
	int scanOrTrace;//0-nothing,1-scaning,2-tracing
	
	bool rstOrNot;
	double imgLight;
	int ycOffset;
	int scanYaw, scanPitch;
	int yawLimit;
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

	void checkModbus();
	void checkTcp();

	void obtainPos();
	void readParams();
	void judgeStatus(int uvOut, int smokeOut, bool ppOut);
	void loseTarget();
	
	Mat frame;
	Mat dst;
	WiringControl* controller;
	paddleDetector* detector;
	modbusClient* modbuser;
	Object lastTrace;
	tcpClient* tcper;

};



#endif //FIREDEMO_MAIN_SYSTEM_H
