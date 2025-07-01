#ifndef FIREDEMO_MAIN_SYSTEM_H
#define FIREDEMO_MAIN_SYSTEM_H

#include "control.h"
#include "paddle_detection.h"
#include "modbus_m_client.h"
#include <opencv2/opencv.hpp>
#include <chrono>

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
	bool modbusTcpStatus;
	
	double currentPos;
	int fireStatus;//0-no,1-small,2-middle,3-big
	int triggerCount;
	int rstTick;
	
	bool rstOrNot;
	double imgLight;
	int ycOffset;
	int scanYaw, scanPitch;
	float gthreshold;
	int leftDirect, upDirect, rightDirect, downDirect;
	int yawInitPos, pitchInitPos;
	float distinct;
	std::string modbusIP;
	int modbusPORT;
	bool flipFlag;
	
	bool feedbackControlpp(const std::vector<Object>& objs, const int& uv);
	void cameraScan();
	void upAndDownTrigger(int randomCurrent);
	void modbusTransfer();
	void checkModbus();
	void obtainPos();
	void readParams();
	void judgeStatus(int uvOut, int smokeOut, bool ppOut);
	
	Mat frame;
	Mat dst;
	WiringControl* controller;
	paddleDetector* detector;
	modbusClient* modbuser;
	Object lastTrace;

};



#endif //FIREDEMO_MAIN_SYSTEM_H
