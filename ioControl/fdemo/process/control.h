#ifndef FIREDEMO_CONTROL_H
#define FIREDEMO_CONTROL_H

#include <cstdio>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <softPwm.h>
#include <chrono>

using seconds_duration = std::chrono::duration<double>;

class WiringControl{
public:
	WiringControl(int leftDirect, int upDirect);
	~WiringControl();

	bool inOpen();
	bool limitIO3();
	bool limitIO4();
	bool rotateMotor_56(int direct);
	bool rotateMotor_2324(int direct);
	bool stopMotor_56();
	bool stopMotor_2324();
	bool startMotor_56();
	bool startMotor_2324();
	bool onTrigger();
	bool unTrigger();
	int readUV();
	double getPosition();
	bool resetPos(int yawInit, int pitchInit);
	int readSmoke();
	int smoking;

private:
	uint8_t LAR_LIMIT;
	uint8_t UAD_LIMIT;
	uint8_t LAR_PUL;
	uint8_t LAR_DIR;
	uint8_t LAR_EN;
	uint8_t UAD_PUL;
	uint8_t UAD_DIR;
	uint8_t UAD_EN;
	uint8_t TRIGGER1;
	uint8_t TRIGGER2;
	uint8_t UV;
	uint8_t SMOKE;
	uint8_t SMOKE_;

	int fd;
	int direct_56;
	int direct_2324;
	double timePos;
	int startFlag;
	int leftFlag, rightFlag;
	int upFlag, downFlag;
	std::chrono::high_resolution_clock::time_point tickPoint;
	
	void readParams();
};


#endif //FIREDEMO_CONTROL_H
