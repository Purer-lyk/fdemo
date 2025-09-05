#ifndef FIREDEMO_CONTROL_H
#define FIREDEMO_CONTROL_H

#include <cstdio>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <softPwm.h>
#include <chrono>

#define POWER 12

using seconds_duration = std::chrono::duration<double>;

class WiringControl{
public:
	WiringControl(int leftDirect, int upDirect, int yawLi, int pitchLi);
	~WiringControl();

	bool inOpen();
	bool limitIO3();
	bool limitIO4();
	bool rotateMotor_56(int direct);
	bool rotateMotor_2324(int direct, bool calTime=true);
	bool stopMotor_56();
	bool stopMotor_2324();
	bool pauseMotor_56();
	bool pauseMotor_2324();
	bool startMotor_56();
	bool startMotor_2324();
	bool onTrigger();
	bool unTrigger();
	bool temprateControl();
	int readUV();
	double getPosition56();
	double getPosition2324();
	bool resetPos();
	int readSmoke();
	// void rstZeroYaw();
	// void rstZeroPitch();
	int reachLimit56();
	int reachLimit2324();
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
	uint8_t TEMPERATE;
	uint8_t TEMPERATE_;

	int fd;
	int direct_56;
	int direct_2324;
	
	int leftFlag, rightFlag;
	int upFlag, downFlag;
	int yawLimit;
	int pitchLimit;

	int startFlag56;
	double timePos56;
	std::chrono::high_resolution_clock::time_point tickPoint56;

	int startFlag2324;
	double timePos2324;
	std::chrono::high_resolution_clock::time_point tickPoint2324;
	
	int windCount;
	
	void readParams();
};


#endif //FIREDEMO_CONTROL_H
