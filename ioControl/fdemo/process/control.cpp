#include "control.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

WiringControl::WiringControl(int leftDirect, int upDirect, int yawLi, int pitchLi):
direct_56(0),
direct_2324(0),
timePos56(0),
timePos2324(0),
startFlag56(0),
startFlag2324(0),
smoking(0),
leftFlag(leftDirect),
rightFlag(!leftDirect),
upFlag(upDirect),
downFlag(!upDirect),
yawLimit(yawLi),
pitchLimit(pitchLi),
windCount(0)
{
	readParams();
}

WiringControl::~WiringControl() {}

void WiringControl::readParams(){
	std::string paramfilename = "/home/l/Pack/param/gpio.txt";
	std::ifstream file(paramfilename);

	if(file.is_open()){
		std::string line, paramName, paramStr;
		while(std::getline(file, line)){
			for(int i=0;i<line.size();i++){
				if(line[i]==':'){
					paramName = line.substr(0, i);
					paramStr = line.substr(i+1, line.size()-i-1);
					if(paramName=="LAR_LIMIT") LAR_LIMIT = std::stoi(paramStr);
					else if(paramName=="UAD_LIMIT") UAD_LIMIT = std::stoi(paramStr);
					else if(paramName=="LAR_PUL") LAR_PUL = std::stoi(paramStr);
					else if(paramName=="LAR_DIR") LAR_DIR = std::stoi(paramStr);
					else if(paramName=="LAR_EN") LAR_EN = std::stoi(paramStr);
					else if(paramName=="UAD_PUL") UAD_PUL = std::stoi(paramStr);
					else if(paramName=="UAD_DIR") UAD_DIR = std::stoi(paramStr);
					else if(paramName=="UAD_EN") UAD_EN = std::stoi(paramStr);
					else if(paramName=="TRIGGER1") TRIGGER1 = std::stoi(paramStr);
					else if(paramName=="TRIGGER2") TRIGGER2 = std::stoi(paramStr);
					else if(paramName=="UV") UV = std::stoi(paramStr);
					else if(paramName=="SMOKE") SMOKE = std::stoi(paramStr);
					else if(paramName=="SMOKE_") SMOKE_ = std::stoi(paramStr);
					else if(paramName=="TEMPERATE") TEMPERATE = std::stoi(paramStr);
					else if(paramName=="TEMPERATE_") TEMPERATE_ = std::stoi(paramStr);
				}
			}
		}
	}
}

bool WiringControl::inOpen() {
	wiringPiSetupGpio();
	//ultraviolet light
	pinMode(UV, INPUT);
	pullUpDnControl(UV, PUD_UP);
	
	//限位
	pinMode(LAR_LIMIT, INPUT);
	pullUpDnControl(LAR_LIMIT, PUD_UP);
	pinMode(UAD_LIMIT, INPUT);
	pullUpDnControl(UAD_LIMIT, PUD_UP);
	
	//trigger
	pinMode(TRIGGER1, OUTPUT);
	pinMode(TRIGGER2, OUTPUT);
	
	//电机1,left and right
	pinMode(LAR_PUL, OUTPUT);//PUL
	pinMode(LAR_DIR, OUTPUT);//DIR
	pinMode(LAR_EN, OUTPUT);//EN
	digitalWrite(LAR_EN, HIGH);//disable
	softPwmCreate(LAR_PUL, 0, POWER);
	
	//电机2, up and down
	pinMode(UAD_PUL, OUTPUT);//PUL
	pinMode(UAD_DIR, OUTPUT);//DIR
	pinMode(UAD_EN, OUTPUT);//EN
	digitalWrite(UAD_EN, HIGH);//disable
	softPwmCreate(UAD_PUL, 0, POWER);
	
	//smoke
	pinMode(SMOKE, INPUT);
	pullUpDnControl(SMOKE, PUD_UP);
	pinMode(SMOKE_, OUTPUT);
	digitalWrite(SMOKE_, LOW);
	
	pinMode(TEMPERATE, OUTPUT);
	digitalWrite(TEMPERATE, LOW);
	pinMode(TEMPERATE_, OUTPUT);
	digitalWrite(TEMPERATE_, LOW);
	
	
	//printf("in\n");

	return true;
}

bool WiringControl::startMotor_56(){
	digitalWrite(LAR_EN, LOW);//enable
	return true;
}

bool WiringControl::startMotor_2324(){
	digitalWrite(UAD_EN, LOW);//enable
	return true;
}

//left and right halter, low is right, right is plus; high is left, left is minus
bool WiringControl::stopMotor_56(){
	if(startFlag56){
		auto tmp = std::chrono::high_resolution_clock::now();
		double duration = seconds_duration(tmp-tickPoint56).count();
		if(direct_56==leftFlag) timePos56-=duration;
		else timePos56+=duration; 
		startFlag56=0;
	}
	softPwmWrite(LAR_PUL, 0);
	digitalWrite(LAR_EN, HIGH);//disable
	direct_56=0;
	return true;
}

//up and down halter, low is down, high is up
bool WiringControl::stopMotor_2324(){
	if(startFlag2324){
		auto tmp = std::chrono::high_resolution_clock::now();
		double duration = seconds_duration(tmp-tickPoint2324).count();
		if(direct_2324==downFlag) timePos2324-=duration;
		else timePos2324+=duration;
		startFlag2324=0;
	}
	softPwmWrite(UAD_PUL, 0);
	digitalWrite(UAD_EN, HIGH);//disable22
	direct_2324=0;
	return true;
}

bool WiringControl::pauseMotor_56(){
	if(startFlag56){
		auto tmp = std::chrono::high_resolution_clock::now();
		double duration = seconds_duration(tmp-tickPoint56).count();
		if(direct_56==leftFlag) timePos56-=duration;
		else timePos56+=duration; 
		startFlag56=0;
	}
	softPwmWrite(LAR_PUL, 0);
	return true;
}

bool WiringControl::pauseMotor_2324(){
	if(startFlag2324){
		auto tmp = std::chrono::high_resolution_clock::now();
		double duration = seconds_duration(tmp-tickPoint2324).count();
		if(direct_2324==downFlag) timePos2324-=duration;
		else timePos2324+=duration;
		startFlag2324=0;
	}
	softPwmWrite(UAD_PUL, 0);
	return true;
}

//left and right controller
bool WiringControl::rotateMotor_56(int direct){
	direct_56 = direct;
	startMotor_56();
	//首先设置正反转,HIGH表示left,LOW表示right
	digitalWrite(LAR_DIR, direct);
	
	//change the value no difference
	//write until manual stop
	softPwmWrite(LAR_PUL, POWER-1);
	auto tmp = std::chrono::high_resolution_clock::now();
	if(startFlag56==0){
		startFlag56=1;
		tickPoint56=tmp;
	}
	else{
		double duration = seconds_duration(tmp-tickPoint56).count();
		tickPoint56=tmp;
		if(direct_56==leftFlag) timePos56-=duration;
		else timePos56+=duration;
	}
	return true;
}

//up and down controller
bool WiringControl::rotateMotor_2324(int direct){
	direct_2324 = direct;
	startMotor_2324();
	//首先设置正反转,HIGH表示up,LOW表示down
	digitalWrite(UAD_DIR, direct);
	
	//change the value no difference
	//write until manual stop
	softPwmWrite(UAD_PUL, POWER-1);
	auto tmp = std::chrono::high_resolution_clock::now();
	if(startFlag2324==0){
		startFlag2324=1;
		tickPoint2324=tmp;
	}
	else{
		double duration = seconds_duration(tmp-tickPoint2324).count();
		tickPoint2324=tmp;
		if(direct_2324==downFlag) timePos2324-=duration;
		else timePos2324+=duration;
	}
	return true;
}

//left and right limit
bool WiringControl::limitIO3(){
	int value = digitalRead(LAR_LIMIT);
	if(value == HIGH){
		printf("lr_limit:HIGH\n");
		return false;
	}
	else if(value == LOW){
		//reach limit, stop another motor and flip direct
		softPwmWrite(LAR_PUL, 0);
		softPwmWrite(UAD_PUL, 0);
		direct_56 = leftFlag;
		startMotor_56();
		digitalWrite(LAR_DIR, direct_56);
		softPwmWrite(LAR_PUL, POWER-1);
		printf("lr_limit:LOW\n");
		delay(200);
		softPwmWrite(LAR_PUL, 0);
		timePos56=0;
		return true;
	}
	else{
		printf("lr_limit:Nothing\n");
	}
	return true;
}

//up and down limit
bool WiringControl::limitIO4(){
	int value = digitalRead(UAD_LIMIT);
	if(value == HIGH){
		printf("ud_limit:HIGH\n");
		return false;
	}
	else if(value == LOW){
		//reach limit, stop another motor and flip direct
		softPwmWrite(LAR_PUL, 0);
		softPwmWrite(UAD_PUL, 0);
		direct_2324 = downFlag;
		startMotor_2324();
		digitalWrite(UAD_DIR, direct_2324);
		softPwmWrite(UAD_PUL, POWER-1);
		printf("ud_limit:LOW\n");
		delay(200);
		softPwmWrite(UAD_PUL, 0);
		timePos2324=0;
		return true;
	}
	else{
		printf("ud_limit:Nothing\n");
	}
	return true;
}

bool WiringControl::onTrigger(){
	digitalWrite(TRIGGER1, 0);
	digitalWrite(TRIGGER2, 0);
	return true;
}

bool WiringControl::unTrigger(){
	digitalWrite(TRIGGER1, 1);
	digitalWrite(TRIGGER2, 1);
	return true;
}

int WiringControl::readUV(){
	int value = digitalRead(UV);
	if(value == HIGH) printf("UV HIGH\n");
	else if(value == LOW) printf("UV LOW\n");
	else printf("UV nothing\n");
	return value;
}

double WiringControl::getPosition56(){
	//printf("timsPos:%lf\n", timePos);
	return timePos56;
}

double WiringControl::getPosition2324(){
	return timePos2324;
}

bool WiringControl::resetPos(){
	startMotor_56();
	startMotor_2324();
	direct_56 = rightFlag;
	digitalWrite(LAR_DIR, direct_56);
	softPwmWrite(LAR_PUL, POWER-1);
	while(!limitIO3()){}
	
	direct_2324 = upFlag;
	int IO4Count=0;
	while(IO4Count<3){
		direct_2324 = upFlag;
		digitalWrite(UAD_DIR, direct_2324);
		softPwmWrite(UAD_PUL, POWER-1);
		if(limitIO4()) IO4Count++;
	}
	auto s = std::chrono::high_resolution_clock::now();
	auto e = s;
	while(seconds_duration(e-s).count()<abs(pitchLimit)){
		direct_2324 = downFlag;
		digitalWrite(UAD_DIR, direct_2324);
		softPwmWrite(UAD_PUL, POWER-1);
		e = std::chrono::high_resolution_clock::now();
	}
	
	timePos56 = 0;
	timePos2324=pitchLimit;
	softPwmWrite(LAR_PUL, 0);
	digitalWrite(LAR_EN, HIGH);//disable
	softPwmWrite(UAD_PUL, 0);
	digitalWrite(UAD_EN, HIGH);//disable
	return true;
}

int WiringControl::readSmoke(){
	int smokeStatus = digitalRead(SMOKE);
	//printf("smoking:%d\n", smoking);
	if(smokeStatus==LOW){
		printf("SMOKE LOW\n");
		
	}
	else if(smokeStatus==HIGH){
		printf("SMOKE HIGH\n");
		smoking++;
	}
	else{
		printf("SMOKE nothing\n");
	}
	return smokeStatus;
}

// void WiringControl::rstZeroYaw(){
// }

// void WiringControl::rstZeroPitch(){
// }

//以限位和时间共同限制
int WiringControl::reachLimit56(){
	if(limitIO3()) return 1;
	else if(timePos56<yawLimit) return 2;
	else return 0;
}

int WiringControl::reachLimit2324(){
	if(limitIO4()) return 1;
	else if(timePos2324<pitchLimit) return 2;
	else return 0;
}

bool WiringControl::temprateControl(){
	ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
	string temp_str;
	float tempTmp = 0;
	if (temp_file.is_open()) {
		std::getline(temp_file, temp_str);
		temp_file.close();
		try {
			tempTmp = std::stof(temp_str) / 1000.0;
		} catch (const std::invalid_argument& e) {
			std::cerr << "Error converting temperature string to float: " << e.what() << std::endl;
		}
		printf("temp:%f\n", tempTmp);
		if(tempTmp>50){
			digitalWrite(TEMPERATE, HIGH);
			digitalWrite(TEMPERATE_, HIGH);
			windCount=5000;
		}
		else{
			if(windCount>0){
				digitalWrite(TEMPERATE, HIGH);
				digitalWrite(TEMPERATE_, HIGH);
				windCount--;
			}
			digitalWrite(TEMPERATE, LOW);
			digitalWrite(TEMPERATE_, LOW);
		}
	} else {
		std::cerr << "Unable to open temperature file" << std::endl;
	}
	return true;
}


