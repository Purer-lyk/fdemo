#include "control.h"
#include <iostream>
#include <fstream>
#include <string>

WiringControl::WiringControl(int leftDirect, int upDirect):
direct_56(0),
direct_2324(0),
timePos(0),
startFlag(0),
smoking(0),
leftFlag(leftDirect),
rightFlag(!leftDirect),
upFlag(upDirect),
downFlag(!upDirect)
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
				}
			}
		}
	}
}

bool WiringControl::inOpen() {
	wiringPiSetupGpio();
	//ultraviolet light
	pinMode(UV, INPUT);
	pullUpDnControl(UV, PUD_DOWN);
	
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
	softPwmCreate(LAR_PUL, 0, 9);
	
	//电机2, up and down
	pinMode(UAD_PUL, OUTPUT);//PUL
	pinMode(UAD_DIR, OUTPUT);//DIR
	pinMode(UAD_EN, OUTPUT);//EN
	digitalWrite(UAD_EN, HIGH);//disable
	softPwmCreate(UAD_PUL, 0, 9);
	
	//smoke
	pinMode(SMOKE, INPUT);
	pullUpDnControl(SMOKE, PUD_UP);
	pinMode(SMOKE_, OUTPUT);
	digitalWrite(SMOKE_, LOW);
	
	//printf("in\n");

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
		softPwmWrite(UAD_PUL, 0);
		softPwmWrite(LAR_PUL, 0);
		direct_56 = !direct_56;
		digitalWrite(LAR_EN, LOW);
		digitalWrite(LAR_DIR, direct_56);
		softPwmWrite(LAR_PUL, 8);
		printf("lr_limit:LOW\n");
		delay(1000);
		softPwmWrite(UAD_PUL, 0);
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
		softPwmWrite(UAD_PUL, 0);
		softPwmWrite(LAR_PUL, 0);
		direct_2324 = !direct_2324;
		digitalWrite(UAD_EN, LOW);
		digitalWrite(UAD_DIR, direct_2324);
		softPwmWrite(UAD_PUL, 8);
		printf("ud_limit:LOW\n");
		delay(1000);
		softPwmWrite(UAD_PUL, 0);
		return true;
	}
	else{
		printf("ud_limit:Nothing\n");
	}
	return true;
}

//left and right controller
bool WiringControl::rotateMotor_56(int direct){
	direct_56 = direct;
	startMotor_56();
	auto tmp = std::chrono::high_resolution_clock::now();
	if(startFlag==0){
		startFlag=1;
		tickPoint=tmp;
	}
	else{
		double duration = seconds_duration(tmp-tickPoint).count();
		tickPoint=tmp;
		if(direct_56) timePos-=duration;
		else timePos+=duration;
	}
	//首先设置正反转,HIGH表示left,LOW表示right
	digitalWrite(LAR_DIR, direct);
	
	//change the value no difference
	//write until manual stop
	softPwmWrite(LAR_PUL, 8);

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
	softPwmWrite(UAD_PUL, 8);
	
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
	if(startFlag){
		auto tmp = std::chrono::high_resolution_clock::now();
		double duration = seconds_duration(tmp-tickPoint).count();
		tickPoint = tmp;
		if(direct_56) timePos-=duration;
		else timePos+=duration; 
		startFlag=0;
	}
	softPwmWrite(LAR_PUL, 0);
	digitalWrite(LAR_EN, HIGH);//disable
	return true;
}

//up and down halter, low is down, high is up
bool WiringControl::stopMotor_2324(){
	softPwmWrite(UAD_PUL, 0);
	digitalWrite(UAD_EN, HIGH);//disable
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

double WiringControl::getPosition(){
	//printf("timsPos:%lf\n", timePos);
	return timePos;
}

bool WiringControl::resetPos(int yawInit=5000, int pitchInit=12000){
	startMotor_56();
	startMotor_2324();
	direct_56 = rightFlag;
	digitalWrite(LAR_DIR, direct_56);
	softPwmWrite(LAR_PUL, 8);
	while(!limitIO3()){}
	direct_56 = leftFlag;
	digitalWrite(LAR_DIR, direct_56);
	softPwmWrite(LAR_PUL, 8);
	delay(yawInit);
	softPwmWrite(LAR_PUL, 0);
	
	direct_2324 = upFlag;
	int IO4Count=0;
	while(IO4Count<3){
		direct_2324 = upFlag;
		digitalWrite(UAD_DIR, direct_2324);
		softPwmWrite(UAD_PUL, 8);
		if(limitIO4()) IO4Count++;
	}
	direct_2324 = downFlag;
	digitalWrite(UAD_DIR, direct_2324);
	softPwmWrite(UAD_PUL, 8);
	delay(pitchInit);
	softPwmWrite(UAD_PUL, 0);
	
	timePos = 0;
	digitalWrite(LAR_EN, HIGH);//disable
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


