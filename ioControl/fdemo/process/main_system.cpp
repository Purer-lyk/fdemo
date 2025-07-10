#include "main_system.h"
#include <random>
#include <iostream>
#include <fstream>
#include <string>

mainSystem::mainSystem(std::string model_file):
srcW(320),
srcH(320),
rangePosx(30),
rangePosy(20),
accumulateTrace(0),
modbusReconnect(0),
tcpReconnect(0),
currentPos(0),
modbusTcpStatus(false),
tcpStatus(false),
fireStatus(0),
triggerCount(0),
rstOrNot(false),
imgLight(60.0),
ycOffset(20),
yawLimit(10),
gthreshold(0.6),
leftDirect(1),
upDirect(1),
yawInitPos(5000),
pitchInitPos(12000),
device(0),
scanOrTrace(0)
{
	readParams();
	direct_56 = scanYaw/4;
	direct_2324 = 0;
	detector = new paddleDetector(model_file, imgLight, ycOffset, gthreshold);
	controller = new WiringControl(leftDirect, upDirect);
	modbuser = new modbusClient(modbusIP, modbusPORT);

	auto now = std::chrono::system_clock::now();
	auto hours = std::chrono::duration_cast<std::chrono::hours>(now.time_since_epoch());
	rstTick = hours.count();

	controller->inOpen();
	controller->unTrigger();
}

mainSystem::~mainSystem() {}

void mainSystem::readParams(){
	std::string paramfilename = "/home/l/Pack/param/params.txt";
	std::ifstream file(paramfilename);

	if(file.is_open()){
		std::string line, paramName, paramStr;
		while(std::getline(file, line)){
			for(int i=0;i<line.size();i++){
				if(line[i]==':'){
					paramName = line.substr(0, i);
					paramStr = line.substr(i+1, line.size()-i-1);
					if(paramName=="rstOrNot") rstOrNot = (bool)std::stoi(paramStr);
					else if(paramName=="imgLight") imgLight = std::stod(paramStr);
					else if(paramName=="ycOffset") ycOffset = std::stoi(paramStr);
					else if(paramName=="rangePosx") rangePosx = std::stoi(paramStr);
					else if(paramName=="rangePosy") rangePosy = std::stoi(paramStr);
					else if(paramName=="scanYaw") scanYaw = std::stoi(paramStr);
					else if(paramName=="scanPitch") scanPitch = std::stoi(paramStr);
					else if(paramName=="yawLimit") yawLimit = std::stoi(paramStr);
					else if(paramName=="threshold") gthreshold = std::stof(paramStr);
					else if(paramName=="leftDirect") leftDirect = std::stoi(paramStr);
					else if(paramName=="upDirect") upDirect = std::stoi(paramStr);
					else if(paramName=="yawInitPos") yawInitPos = std::stoi(paramStr);
					else if(paramName=="pitchInitPos") pitchInitPos = std::stoi(paramStr);
					else if(paramName=="distinct"){
						// distinct = std::stof(paramStr);
						for(int j=0;i<paramstr.size();j++){
							if(paramStr[j]==',' || paramStr[j]=='\n'){
								float point = paramstr.substr(0,j);
								distinct.push_back(point);
							}
						}
					}
					else if(paramName=="modbusIP") modbusIP = paramStr;
					else if(paramName=="modbusPORT") modbusPORT = std::stoi(paramStr);
					else if(paramName=="flipFlag") flipFlag = (bool)std::stoi(paramStr);
					else if(paramName=="device") device = std::stoi(paramStr);
				}
			}
		}
	}
	rightDirect = !leftDirect;
	downDirect = !upDirect;
}

void mainSystem::run() {
	//VideoCapture v("fireDemo.mp4");
	VideoCapture v(0);
	if(!v.isOpened()) return;
	std::vector<Object> objs;
	std::vector<int> diffVec;
	int unblockDelay=0;
	int saveIndex=0;
	std::random_device rd;
	std::default_random_engine eng(rd());
	std::uniform_int_distribution<int> dist(0,1);
	modbusTcpStatus = modbuser->modbusConnect();
	
	if(rstOrNot) controller->resetPos(yawInitPos, pitchInitPos);
	while(v.isOpened()){
		checkModbus();
		// checkTcp();
		loseTarget();
		
		v.read(frame);
		if(flipFlag) flip(frame, frame, -1);
		if(frame.empty())break;
		srcW = frame.cols;
		srcH = frame.rows;
		
		// nerual network
		objs = detector->RunModel(frame, dst);
		
		// Tracing and UV then Trigger
		int uvOutput = controller->readUV();
		int smokeOutput = controller->readSmoke();
		//uvOutput=HIGH;
		bool ppOutput = feedbackControlpp(objs, uvOutput);
		obtainPos();
		judgeStatus(uvOutput, smokeOutput, ppOutput);
		if(ppOutput && uvOutput==HIGH) {
			upAndDownTrigger(dist(eng));
			//unblockDelay=unblockDelay>0?unblockDelay:1;
		}
		/*if(unblockDelay>0) unblockDelay++;
		if(unblockDelay==50*60){
			controller->unTrigger();
			unblockDelay=0;
			lastTrace = Object();
		}*/
		else if(ppOutput && uvOutput==LOW){
			lastTrace = Object();
		}
		else if(uvOutput==LOW){
			controller->unTrigger();
		}
		// else if(uvOutput==LOW && smokeOutput==HIGH){
		// 	fireStatus = 1;
		// 	controller->onTrigger();
		// }
		// else if(uvOutput==LOW && smokeOutput==LOW){
		// 	fireStatus = 0;
		// 	controller->unTrigger();
		// }
		if(modbusTcpStatus) modbusTcpStatus = modbusTransfer();
		//imshow("origin", frame);
		imshow("result",dst);
		
		// ontime reset
		auto now = std::chrono::system_clock::now();
		auto hours = std::chrono::duration_cast<std::chrono::hours>(now.time_since_epoch());
		if(hours.count()-rstTick==1) controller->resetPos(yawInitPos, pitchInitPos);
		rstTick = hours.count();
		
		// keyboard
		int key = waitKey(1);
		//printf("key:%d\n", key);
		if(key==27){
			modbuser->modbusDisConnect();
			tcper->disconnectServer();
			break;
		}
		else if(key==115){
			char saveFileName[50];
			sprintf(saveFileName, "%04d.jpg", saveIndex); 
			imwrite(saveFileName, frame);
			saveIndex++;
		}
		else if(key==82){
			controller->rotateMotor_2324(upDirect);
			delay(200);
			controller->stopMotor_2324();
		}
		else if(key==84){
			controller->rotateMotor_2324(downDirect);
			delay(200);
			controller->stopMotor_2324();
		}
		else if(key==81){
			controller->rotateMotor_56(leftDirect);
			delay(200);
			controller->stopMotor_56();
		}
		else if(key==83){
			controller->rotateMotor_56(rightDirect);
			delay(200);
			controller->stopMotor_56();
		}
		else if(key==114){
			controller->rstZeroYaw();
		}
	}
}

void mainSystem::checkModbus(){
	if(modbusTcpStatus){
		const uint8_t M_BITS = 0x01; 
		//modbuser->writeBits(M_BITS);
		modbusReconnect = 0;
	}
	else modbusReconnect = ++modbusReconnect%21;
	if(modbusReconnect==20) modbusTcpStatus = modbuser->modbusConnect();
}

void mainSystem::checkTcp(){
	if(tcpStatus){
		tcpReconnect = 0;
	}
	else tcpReconnect = ++tcpReconnect%21;
	if(tcpReconnect==20) tcpStatus = tcper->connectServer();
}

void mainSystem::loseTarget(){
	if(scanOrTrace==-1) return;
	scanOrTrace = --scanOrTrace<0?0:scanOrTrace;
	if(scanOrTrace==0) {
		controller->rstZeroYaw();
		direct_56 = scanYaw/4;;
		direct_2324 = 0;
	}
}

bool mainSystem::feedbackControlpp(const std::vector<Object>& objs, const int& uv){
	controller->limitIO3();
	controller->limitIO4();
	float threshold = gthreshold;
	//printf("threshold:%f\n", threshold);
	Object traceObject;
	for(auto obj:objs){
		if(obj.class_id==0 && obj.prob>=threshold){
			threshold = obj.prob;
			traceObject = obj;
		}
	}
	if(traceObject.prob<threshold) {
		accumulateTrace-=5;
		if(accumulateTrace<0)accumulateTrace=0;
	}
	printf("accumulate:%d\n", accumulateTrace);
	//constantly trace
	if(traceObject.prob<threshold){
		if(lastTrace.prob>=threshold){
			int ldx = lastTrace.diff_cx;
			int ldy = lastTrace.diff_cy;
			traceObject.prob = lastTrace.prob;
			traceObject.rec = lastTrace.rec;
			if(abs(ldx)<rangePosx) traceObject.diff_cx = ldx;
			else traceObject.diff_cx = ldx>0?ldx-rangePosx:ldx+rangePosx;
			if(abs(ldy)<rangePosy) traceObject.diff_cy = ldy;
			else traceObject.diff_cy = ldy>0?ldy-rangePosy:ldy+rangePosy;
		}
		else if(uv==HIGH){
			cameraScan();
			return false;
		}
	}
	scanOrTrace+=20;

	//HIGH is left, LOW is right
	if(traceObject.diff_cx<-rangePosx) controller->rotateMotor_56(leftDirect);
	else if(traceObject.diff_cx>rangePosx) controller->rotateMotor_56(rightDirect);
	else if(abs(traceObject.diff_cx)<rangePosx) controller->stopMotor_56();
	
	//HIGH is up, LOW is down
	if(traceObject.diff_cy<-rangePosy) controller->rotateMotor_2324(upDirect);
	else if(traceObject.diff_cy>rangePosy) controller->rotateMotor_2324(downDirect);
	else if(abs(traceObject.diff_cy)<rangePosy) controller->stopMotor_2324();

	lastTrace = traceObject;
	if(traceObject.prob>=threshold) {
		accumulateTrace+=4;
		if(accumulateTrace>100)accumulateTrace=100;
	}
	if(abs(traceObject.diff_cx)<rangePosx && abs(traceObject.diff_cy)<rangePosy){
		if(accumulateTrace>30)return true;
		else if(accumulateTrace<10)lastTrace = Object();
	}
	return false;
}


void mainSystem::cameraScan(){
	scanOrTrace = -1;//todo:可能使定位更丝滑

	direct_56 = ++direct_56%(scanYaw);
	direct_2324 = ++direct_2324%(scanPitch);
	// if(controller->inLimit()==-1) controller->rotateMotor_56(rightDirect);
	// else if(controller->inLimit()==1) controller->rotateMotor_56(leftDirect);
	// else controller->rotateMotor_56(rightDirect);
	if(direct_56>scanYaw/2) controller->rotateMotor_56(rightDirect);
	else controller->rotateMotor_56(leftDirect);
	
	if(direct_2324>scanPitch/2) controller->rotateMotor_2324(upDirect);
	else controller->rotateMotor_2324(downDirect);


	
	// if(controller->rstZeroYawOr){
	// 	direct_56 = scanYaw/4;;
	// 	direct_2324 = 0;
	// }
	return;
}

void mainSystem::judgeStatus(int uvOut, int smokeOut, bool ppOut){
	if(smokeOut==HIGH) fireStatus=1;
	if(uvOut) fireStatus=2;
	if(ppOut && uvOut) fireStatus=3;
}

void mainSystem::upAndDownTrigger(int randomCurrent){
	triggerCount = ++triggerCount%11;
	if(triggerCount<10) return; 
	
	controller->onTrigger();
	controller->rotateMotor_2324(randomCurrent);
	delay(500);
	controller->rotateMotor_2324(!randomCurrent);
	delay(500);
}

bool mainSystem::modbusTransfer(){
	uint8_t M_BIT1 = 0x00; 
	// if(currentPos>distinct) M_BIT1=0x01;
	// else if(currentPos<-distinct) M_BIT1=0x03;
	// else M_BIT1=0x02;
	for(int i=0;i<distinct.size()-1;i++){
		if(currentPos>distinct[i] && currentPos<distinct[i+1]) M_BIT1 = i+1;
	}

	uint8_t M_BIT2 = 0x00;
	if(fireStatus==1) M_BIT2=0x01;
	else if(fireStatus==2) M_BIT2=0x02;
	else if(fireStatus==3) M_BIT2=0x03;
	
	uint8_t M_BIT3 = device;
	
	uint8_t M_BITS[] = {M_BIT1,M_BIT2, M_BIT3};
	return modbuser->writeBits(M_BITS, 0x18);
}
bool mainSystem::tcpTransfer(){

}

void mainSystem::obtainPos(){
	currentPos = controller->getPosition();
	//printf("pos:%lf\n", currentPos);
	char posText[20];
	sprintf(posText, "pos:%lf", currentPos);
	putText(dst,
			std::string(posText),
			Point(5,20),
			FONT_HERSHEY_COMPLEX_SMALL,
			1.f,
			cv::Scalar(255, 0, 0),
			1,
			cv::LINE_AA);
}

