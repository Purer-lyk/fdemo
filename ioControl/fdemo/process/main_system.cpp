#include "main_system.h"
#include <random>
#include <iostream>
#include <fstream>
#include <string>


mainSystem::mainSystem():
srcW(320),
srcH(320),
rangePosx(30),
rangePosy(20),
accumulateTrace(0),
modbusReconnect(0),
tcpReconnect(0),
serverReconnect(0),
currentPos56(0),
currentPos2324(0),
modbusTcpStatus(false),
tcpStatus(false),
serverStatus(false),
fireStatus(0),
triggerCount(0),
rstOrNot(false),
imgLight(60.0),
ycOffset(20),
yawLimit(-5),
pitchLimit(-5),
gthreshold(0.6),
leftDirect(1),
upDirect(1),
yawInitPos(5000),
pitchInitPos(12000),
device(0),
scanOrTrace(0),
modbusInterval(0),
tcpInterval(0),
serverInterval(0)
{
	readParams();
	scanLR = leftDirect;
	scanUD = downDirect;
	detector = new paddleDetector(modelFile, imgLight, ycOffset, gthreshold);
	controller = new WiringControl(leftDirect, upDirect, yawLimit, pitchLimit);
	modbuser = new modbusClient(modbusIP, modbusPORT);
	tcper = new tcpClient(modbusIP, modbusPORT);
	server = new modbusServer(modbusIP, modbusPORT);

	auto now = std::chrono::system_clock::now();
	auto hours = std::chrono::duration_cast<std::chrono::hours>(now.time_since_epoch());

	controller->inOpen();
	unTrigger();
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
					paramStr = line.substr(i+1, line.size()-i-2);
					if(paramName=="modelFile") modelFile = paramStr;
					else if(paramName=="rstOrNot") rstOrNot = (bool)std::stoi(paramStr);
					else if(paramName=="imgLight") imgLight = std::stod(paramStr);
					else if(paramName=="ycOffset") ycOffset = std::stoi(paramStr);
					else if(paramName=="rangePosx") rangePosx = std::stoi(paramStr);
					else if(paramName=="rangePosy") rangePosy = std::stoi(paramStr);
					else if(paramName=="yawLimit") yawLimit = std::stoi(paramStr);
					else if(paramName=="pitchLimit") pitchLimit = std::stoi(paramStr);
					else if(paramName=="threshold") gthreshold = std::stof(paramStr);
					else if(paramName=="leftDirect") leftDirect = std::stoi(paramStr);
					else if(paramName=="upDirect") upDirect = std::stoi(paramStr);
					else if(paramName=="distinct"){
						// distinct = std::stof(paramStr);
						int s=0,e=0;
						for(int j=0;j<paramStr.size();j++){
							if(paramStr[j]=='|' || paramStr[j]=='\n'){
								e=j;
								float point = std::stof(paramStr.substr(s,e-s));
								distinct.push_back(point);
								printf("pt:%f\n",point);
								s=j+1;
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
	serverStatus = server->modbusConnect();

	if(rstOrNot) controller->resetPos();
	while(v.isOpened()){
		//checkModbus();
		// checkTcp();
		checkServer();
		loseTarget();
		controller->temprateControl();
		controller->limitIO3();
		controller->limitIO4();
		
		v.read(frame);
		if(flipFlag) flip(frame, frame, -1);
		if(frame.empty()) {
			printf("camera error!");
			break;
		}
		srcW = frame.cols;
		srcH = frame.rows;
		frame.copyTo(dst);
		
		// Tracing and UV then Trigger
		int uvOutput = controller->readUV();
		int smokeOutput = controller->readSmoke();
		bool ppOutput = false;
		//uvOutput=HIGH;

		if(uvOutput==HIGH){
			// nerual network
			objs = detector->RunModel(frame, dst);

			ppOutput = feedbackControlpp(objs, uvOutput);
			if(ppOutput){
				upAndDownTrigger(dist(eng));
			}
			else{
				unTrigger();
			}
			scanOrTrace+=12;
		}
		else{
			controller->stopMotor_2324();
			controller->stopMotor_56();
		}

		obtainPos();
		judgeStatus(uvOutput, smokeOutput, ppOutput);

		modbusInterval = ++modbusInterval%21;
		if(modbusTcpStatus && modbusInterval==20) modbusTcpStatus = modbusTransfer();
		tcpInterval = ++tcpInterval%21;
		if(tcpStatus && tcpInterval==20) tcpStatus = tcpTransfer();
		serverInterval = ++serverInterval%21;
		if(serverStatus && serverInterval==20) serverStatus = modbusReply();

		//imshow("origin", frame);
		imshow("result",dst);
		
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

void mainSystem::checkServer(){
	if(serverStatus){
		serverReconnect = 0;
	}
	else serverReconnect = ++serverReconnect%21;
	if(serverReconnect==20) serverStatus = server->modbusConnect();
}

bool mainSystem::feedbackControlpp(const std::vector<Object>& objs, const int& uv){
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
			else traceObject.diff_cx = ldx>0?ldx-rangePosx/2:ldx+rangePosx/2;
			
			if(abs(ldy)<rangePosy) traceObject.diff_cy = ldy;
			else traceObject.diff_cy = ldy>0?ldy-rangePosy/2:ldy+rangePosy/2;
		}
		else if(uv==HIGH){
			cameraScan();
			return false;
		}
		else return false;
	}

	//HIGH is left, LOW is right, 注意diff是跟踪点减中心点
	if(traceObject.diff_cx<-rangePosx){
		if(flipFlag) controller->rotateMotor_56(rightDirect);
		else controller->rotateMotor_56(leftDirect);
	}
	else if(traceObject.diff_cx>rangePosx){
		if(flipFlag) controller->rotateMotor_56(leftDirect);
		else controller->rotateMotor_56(rightDirect);
	}
	else if(abs(traceObject.diff_cx)<rangePosx) controller->stopMotor_56();
	
	//HIGH is up, LOW is down
	if(traceObject.diff_cy<-rangePosy){
		if(flipFlag) controller->rotateMotor_2324(downDirect);
		else controller->rotateMotor_2324(upDirect);
	}
	else if(traceObject.diff_cy>rangePosy){
		if(flipFlag) controller->rotateMotor_2324(upDirect);
		else controller->rotateMotor_2324(downDirect);
	}
	else if(abs(traceObject.diff_cy)<rangePosy) controller->stopMotor_2324();

	lastTrace = traceObject;
	if(traceObject.prob>=threshold) {
		accumulateTrace+=4;
		if(accumulateTrace>100)accumulateTrace=100;
		if(accumulateTrace>50) return true;
	}
	if(abs(traceObject.diff_cx)<rangePosx && abs(traceObject.diff_cy)<rangePosy){
		if(accumulateTrace>40)return true;
		else if(accumulateTrace<5)lastTrace = Object();
	}
	return false;
}

void mainSystem::loseTarget(){
	printf("scanOrTrace:%d\n", scanOrTrace);
	scanOrTrace = --scanOrTrace<0?0:scanOrTrace;
	if(scanOrTrace==13) {
		resetStatus();
		controller->resetPos();
		scanLR = leftDirect;
		scanUD = downDirect;
	}
	else if(scanOrTrace>300) scanOrTrace=300;
}

void mainSystem::resetStatus(){
	//accumulateTrace = 0;
	unTrigger();
	lastTrace = Object();
	controller->stopMotor_2324();
	controller->stopMotor_56();
}

void mainSystem::unTrigger(){
	controller->unTrigger();
	lastTrigger = false;
}

void mainSystem::cameraScan(){
	//printf("lr:%d,ud:%d\n",scanLR, scanUD);
	//在限制内就简单的rotate
	//不在限制就反转方向再rotate
	int lrCondition = controller->reachLimit56();
	int udCondition = controller->reachLimit2324();
	if(!lrCondition && !udCondition){
		printf("in\n");
		controller->rotateMotor_56(scanLR);
		controller->rotateMotor_2324(scanUD);
	}
	else{
		if(lrCondition==1) scanLR = leftDirect;
		else if(lrCondition==2) scanLR = rightDirect;
		
		if(udCondition==1) scanUD = downDirect;
		else if(udCondition==2) scanUD = upDirect;
		printf("out\n");
		controller->rotateMotor_56(scanLR);
		controller->rotateMotor_2324(scanUD);
		delay(200);
	}
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
	lastTrigger = true;
}

bool mainSystem::modbusTransfer(){
	uint8_t M_BIT1 = 0x00; 
	// if(currentPos>distinct) M_BIT1=0x01;
	// else if(currentPos<-distinct) M_BIT1=0x03;
	// else M_BIT1=0x02;
	for(int i=0;i<distinct.size()-1;i++){
		if(currentPos56>distinct[i] && currentPos56<distinct[i+1]) M_BIT1 = i+1;
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
	uint8_t M_BIT1 = 0x00; 
	// if(currentPos>distinct) M_BIT1=0x01;
	// else if(currentPos<-distinct) M_BIT1=0x03;
	// else M_BIT1=0x02;
	for(int i=0;i<distinct.size()-1;i++){
		if(currentPos56>distinct[i] && currentPos56<distinct[i+1]) M_BIT1 = i+1;
	}

	uint8_t M_BIT2 = 0x00;
	if(fireStatus==1) M_BIT2=0x01;
	else if(fireStatus==2) M_BIT2=0x02;
	else if(fireStatus==3) M_BIT2=0x03;
	
	uint8_t M_BIT3 = device;
	
	uint8_t M_BITS[] = {M_BIT1,M_BIT2, M_BIT3};
	return tcper->writeBits(M_BITS, 0x03);
}

bool mainSystem::modbusReply(){
	uint8_t M_BIT1 = 0x00; 
	// if(currentPos>distinct) M_BIT1=0x01;
	// else if(currentPos<-distinct) M_BIT1=0x03;
	// else M_BIT1=0x02;
	for(int i=0;i<distinct.size()-1;i++){
		if(currentPos56>distinct[i] && currentPos56<distinct[i+1]) M_BIT1 = i+1;
	}

	uint8_t M_BITS[] = {M_BIT1};
	return server->writeBits(M_BITS, 0x01);
}

void mainSystem::obtainPos(){
	currentPos56 = controller->getPosition56();
	currentPos2324 = controller->getPosition2324();
	//printf("pos:%lf\n", currentPos);
	char posText56[20];
	char posText2324[20];
	sprintf(posText56, "poslr:%lf", currentPos56);
	sprintf(posText2324, "posud:%lf", currentPos2324);
	putText(dst,
			std::string(posText56),
			Point(5,20),
			FONT_HERSHEY_COMPLEX_SMALL,
			1.f,
			cv::Scalar(255, 0, 0),
			1,
			cv::LINE_AA);
	putText(dst,
			std::string(posText2324),
			Point(5,40),
			FONT_HERSHEY_COMPLEX_SMALL,
			1.f,
			cv::Scalar(255, 0, 0),
			1,
			cv::LINE_AA);
}

