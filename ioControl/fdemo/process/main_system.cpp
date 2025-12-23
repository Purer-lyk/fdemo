#include "main_system.h"
#include <iostream>
#include <fstream>
#include <string>

mainSystem::mainSystem():
srcW(320),
srcH(320),
rangePosx(30),
rangePosy(20),
accumulateTrace(0),
noTraceOffset(0),
modbusReconnect(0),
tcpReconnect(0),
serverReconnect(0),
currentPos56(0),
currentPos2324(0),
triggerPos56(0),
triggerPos2324(0),
modbusTcpStatus(false),
tcpStatus(false),
serverStatus(false),
fireStatus(0),
triggerCount(0),
rstOrNot(false),
uvInit(false),
imgLight(60.0),
ycOffset(20),
xcOffset(2),
yawLimit(-5),
pitchLimit(-5),
gthreshold(0.6),
leftDirect(1),
upDirect(1),
lastScan(false),
scanOffset(1.5),
device(0),
scanOrTrace(0),
modbusInterval(0),
tcpInterval(0),
serverInterval(0),
systemStatus(STILL)
{
	readParams();
	scanLR = leftDirect;
	scanUD = upDirect;
	triggerDirect = downDirect;
	scanState = scanOffset;
	triggerPos2324 = pitchLimit/2.0f;
	detector = new paddleDetector(modelFile, reModelFile, imgLight, ycOffset, xcOffset, gthreshold, rethreshold);
	controller = new WiringControl(leftDirect, upDirect, yawLimit, pitchLimit);
	modbuser = new modbusClient(modbusIP, modbusPORT);
	tcper = new tcpClient(modbusIP, modbusPORT);
	server = new modbusServer(modbusIP, modbusPORT);

	auto now = std::chrono::system_clock::now();
	auto hours = std::chrono::duration_cast<std::chrono::hours>(now.time_since_epoch());

	controller->inOpen();
	unTrigger();
}

mainSystem::~mainSystem() {
	stop();
}

void mainSystem::stop(){
    printf("stop\n");
    unTrigger();
    controller->stopMotor_56();
    controller->stopMotor_2324();
    controller->stopTempControl();
    detector->~paddleDetector();
}

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
					else if(paramName=="reModelFile") reModelFile = paramStr;
					else if(paramName=="rstOrNot") rstOrNot = (bool)std::stoi(paramStr);
					else if(paramName=="uvInit") uvInit = (bool)std::stoi(paramStr);
					else if(paramName=="imgLight") imgLight = std::stod(paramStr);
					else if(paramName=="ycOffset") ycOffset = std::stoi(paramStr);
					else if(paramName=="xcOffset") xcOffset = std::stoi(paramStr);
					else if(paramName=="rangePosx") rangePosx = std::stoi(paramStr);
					else if(paramName=="rangePosy") rangePosy = std::stoi(paramStr);
					else if(paramName=="yawLimit") yawLimit = std::stof(paramStr);
					else if(paramName=="pitchLimit") pitchLimit = std::stof(paramStr);
					else if(paramName=="threshold") gthreshold = std::stof(paramStr); //0.69
					else if(paramName=="rethreshold") rethreshold = std::stof(paramStr); //0.75
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
	//VideoCapture v("./test/3.mp4");
	VideoCapture v(0);
	if(!v.isOpened()) return;
	std::vector<Object> objs;
	std::vector<int> diffVec;
	int unblockDelay=0;
	int saveIndex=0;
	modbusTcpStatus = modbuser->modbusConnect();
	serverStatus = server->modbusConnect();

	if(rstOrNot) systemStatus = RESET;
	while(v.isOpened() && run_){
		controller->temprateControl();
		controller->limitIO3();
		controller->limitIO4();
		
		v.read(frame);
		//frame = imread("./test/WEB03143.jpg");
		if(frame.empty()) {
			continue;
		}
		cv::resize(frame, frame, cv::Size(320,320));
		if(flipFlag) flip(frame, frame, -1);
		
		srcW = frame.cols;
		srcH = frame.rows;
		frame.copyTo(dst);
		
		// Tracing and UV then Trigger
		int uvOutput = controller->readUV();
		int smokeOutput = controller->readSmoke();
		bool ppOutput = false;
		if(uvInit) uvOutput=HIGH;
		printf("UV:%d\n",uvOutput);

		obtainPos();
		
		//old state change
		/*if(uvOutput==HIGH){
			if(scanOrTrace>400){
				lastScan = true;
				// nerual network
				objs = detector->RunModel(dst);
				//printf("objs size:%d\n",objs.size());

				ppOutput = feedbackControlpp(objs, uvOutput);
				if(ppOutput||lastTrigger){
					imshow("result",dst);
					upAndDownTrigger();
				}
				else{
					unTrigger();
					//if(abs(currentPos56-triggerPos56)>1.5||abs(currentPos2324-triggerPos2324)>1.5)accumulateTrace=0;
				}
			}
			scanOrTrace+=6;
		}
		else{
			accumulateTrace=0;
			unTrigger();
			controller->stopMotor_2324();
			controller->stopMotor_56();
		}*/
		
		//new state transfer
		if(systemStatus==RESET){
			loseTarget();
			systemStatus = STILL;
		}
		else if(systemStatus==STILL){
			accumulateTrace = 0;
			
			controller->stopMotor_56();
			controller->stopMotor_2324();
			
			if(uvOutput==HIGH){
				scanOrTrace+=3;
				if(scanOrTrace>400) systemStatus=SCAN;
			}
			else if(uvOutput==LOW){
				scanOrTrace--;
				if(scanOrTrace==307) systemStatus = RESET;
				else if(scanOrTrace<0) scanOrTrace=0;
			}
		}
		else if(systemStatus==SCAN){
			objs = detector->RunModel(dst);
			bool ppOutput = findTarget(objs, uvOutput);
			if(accumulateTrace>=4){ 
				systemStatus = CONTROL;
				scanOrTrace=800;
			}
			
			if(uvOutput==HIGH){
				scanOrTrace+=3;
				if(scanOrTrace>800) scanOrTrace=800;
			}
			else if(uvOutput==LOW){
				systemStatus = STILL;
			}
		}
		else if(systemStatus==CONTROL){
			objs = detector->RunModel(dst);
			bool ppOutput = feedbackControlpp(objs);
			if(ppOutput) accumulateTrace+=2;
			
			if(accumulateTrace>=400) systemStatus = TRIGGER;
			else if(accumulateTrace==0) systemStatus = SCAN;
		}
		else if(systemStatus==TRIGGER){
			upAndDownTrigger();
			objs = detector->RunModel(dst);
			bool ppOutput = feedbackControlpp(objs);
			if(accumulateTrace<5){
				unTrigger();
				systemStatus = CONTROL;
			}
		}

		printf("scanOrTrace:%d\n", scanOrTrace);
		printf("systemStatus:%d\n", systemStatus);
		printPos();

		imshow("result",dst);
		//imshow("origin", frame);
		
		// keyboard
		int key = waitKey(1);
		//printf("key:%d\n", key);
		if(key==27){
			unTrigger();
			controller->stopMotor_56();
			controller->stopMotor_2324();
			break;
		}
		else if(key==115){
			char saveFileName[50];
			sprintf(saveFileName, "%04d.jpg", saveIndex); 
			imwrite(saveFileName, dst);
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

bool mainSystem::findTarget(const std::vector<Object>& objs, const int& uv){
    float threshold = gthreshold;
    Object traceObject;
    for(auto obj:objs){
	    if(obj.class_id==0 && obj.prob>=threshold){
		    threshold = obj.prob;
		    traceObject = obj;
	    }
    }
    //~ printf("accumulate:%d\n", accumulateTrace);
    //constantly trace
    if(traceObject.prob<threshold){
	    accumulateTrace-=3;
	    if(accumulateTrace<0)accumulateTrace=0;
	    
	    if(lastTrace.prob>=threshold){
		    int ldx = lastTrace.diff_cx;
		    int ldy = lastTrace.diff_cy;
		    traceObject.prob = lastTrace.prob;
		    traceObject.rec = lastTrace.rec;
		    
		    if(abs(ldx)<rangePosx) traceObject.diff_cx = ldx;
		    else traceObject.diff_cx = ldx>0?ldx-rangePosx/3:ldx+rangePosx/3;
		    
		    if(abs(ldy)<rangePosy) traceObject.diff_cy = ldy;
		    else traceObject.diff_cy = ldy>0?ldy-rangePosy/3:ldy+rangePosy/3;
	    }
	    else if(uv==HIGH){
		    singleScan();
		    return false;
	    }
	    else return false;
    }
    else if(traceObject.prob>=threshold){
	    accumulateTrace+=2;
    }
    lastTrace = traceObject;
    return true;
}

void mainSystem::tinyModify56(int diffCx){
	if(diffCx<-3){
		if(flipFlag) controller->rotateMotor_56(rightDirect);
		else controller->rotateMotor_56(leftDirect);
		diffCx++;
		delay(100);
	}
	else if(diffCx>3){
		if(flipFlag) controller->rotateMotor_56(leftDirect);
		else controller->rotateMotor_56(rightDirect);
		diffCx--;
		delay(100);
	}
}

bool mainSystem::feedbackControlpp(const std::vector<Object>& objs){
	float threshold = gthreshold;
	//printf("threshold:%f\n", threshold);
	Object traceObject;
	for(auto obj:objs){
		if(obj.class_id==0 && obj.prob>=threshold){
			threshold = obj.prob;
			traceObject = obj;
		}
	}
	printf("accumulate:%d\n", accumulateTrace);
	//constantly trace
	if(traceObject.prob<gthreshold){
		accumulateTrace-=5;
		if(accumulateTrace<0)accumulateTrace=0;
		
		if(lastTrace.prob>=gthreshold){
			int ldx = lastTrace.diff_cx;
			int ldy = lastTrace.diff_cy;
			traceObject.prob = lastTrace.prob;
			traceObject.rec = lastTrace.rec;
			
			if(abs(ldx)<rangePosx) traceObject.diff_cx = ldx;
			else traceObject.diff_cx = ldx>0?ldx-rangePosx/3:ldx+rangePosx/3;
			
			if(abs(ldy)<rangePosy) traceObject.diff_cy = ldy;
			else traceObject.diff_cy = ldy>0?ldy-rangePosy/3:ldy+rangePosy/3;
		}
	}
	else if(accumulateTrace>25 && abs(traceObject.diff_cx)<rangePosx && abs(traceObject.diff_cy)<rangePosy){
		accumulateTrace=400;
	}

	if(!lastTrigger){
		//HIGH is left, LOW is right, 注意diff是跟踪点减中心点
		if(traceObject.diff_cx<-rangePosx){
			if(flipFlag) controller->rotateMotor_56(rightDirect);
			else controller->rotateMotor_56(leftDirect);
		}
		else if(traceObject.diff_cx>rangePosx){
			if(flipFlag) controller->rotateMotor_56(leftDirect);
			else controller->rotateMotor_56(rightDirect);
		}
		else if(traceObject.diff_cx!=0){
			tinyModify56(traceObject.diff_cx);
			controller->stopMotor_56();
		}
		
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
	}
	
	lastTrace = traceObject;
	if(traceObject.prob>=gthreshold) {
		accumulateTrace+=4;
		if(accumulateTrace>400)accumulateTrace=400;
		//if(lastTrigger&&accumulateTrace>80) return true;//ensure the constantly water in triggering
	}
	if(abs(traceObject.diff_cx)<rangePosx && abs(traceObject.diff_cy)<rangePosy){
		if(accumulateTrace>25){
			return true;
		}
	}
	if(accumulateTrace<5){
		lastTrace = Object();
		lastTrigger = false;
	}
	return false;
}

void mainSystem::loseTarget(){
	//~ printf("scanOrTrace:%d\n", scanOrTrace);
	//~ scanOrTrace = --scanOrTrace<0?0:scanOrTrace;
	//~ if(lastScan && scanOrTrace==307) {
	resetStatus();
	controller->resetPos();
	scanLR = leftDirect;
	scanUD = downDirect;
	lastScan = false;
	scanState = scanOffset;
	scanOrTrace = 0;
	//~ }
	//~ else if(scanOrTrace>800) scanOrTrace=800;
}

void mainSystem::resetStatus(){
	//accumulateTrace = 0;
	unTrigger();
	lastTrace = Object();
	controller->stopMotor_2324();
	controller->stopMotor_56();
}

void mainSystem::unTrigger(){
	triggerDirect = downDirect;
	triggerCount = 0;
	controller->unTrigger();
	if(accumulateTrace==0)lastTrigger = false;
}

void mainSystem::cameraScan(){
	//printf("lr:%d,ud:%d\n",scanLR, scanUD);
	//在限制内就简单的rotate
	//不在限制就反转方向再rotate
	int lrCondition = controller->reachLimit56();
	int udCondition = controller->reachLimit2324();
	if(!lrCondition && !udCondition){
			controller->rotateMotor_2324(scanUD);
			controller->rotateMotor_56(scanLR);
	}
	else{
		if(lrCondition==1) scanLR = leftDirect;
		else if(lrCondition==2) scanLR = rightDirect;
		if(udCondition==1) scanUD = downDirect;
		else if(udCondition==2) scanUD = upDirect;
		
		controller->rotateMotor_56(scanLR);
		controller->rotateMotor_2324(scanUD);
		delay(200);
	}
	return;
}

void mainSystem::stdScan(){
	//printf("lr:%d,ud:%d\n",scanLR, scanUD);
	int lrCondition = controller->reachLimit56();
	int udCondition = controller->reachLimit2324();
	if(lrCondition||udCondition){
		if(lrCondition==1){
			scanLR = leftDirect;
			scanState = scanOffset;
		}
		else if(lrCondition==2){
			scanLR = rightDirect;
			scanState = yawLimit-scanOffset;
		}
		
		if(udCondition==1){
			scanUD = downDirect;
			scanState=currentPos56;
		}
		else if(udCondition==2){
			scanUD = upDirect;
			scanState=currentPos56;
		}
		if(lrCondition)controller->rotateMotor_56(scanLR);
		if(udCondition)controller->rotateMotor_2324(scanUD);
		delay(200);
		if(lrCondition)controller->stopMotor_56();
		if(udCondition)controller->stopMotor_2324();
	}
	
	if(abs(currentPos56-scanState)>=scanOffset){
		controller->stopMotor_56();
		controller->rotateMotor_2324(scanUD);
		
	}
	if(abs(currentPos56-scanState)<scanOffset){
		controller->stopMotor_2324();
		controller->rotateMotor_56(scanLR);
	}
	printf("scanState:%d\n",scanState);
	return;
}

void mainSystem::singleScan(){
	/*if(abs(currentPos2324-pitchLimit)>0.8 && accumulateTrace==0){
		noTraceOffset++;
		if()controller->resetPitch();
	}*/
	int lrCondition = controller->reachLimit56();
	controller->stopMotor_2324();
	if(!lrCondition){
			controller->rotateMotor_56(scanLR);
	}
	else{
		if(lrCondition==1) scanLR = leftDirect;
		else if(lrCondition==2) scanLR = rightDirect;
		
		controller->rotateMotor_56(scanLR);
		delay(200);
	}
	return;
}

void mainSystem::judgeStatus(int uvOut, int smokeOut, bool ppOut){
	if(smokeOut==HIGH) fireStatus=1;
	if(uvOut) fireStatus=2;
	if(ppOut && uvOut) fireStatus=3;
}

void mainSystem::upAndDownTrigger(){
	//triggerCount = ++triggerCount%41;
	//if(triggerCount<30) return;
	if(!lastTrigger){
		triggerPos56=currentPos56;
		triggerPos2324=currentPos2324;
	}
	
	if(triggerCount<50){
		triggerCount++;
		return;
	}
	controller->stopMotor_56();
	controller->onTrigger();
	
	
	if(currentPos2324<=triggerPos2324-0.25) triggerDirect = upDirect;
	else if(currentPos2324>=triggerPos2324+0.25) triggerDirect = downDirect;
	
	controller->rotateMotor_2324(triggerDirect, true);
	/*delay(500);
	controller->rotateMotor_2324(!randomCurrent, false);
	delay(500);
	controller->stopMotor_2324();*/
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
	detector->updateYc(abs(triggerPos2324)*2+6);
}

void mainSystem::printPos(){
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

