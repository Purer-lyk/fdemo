#include "main_system.h"
#include <iostream>
#include <fstream>
#include <string>
#include "sample.h"
#include "main.h"

mainSystem::mainSystem():
save_cnt(0),
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
scanOrTrace(0),
thermalDisable(0),
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
	controller->unAlarm();
}

mainSystem::~mainSystem() {
	stop();
}

void mainSystem::stop(){
    printf("stop\n");
    unTrigger();
    controller->unAlarm();
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
					else if(paramName=="device") device = paramStr;
				}
			}
		}
	}
	rightDirect = !leftDirect;
	downDirect = !upDirect;
}

void mainSystem::run() {
	//VideoCapture v("./test/3.mp4");
	VideoCapture v("/dev/v4l/by-id/usb-SYD_USB_Camera_200901010001-video-index0");
	if(!v.isOpened()) return;
	std::vector<Object> objs;
	std::vector<int> diffVec;
	int unblockDelay=0;
	int saveIndex=0;
	//modbusTcpStatus = modbuser->modbusConnect();
	//serverStatus = server->modbusConnect();

	startThermal();
	if(rstOrNot) controller->resetPos();
	while(v.isOpened() && run_){
		//~ controller->temprateControl();
		loseTarget();
		//modifyPitch();
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
		//cv::cvtColor(dst, dst, COLOR_BGR2GRAY);
		//cv::rectangle(dst, cv::Rect(145-rangePosx, 180-rangePosy, rangePosx*2, rangePosy*2), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		
		// Tracing and UV then Trigger
		int uvOutput = controller->readUV();
		//int smokeOutput = controller->readSmoke();
		bool ppOutput = false, thermalOutput = false;
		if(uvInit) uvOutput=HIGH;
		//printf("UV:%d\n",uvOutput);

		obtainPos();
		readThermal();
		
		//thermalOutput = readThermal();
		//old state change
		if(uvOutput==HIGH){
			controller->onAlarm();
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
		}
		
		//new state transfer
		/*if(systemStatus==RESET){
			loseTarget();
			systemStatus = STILL;
		}
		if(systemStatus==STILL){
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
		if(systemStatus==SCAN){
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
		if(systemStatus==CONTROL){
			objs = detector->RunModel(dst);
			
			bool ppOutput = feedbackControlpp(objs, uvOutput);
			if(ppOutput) accumulateTrace+=2;
			
			bool thermalOutput = readThermal();
			if(thermalCount==-25){
				accumulateTrace=0;
			}
			
			if(accumulateTrace>=400 && thermalOutput){
				thermalCount=0;
				systemStatus = TRIGGER;
			}
			else if(accumulateTrace==0){
				thermalCount = 0;
				lastTrace = Object();
				systemStatus = SCAN;
			}
		}
		if(systemStatus==TRIGGER){
			upAndDownTrigger();
			objs = detector->RunModel(dst);
			bool ppOutput = feedbackControlpp(objs, uvOutput);
			if(accumulateTrace<5){
				unTrigger();
				systemStatus = CONTROL;
			}
		}*/
		//bool thermalOutput = readThermal();

		//printf("thermal:%d\n", thermalOutput);
		//printf("scanOrTrace:%d\n", scanOrTrace);
		//printf("systemStatus:%d\n", systemStatus);
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
			isRUNNING = false;
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
		if(flipFlag) controller->rotateMotor_56(rightDirect, false);
		else controller->rotateMotor_56(leftDirect, false);
		diffCx++;
		delay(90);
	}
	else if(diffCx>3){
		if(flipFlag) controller->rotateMotor_56(leftDirect, false);
		else controller->rotateMotor_56(rightDirect, false);
		diffCx--;
		delay(90);
	}
}

void mainSystem::modifyPitch(){
	if(accumulateTrace>0) return;
	controller->stopMotor_56();
	int t = abs(currentPos2324-pitchLimit)*100+200;
	if(currentPos2324-pitchLimit>1.5){
		controller->rotateMotor_2324(downDirect);
		delay(t);
		controller->stopMotor_2324();
	}
	else if(currentPos2324-pitchLimit<-1.5){
		controller->rotateMotor_2324(upDirect);
		delay(t);
		controller->stopMotor_2324();
	}
}

bool mainSystem::feedbackControlpp(const std::vector<Object>& objs, const int& uv){
	float threshold = gthreshold;
	Object traceObject;
	
	for(auto obj:objs){
		if(obj.class_id==0 && obj.prob>=threshold){
			threshold = obj.prob;
			traceObject = obj;
		}
	}
	
	if(thermalDisable || thermalCount<0){
		thermalCount++;
		if(thermalDisable) traceObject = Object();
		if(thermalCount==0) thermalDisable = 0;
	}
	
	printf("accumulate:%d\n", accumulateTrace);
	printf("thermalCount:%d\n", thermalCount);
	//constantly trace
	if(traceObject.prob<gthreshold){
		accumulateTrace-=5;
		if(accumulateTrace<0)accumulateTrace=0;
		
		if(lastTrace.prob>=gthreshold){
			int ldx = lastTrace.diff_cx;
			int ldy = lastTrace.diff_cy;
			traceObject.prob = lastTrace.prob;
			traceObject.rec = lastTrace.rec;
			traceObject.roi = lastTrace.roi;
			
			if(abs(ldx)<rangePosx) traceObject.diff_cx = ldx;
			else traceObject.diff_cx = ldx>0?ldx-rangePosx/3:ldx+rangePosx/3;
			
			if(abs(ldy)<rangePosy) traceObject.diff_cy = ldy;
			else traceObject.diff_cy = ldy>0?ldy-rangePosy/3:ldy+rangePosy/3;
		}
		else if(uv==HIGH){
		    singleScan();
		    return false;
	    }
	    //else return false;
	}
	else{
		/*double cRatio = traceObject.cLength/lastTrace.cLength;
		
		if(cRatio>0.8 && cRatio<1.2) cMisMatch++;*/
		accumulateTrace+=4;
		if(accumulateTrace>400)accumulateTrace=400;
		if(accumulateTrace>20 && abs(traceObject.diff_cx)<rangePosx && abs(traceObject.diff_cy)<rangePosy){
			accumulateTrace=400;
		}
	}
	
	if(!lastTrigger && !thermalDisable){
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
	/*if(traceObject.prob>=gthreshold){
		accumulateTrace+=4;
		if(accumulateTrace>400)accumulateTrace=400;
		//if(lastTrigger&&accumulateTrace>80) return true;//ensure the constantly water in triggering
	}*/
	if(abs(traceObject.diff_cx)<rangePosx && abs(traceObject.diff_cy)<rangePosy){
		bool thermalOutput;
		if(lastTrigger) thermalOutput = true;
		else{	
			thermalOutput = readThermal();
			if(thermalCount<-70){
				accumulateTrace = 0;
				/*char imgName[20];
				sprintf(imgName, "%d.jpg", save_cnt);
				save_cnt++;
				cv::imwrite(imgName, traceObject.roi);*/
				thermalDisable = 1;
				//detector->thermalDisbles.push_back(traceObject.roi);
			}
		}
		
		if(accumulateTrace>20 && thermalOutput){
			return true;
		}
	}
	if(accumulateTrace<10){
		if(accumulateTrace==0)lastTrace = Object();
		lastTrigger = false;
	}
	return false;
}

void mainSystem::loseTarget(){
	printf("scanOrTrace:%d\n", scanOrTrace);
	scanOrTrace = --scanOrTrace<0?0:scanOrTrace;
	if(lastScan && scanOrTrace==307) {
		controller->unAlarm();
		resetStatus();
		controller->resetPos();
		scanLR = leftDirect;
		scanUD = downDirect;
		lastScan = false;
		scanState = scanOffset;
		scanOrTrace = 0;
	}
	else if(scanOrTrace>800) scanOrTrace=800;
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
	//modifyPitch();
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
	printf("triggerCount:%d\n", triggerCount);
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
	
	
	if(currentPos2324<=triggerPos2324-0.6) triggerDirect = upDirect;
	else if(currentPos2324>=triggerPos2324+0.05) triggerDirect = downDirect;
	
	controller->rotateMotor_2324(triggerDirect, true);
	/*delay(500);
	controller->rotateMotor_2324(!randomCurrent, false);
	delay(500);
	controller->stopMotor_2324();*/
	lastTrigger = true;
}

//~ bool mainSystem::modbusTransfer(){
	//~ uint8_t M_BIT1 = 0x00; 
	//~ // if(currentPos>distinct) M_BIT1=0x01;
	//~ // else if(currentPos<-distinct) M_BIT1=0x03;
	//~ // else M_BIT1=0x02;
	//~ for(int i=0;i<distinct.size()-1;i++){
		//~ if(currentPos56>distinct[i] && currentPos56<distinct[i+1]) M_BIT1 = i+1;
	//~ }

	//~ uint8_t M_BIT2 = 0x00;
	//~ if(fireStatus==1) M_BIT2=0x01;
	//~ else if(fireStatus==2) M_BIT2=0x02;
	//~ else if(fireStatus==3) M_BIT2=0x03;
	
	//~ uint8_t M_BIT3 = device;
	
	//~ uint8_t M_BITS[] = {M_BIT1,M_BIT2, M_BIT3};
	//~ return modbuser->writeBits(M_BITS, 0x18);
//~ }

//~ bool mainSystem::tcpTransfer(){
	//~ uint8_t M_BIT1 = 0x00; 
	//~ // if(currentPos>distinct) M_BIT1=0x01;
	//~ // else if(currentPos<-distinct) M_BIT1=0x03;
	//~ // else M_BIT1=0x02;
	//~ for(int i=0;i<distinct.size()-1;i++){
		//~ if(currentPos56>distinct[i] && currentPos56<distinct[i+1]) M_BIT1 = i+1;
	//~ }

	//~ uint8_t M_BIT2 = 0x00;
	//~ if(fireStatus==1) M_BIT2=0x01;
	//~ else if(fireStatus==2) M_BIT2=0x02;
	//~ else if(fireStatus==3) M_BIT2=0x03;
	
	//~ uint8_t M_BIT3 = device;
	
	//~ uint8_t M_BITS[] = {M_BIT1,M_BIT2, M_BIT3};
	//~ return tcper->writeBits(M_BITS, 0x03);
//~ }

//~ bool mainSystem::modbusReply(){
	//~ uint8_t M_BIT1 = 0x00; 
	//~ // if(currentPos>distinct) M_BIT1=0x01;
	//~ // else if(currentPos<-distinct) M_BIT1=0x03;
	//~ // else M_BIT1=0x02;
	//~ for(int i=0;i<distinct.size()-1;i++){
		//~ if(currentPos56>distinct[i] && currentPos56<distinct[i+1]) M_BIT1 = i+1;
	//~ }

	//~ uint8_t M_BITS[] = {M_BIT1};
	//~ return server->writeBits(M_BITS, 0x01);
//~ }

void mainSystem::obtainPos(){
	currentPos56 = controller->getPosition56();
	currentPos2324 = controller->getPosition2324();
	detector->updateYc(abs(triggerPos2324)*0.1+ycOffset);
}

void mainSystem::printPos(){
	printf("poslr:%lf\n",currentPos56);
	printf("posud:%lf\n",currentPos2324);
	/*char posText56[20];
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
			cv::LINE_AA);*/
}

void mainSystem::startThermal(){
	return;
}

void mainSystem::stopThermal(){
	return;
}

bool mainSystem::readThermal(){
	LineRectTempInfo_t* temp_info = temp_command(&stream_frame_info, 0, 0, 255, 191);
	if(temp_info==NULL){
		return false;
	}
	/*cv::Mat yuyv = cv::Mat(global_height, global_width, CV_8UC2, global_frame);
	cv::Mat bgr;
	cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);*/
			
	/*for (int y = 1; y < bgr.rows-1; y++) {
		cv::Vec3b* rowPtr = bgr.ptr<cv::Vec3b>(y);
		cv::Vec3b* rowPtrM = bgr.ptr<cv::Vec3b>(y+1);
		cv::Vec3b* rowPtrP = bgr.ptr<cv::Vec3b>(y-1);
		for (int x = 1; x < bgr.cols-1; x++) {
			if(rowPtr[x][2]>rowPtr[x][1] && rowPtr[x][2]>rowPtr[x][0]){
				if(rowPtr[x+1][2]>rowPtr[x+1][1] && rowPtr[x+1][2]>rowPtr[x+1][0] && 
				rowPtr[x-1][2]>rowPtr[x-1][1] && rowPtr[x-1][2]>rowPtr[x-1][0] &&
				rowPtrM[x][2]>rowPtrM[x][1] && rowPtrM[x][2]>rowPtrM[x][0] &&
				rowPtrP[x][2]>rowPtrP[x][1] && rowPtrP[x][2]>rowPtrP[x][0]){
					printf("pixel_x:%d, pixel_y:%d\n",x,y);
				}
			}
		}
	}*/
	//cv::cvtColor(bgr, bgr, COLOR_BGR2GRAY);
	//cv::imshow("thermal", bgr);
	
	//<200 x area, >20 y area, left top y：5-14，x：2-100
	
	/*for (int y = 80; y < bgr.rows; y++) {
		cv::Vec3b* rowPtr = bgr.ptr<cv::Vec3b>(y);
		for (int x = 30; x < bgr.cols; x++) {
			if(rowPtr[x][2]-rowPtr[x][1]>50 && rowPtr[x][2]-rowPtr[x][0]>50){
				correctLoc = true;
				//printf("pixel_x:%d, pixel_y:%d\n",x,y);
			}
		}
	}*/
	
	/*cv::Vec3b* rowPtr = bgr.ptr<cv::Vec3b>(6);
	if(rowPtr[3][2]-rowPtr[3][1]>20 && rowPtr[3][2]-rowPtr[3][0]>20 && correctLoc) thermalCount++;
	else thermalCount-=7;*/
	
	bool correctLoc = true;
	if(temp_info->max_min_temp_info.max_temp>550) thermalCount++;
	else thermalCount-=7;
	
	if(thermalCount>10){
		thermalCount=10;
		return true;
	}
	else{
		if(thermalCount<-80) thermalCount=-80;
		return false;
	}

}

