#include "thread_system.h"
#include <iostream>
#include <fstream>
#include <string>

threadSystem::threadSystem():
saveIndex(0),
rangePosx(30),
rangePosy(20),
accumulateTrace(0),
scanOrTrace(0),
currentPos56(0),
currentPos2324(0),
triggerPos56(0),
triggerPos2324(0),
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
reseting(false)
{
    readParams();
    scanLR = leftDirect;
    scanUD = upDirect;
    triggerDirect = downDirect;
    triggerPos2324 = pitchLimit/2.0f;
    detector = new paddleDetector(modelFile, reModelFile, imgLight, ycOffset, xcOffset, gthreshold, rethreshold);
    controller = new WiringControl(leftDirect, upDirect, yawLimit, pitchLimit);
    controller->inOpen();
    unTrigger();
    
    if(rstOrNot) systemStatus = RESET;
    else systemStatus = STILL;
}

threadSystem::~threadSystem(){
    if(producer.joinable()) producer.join();
    if(inferencer.joinable()) inferencer.join();
    if(motioner.joinable()) motioner.join();
}

void threadSystem::readParams(){
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
			else if(paramName=="flipFlag") flipFlag = (bool)std::stoi(paramStr);
		    }
		}
	    }
	}
	rightDirect = !leftDirect;
	downDirect = !upDirect;
}

void threadSystem::start(){
    producer = std::thread([this]{produce();});
    inferencer = std::thread([this]{inference();});
    motioner = std::thread([this]{motion();});
}

void threadSystem::join(){
    if(producer.joinable()) producer.join();
    if(inferencer.joinable()) inferencer.join();
    if(motioner.joinable()) motioner.join();
}

void threadSystem::stop(){
    printf("stop\n");
    u_cv.notify_all();
    start_cv.notify_all();
    unTrigger();
    controller->stopMotor_56();
    controller->stopMotor_2324();
    controller->stopTempControl();
}

void threadSystem::produce(){
    Mat frameLocal;
    int uvLocal;
    float posLocal56, posLocal2324;
    VideoCapture v(0);
    if(!v.isOpened()) run_=false;
    while(run_){
	{
	    std::unique_lock<std::mutex> start_lock(start_mtx);
	    start_cv.wait(start_lock, [this]{return !reseting;});
	}
	
	v.read(frameLocal);
	if(frameLocal.empty()) break;
	cv::resize(frameLocal, frameLocal, cv::Size(320,320));
	if(flipFlag) flip(frameLocal, frameLocal, -1);
	
	
	uvLocal = controller->readUV();
	if(uvInit) uvLocal=HIGH;
	
	obtainPos(posLocal56, posLocal2324);
	
	//shared area
	{
	    std::lock_guard<std::mutex> p_lock(r_mtx);
	    frame = frameLocal;
	    currentPos56 = posLocal56;
	    currentPos2324 = posLocal2324;
	}
	{
	    std::lock_guard<std::mutex> p_lock(u_mtx);
	    uvOutput = uvLocal;
	}
	u_cv.notify_all();
	
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
    run_=false;
    stop();
}

void threadSystem::inference(){
    Mat dstLocal;
    int uvLocal;
    std::vector<Object> objsLocal;
    while(run_){
	{
	    std::unique_lock<std::mutex> start_lock(start_mtx);
	    start_cv.wait(start_lock, [this]{return !reseting;});
	}
	
	objsLocal = std::vector<Object>();
	{
	    std::unique_lock<std::mutex> iu_lock(u_mtx);
	    u_cv.wait(iu_lock, [this]{return uvOutput==HIGH || !run_;});
	    if(!run_) break;
	    uvLocal = uvOutput;
	}
	{
	    std::lock_guard<std::mutex> ir_lock(r_mtx);
	    dstLocal = frame;
	}
	
	
	objsLocal = detector->RunModel(dstLocal);
	
	
	{
	    std::lock_guard<std::mutex> i_lock(t_mtx);
	    dst = dstLocal;
	    objects = objsLocal;
	}
    }
}

void threadSystem::motion(){
    Mat dstLocal;
    int uvLocal;
    float posLocal56, posLocal2324;
    std::vector<Object> objsLocal;
    while(run_){
	{
	    std::lock_guard<std::mutex> mu_lock(u_mtx);
	    uvLocal = uvOutput;
	}
	{
	    std::lock_guard<std::mutex> mt_lock(t_mtx);
	    dstLocal = dst;
	    objsLocal = objects;
	}
	{
	    std::lock_guard<std::mutex> mr_lock(r_mtx);
	    posLocal56 = currentPos56;
	    posLocal2324 = currentPos2324;
	}
	
	controller->temprateControl();
	controller->limitIO3();
	controller->limitIO4();
	
	if(uvLocal == HIGH) printf("UV HIGH\n");
	else if(uvLocal == LOW) printf("UV LOW\n");
	else printf("UV nothing\n");
	printf("scanOrTrace:%d\n", scanOrTrace);
	printf("accumulate:%d\n", accumulateTrace);
	printf("TriggerCount:%d\n", triggerCount);
	printf("systemStatus:%d\n", systemStatus);
	if(systemStatus==RESET){
	    {
		std::lock_guard<std::mutex> start_lock(start_mtx);
		reseting = true;
	    }
	    controller->resetPos();
	    {
		std::lock_guard<std::mutex> start_lock(start_mtx);
		reseting = false;
	    }
	    start_cv.notify_all();
	    systemStatus = STILL;
	}
	else if(systemStatus==STILL){
	    accumulateTrace = 0;
	    
	    controller->stopMotor_56();
	    controller->stopMotor_2324();
	    
	    if(uvLocal==HIGH){
		scanOrTrace++;
		if(scanOrTrace>5000) systemStatus=SCAN;
	    }
	    else if(uvLocal==LOW){
		scanOrTrace--;
		if(scanOrTrace==4007) systemStatus = RESET;
		else if(scanOrTrace<0) scanOrTrace=0;
	    }
	}
	else if(systemStatus==SCAN){
	    bool ppOutput = findTarget(objsLocal, uvLocal);
	    if(accumulateTrace>=100){
		systemStatus = CONTROL;
		scanOrTrace=8000;
	    }
	    
	    if(uvLocal==HIGH){
		scanOrTrace++;
		if(scanOrTrace>8000) scanOrTrace=8000;
	    }
	    else if(uvLocal==LOW){
		systemStatus = STILL;
	    }
	}
	else if(systemStatus==CONTROL){
	    bool ppOutput = feedbackControlpp(objsLocal);
	    if(ppOutput) accumulateTrace+=2;
	    
	    if(accumulateTrace>=5000) systemStatus = TRIGGER;
	    else if(accumulateTrace==0) systemStatus = SCAN;
	}
	else if(systemStatus==TRIGGER){
	    upAndDownTrigger();
	    bool ppOutput = feedbackControlpp(objsLocal);
	    imshow("dstLocal", dstLocal);
	    if(accumulateTrace<20){
		unTrigger();
		systemStatus = CONTROL;
	    }
	}
	if(dstLocal.empty()) continue;
	
    }
    
}

bool threadSystem::findTarget(const std::vector<Object>& objs, const int& uv){
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
	    accumulateTrace-=2;
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
	    accumulateTrace++;
    }
    lastTrace = traceObject;
    return true;
}

bool threadSystem::feedbackControlpp(const std::vector<Object>& objs){
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
		accumulateTrace-=2;
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
		//~ else if(uv==HIGH){
			//~ singleScan();
			//~ return false;
		//~ }
		//else return false;
	}
	else if(accumulateTrace>=1000 && abs(traceObject.diff_cx)<rangePosx && abs(traceObject.diff_cy)<rangePosy){
	    accumulateTrace = 5000;
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
	if(traceObject.prob>=threshold) {
		accumulateTrace++;
		if(accumulateTrace>5000)accumulateTrace=5000;
		//if(lastTrigger&&accumulateTrace>80) return true;//ensure the constantly water in triggering
	}
	if(abs(traceObject.diff_cx)<rangePosx && abs(traceObject.diff_cy)<rangePosy){
		if(accumulateTrace>=200){
			return true;
		}
	}
	if(accumulateTrace<20){
		lastTrace = Object();
		lastTrigger = false;
	}
	return false;
}

void threadSystem::tinyModify56(int& diffCx){
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

void threadSystem::singleScan(){
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
		delay(100);
	}
	return;
}

void threadSystem::upAndDownTrigger(){
	if(!lastTrigger){
		triggerPos56=currentPos56;
		triggerPos2324=currentPos2324;
	}
	
	if(triggerCount<600){
		triggerCount++;
		return;
	}
	controller->stopMotor_56();
	controller->onTrigger();
	
	
	if(currentPos2324<=triggerPos2324-0.25) triggerDirect = upDirect;
	else if(currentPos2324>=triggerPos2324+0.25) triggerDirect = downDirect;
	
	controller->rotateMotor_2324(triggerDirect, true);
	lastTrigger = true;
}

void threadSystem::unTrigger(){
	triggerDirect = downDirect;
	triggerCount = 0;
	controller->unTrigger();
	if(accumulateTrace==0)lastTrigger = false;
}

void threadSystem::obtainPos(float& pos56, float& pos2324){
	pos56 = controller->getPosition56();
	pos2324 = controller->getPosition2324();
}

void threadSystem::printPos(cv::Mat& dst_, const float& pos56, const float& pos2324){
    char posText56[20];
    char posText2324[20];
    sprintf(posText56, "poslr:%f", pos56);
    sprintf(posText2324, "posud:%f", pos2324);
    putText(dst_,
		    std::string(posText56),
		    Point(5,20),
		    FONT_HERSHEY_COMPLEX_SMALL,
		    1.f,
		    cv::Scalar(255, 0, 0),
		    1,
		    cv::LINE_AA);
    putText(dst_,
		    std::string(posText2324),
		    Point(5,40),
		    FONT_HERSHEY_COMPLEX_SMALL,
		    1.f,
		    cv::Scalar(255, 0, 0),
		    1,
		    cv::LINE_AA);
}
