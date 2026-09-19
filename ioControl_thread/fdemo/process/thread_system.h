#ifndef THREAD_SYSTEM_H
#define THREAD_SYSTEM_H

#include <thread>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <vector>
#include "main.h"
#include "control.h"
#include "paddle_detection.h"

using namespace cv;

enum status{
    RESET=1,
    STILL,
    SCAN,
    CONTROL,
    TRIGGER
};

class threadSystem{
public:
    threadSystem();
    ~threadSystem();

    void start();
    void join();
    void stop();
    
private:
    void produce();
    void inference();
    void motion();

    std::thread producer, inferencer, motioner;
    std::mutex r_mtx, u_mtx, t_mtx, start_mtx;
    std::condition_variable u_cv, start_cv;
    bool reseting;

private:
    enum status systemStatus;
    int rangePosx, rangePosy;

    double currentPos56, currentPos2324;
    double triggerPos56, triggerPos2324;
    
    int accumulateTrace;
    int scanOrTrace;
    int scanUD, scanLR;
    bool lastScan;

    bool lastTrigger;
    int triggerCount;
    int triggerDirect;

    std::string modelFile, reModelFile;
    bool rstOrNot;
    double imgLight;
    int ycOffset, xcOffset;
    float yawLimit, pitchLimit;
    float gthreshold, rethreshold;
    int leftDirect, upDirect, rightDirect, downDirect;
    bool uvInit;
    bool flipFlag;

    Mat frame;
    Mat dst;
    int uvOutput;
    WiringControl* controller;
    paddleDetector* detector;
    Object lastTrace;
    
    int saveIndex;
    
    std::vector<Object> objects;
    
    void readParams();
    void obtainPos(float& pos56, float& pos2324);
    bool findTarget(const std::vector<Object>& objs, const int& uv);
    void singleScan();
    bool feedbackControlpp(const std::vector<Object>& objs);
    void tinyModify56(int& diffCx);
    void upAndDownTrigger();
    void unTrigger();
    void printPos(cv::Mat& dst_, const float& pos56, const float& pos2324);
};

#endif
