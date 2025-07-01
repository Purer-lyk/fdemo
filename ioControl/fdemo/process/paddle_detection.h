#ifndef PADDLE_DETECTION_H
#define PADDLE_DETECTION_H

#include <iostream>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "paddle_api.h"

using namespace paddle::lite_api;

struct Object {
  int diff_cx;
  int diff_cy;
  int class_id;
  float prob;
  cv::Rect rec;
  Object():rec(cv::Rect()),diff_cx(0),diff_cy(0),class_id(0),prob(0)
  {}
  
};

//const char* categories[] = {"fire"};

class paddleDetector{
public:
	paddleDetector(std::string model_file, double imgLight, int yco, float thres);
	~paddleDetector();
	std::vector<Object> RunModel(cv::Mat &in, cv::Mat &img);
	
	float threshold;
private:
	MobileConfig config;
	std::shared_ptr<PaddlePredictor> predictor;
	std::vector<std::string> categories;
	int in_width;
	int in_height;
	float wScale, hScale;
	int traceCx, traceCy;
	int ycOffset;
	int convertLight;
	
	void pre_initial(cv::Mat& img);
	void pre_process(const cv::Mat& img, int width, int height, float* data);
	std::vector<Object> detect_object(const float* data, int count, float thresh, cv::Mat& image);
	void neon_mean_scale(const float* din, float* dout, int size, const std::vector<float> mean, const std::vector<float> scale);
	
};




#endif
