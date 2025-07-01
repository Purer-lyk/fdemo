#ifndef FIREDEMO_DETECTION_H
#define FIREDEMO_DETECTION_H

#include "opencv2/opencv.hpp"

using namespace cv;

class Detector{
public:
	Detector();
	~Detector();
	std::vector<int> mainDetect(const Mat &src, Mat &dst);

private:
	int t, satOffset;
	int in_width, in_height;
	Mat ycrcbSrc;
	Mat hsvSrc;
	Mat demoTemplate;
	


};


#endif //FIREDEMO_DETECTION_H
