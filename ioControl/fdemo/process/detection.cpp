#include "detection.h"
#include <vector>

void onMouse(int event, int x, int y, int flags, void *param){
	Mat src = *(Mat*)param;
	const auto hsvPix = src.at<Vec3b>(x, y);
	switch(event){
		case cv::EVENT_LBUTTONDOWN:printf("H:%d\tS:%d\tV:%d\n",hsvPix[0],hsvPix[1],hsvPix[2]);break;
		default:break;
	}
}

Detector::Detector():t(70),satOffset(50),in_width(0),in_height(0)
{
	demoTemplate = imread("/home/l/ioControl/fdemo/build/fireTemp.png");
	resize(demoTemplate, demoTemplate, Size(25,25));
}

Detector::~Detector(){}


std::vector<int> Detector::mainDetect(const Mat &src, Mat &dst){
	src.copyTo(dst);
	in_width = src.cols;
	in_height = src.rows;
	cvtColor(src,ycrcbSrc,COLOR_BGR2YCrCb);
	
	//YCbCr模型两个公式识别火焰:if (Y>Ymean && Cb<Cbmean && Cr>Crmean)&&(Cr-Cb>t)
	std::vector<Mat> mv;
	Scalar picMeans;
	double yMean=0, CrMean=0, CbMean=0;
	picMeans = mean(ycrcbSrc);
	yMean = picMeans.val[0];
	CrMean = picMeans.val[1];
	CbMean = picMeans.val[2];
	
	Mat _src = Mat::zeros(src.rows, src.cols, CV_8UC1);
	for(int i=0;i<ycrcbSrc.rows;i++){
		for(int j=0;j<ycrcbSrc.cols;j++){
			Vec3b pix = ycrcbSrc.at<Vec3b>(i, j);
			const auto& orPix = src.at<Vec3b>(i, j);
			int Y = static_cast<int>(pix[0]);
			int Cr = static_cast<int>(pix[1]);
			int Cb = static_cast<int>(pix[2]);
			int B = static_cast<int>(orPix[0]);
			int G = static_cast<int>(orPix[1]);
			int R = static_cast<int>(orPix[2]);
			
			if(Y>yMean && Cr>CrMean && Y>100 && Cr>150 && Cb<CbMean && Cr-Cb>t && R>G && G>B) {
				_src.at<uchar>(i, j) = 255;
			}
		}
	}
//	dst = _src;
	
	//HSV Experiment
	//H通道的取值范围为0—180；S通道的取值范围为0—255；V通道的取值范围为0—255
	cvtColor(src, hsvSrc, COLOR_BGR2HSV);
	Scalar hsvMeans;
	Mat dstHsv = Mat::zeros(src.rows, src.cols, CV_8UC3);;
	hsvMeans = mean(hsvSrc);
	double hMean=0, sMean=0, vMean=0;
	hMean = hsvMeans.val[0];
	sMean = hsvMeans.val[1];
	vMean = hsvMeans.val[2];
	for(int i=0;i<hsvSrc.rows;i++){
		for(int j=0;j<hsvSrc.cols;j++){
			Vec3b pix = hsvSrc.at<Vec3b>(i, j);
			int H = static_cast<int>(pix[0]);
			int S = static_cast<int>(pix[1]);
			int V = static_cast<int>(pix[2]);
			
			if(S-sMean>satOffset && V>vMean && S>100 && V>100 && H<hMean && _src.at<uchar>(i, j)==255) {
				dstHsv.at<Vec3b>(i, j) = {255,255,255};
			}
		}
	}
	//printf("means:%lf,%lf,%lf\n",hMean, sMean, vMean);
	
	//形态学操作
	Mat element = getStructuringElement(MORPH_RECT, Size(50, 50));
	Mat element2 = getStructuringElement(MORPH_OPEN, Size(20, 20));
	Mat element3 = getStructuringElement(MORPH_CROSS, Size(15, 15));
//	GaussianBlur(dstHsv,dstHsv, Size(21,21), 1);
	dilate(dstHsv, dstHsv, element3);
	erode(dstHsv, dstHsv, element2);
	dilate(dstHsv, dstHsv, element);
	
	//findContours找轮廓
	std::vector<std::vector<Point>> contours;
	Mat dstHsvBinary;
	cvtColor(dstHsv,dstHsvBinary,COLOR_BGR2GRAY);
	findContours(dstHsvBinary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//int diff_cx = in_width, diff_cy = in_height;
	std::vector<int> diff_vec;
	diff_vec.push_back(in_width);
	diff_vec.push_back(in_height);
//	drawContours(dstHsv, contours, -1, Scalar(0,0,255), 5);
	for(int i=0;i<contours.size();i++){
		Mat ret;
		Mat demoTemp;
		RotatedRect r_rect = minAreaRect(contours[i]);
		//Rect stdrect = boundingRect(contours[i]);
//		Rect r_stdrect(stdrect.x+stdrect.width/2,stdrect.y+stdrect.height/2,stdrect.width/2,stdrect.height/2);
//		resize(demoTemplate, demoTemp, r_stdrect.size());
		//matchTemplate(src(stdrect),demoTemplate,ret, TM_CCOEFF_NORMED);
//		imshow("1",src(stdrect));
//		imshow("2",demoTemplate);
		//double maxMatch;
		//minMaxLoc(ret,new double,&maxMatch,new Point(),new Point());
		//printf("ret.row:%d, ret.col:%d, ret:%f\n",ret.rows, ret.cols, maxMatch);
		//drawContours(dstHsv, contours, i, Scalar(0,0,255), 5);
		//circle(dstHsv, r_rect.center, 5, Scalar(255,0,0), -1);
		//if(maxMatch<0.3)continue;
		Point2f vetPoints[4];
		r_rect.points(vetPoints);
		Point rc = r_rect.center;
		int rcx = rc.x;
		int rcy = rc.y;
		int diff_cx_temp = rcx - static_cast<int>(in_width/2);
		int diff_cy_temp = rcy - static_cast<int>(in_height/2);
		if(abs(diff_cx_temp)+abs(diff_cy_temp)<abs(diff_vec[0])+abs(diff_vec[1])){
			diff_vec[0] = diff_cx_temp;
			diff_vec[1] = diff_cy_temp;
		}
		for(int j=0;j<4;j++)line(dst, vetPoints[j],vetPoints[(j+1)%4],Scalar(255,0,0),5);
	}
	
//	imshow("hsvFrame", hsvSrc);
//	imshow("dstHsv", dstHsv);
	//setMouseCallback("hsvFrame", onMouse, (void*)&hsvSrc);
	return diff_vec;
}



