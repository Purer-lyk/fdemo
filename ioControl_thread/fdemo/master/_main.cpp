#include "main.h"
#include "dirent.h"
#include "detection.h"
#include "control.h"
#include <wiringPi.h>

#define FIREDEMO "/home/liyankuan/文档/fdemo/build/fireDemo.jpg"
#define FIREFALSE "/home/liyankuan/文档/fdemo/build/fireFalse.png"
#define APPLE "/home/liyankuan/文档/fdemo/build/appele.png"
#define CAMFIRE "/home/liyankuan/文档/fdemo/build/Campfire.jpg"
#define FIREDEMO2 "/home/liyankuan/文档/fdemo/build/fireDemo2.jpg"
#define FIREDEMO3 "/home/liyankuan/文档/fdemo/build/fireDemo3.jpg"
#define FIREDEMO4 "/home/liyankuan/文档/fdemo/build/fireDemo4.jpg"
#define FIREDEDIFF "/home/liyankuan/文档/fdemo/build/fireDifficult.png"
#define FIREDEDIFF2 "/home/liyankuan/文档/fdemo/build/fireDifficult2.png"
#define SCENCE "/home/liyankuan/文档/fdemo/build/scence.png"
#define VIDEO "/home/liyankuan/文档/fdemo/build/fireDemo.mp4"

using namespace std;

void GetFileNames(string path,vector<string>& filenames) {
	DIR *pDir;
	struct dirent *ptr;
	if (!(pDir = opendir(path.c_str()))) {
		cout << "Folder doesn't Exist!" << endl;
		return;
	}
	while ((ptr = readdir(pDir)) != 0) {
		if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
			filenames.push_back(path + "/" + ptr->d_name);
		}
	}
	closedir(pDir);
}

int main(){
//	Detector det;
//	Mat frame;
//	Mat dst;
//	VideoCapture v(VIDEO);
//	while(v.isOpened()){
//		v.set(CAP_PROP_EXPOSURE, -4);//曝光 -4
//		v.read(frame);
//		if(frame.empty())break;
//		auto start = getTickCount();
//		det.mainDetect(frame, dst);
//		double fps = (double)(getTickCount() - start)/getTickFrequency();
//		printf("fps:%lf\n", 1/fps);
//		imshow("origin", frame);
//		imshow("result",dst);
//		if(waitKey(1)==27)break;
//	}

	WiringControl wControl;
	wControl.inOpen();
	
	int flipCount = 0;
	int rotate_56 = 0;//HIGH is left, LOW is right
	int rotate_2324 = 0;//HIGH is up, LOW is down
	wControl.rotateMotor_56(rotate_56);
	wControl.startMotor_56();
	wControl.rotateMotor_2324(rotate_2324);
	wControl.startMotor_2324();
	
	while(flipCount>=0){
		/*rotate_56 = !rotate_56;
		rotate_2324 = !rotate_2324;
		wControl.rotateMotor_56(rotate_56);
		wControl.rotateMotor_2324(rotate_2324);*/
		wControl.limitIO3();
		wControl.limitIO4();
		if(flipCount%2000==0){
			wControl.rotateMotor_56(rotate_56);
			wControl.rotateMotor_2324(rotate_2324);
			wControl.limitIO3();
			wControl.limitIO4();
			rotate_56 = !rotate_56;
			rotate_2324 = !rotate_2324;
			printf("rotate56:%d, rotate2324:%d\n", rotate_56, rotate_2324);
		}
		flipCount++;
		delay(1);
	}
	
	
//	std::string picPaths = "/home/liyankuan/文档/fdemo/build/JPEGImages";
//	vector<string> picnames;
//	GetFileNames(picPaths, picnames);
	
//	for(auto p:picnames){
//		frame = imread(p);
//		auto start = getTickCount();
//		det.mainDetect(frame, dst);
//		double fps = (double)(getTickCount() - start)/getTickFrequency();
//		printf("fps:%lf\n", 1/fps);
//		imshow("origin", frame);
//		imshow("result",dst);
//		while(waitKey(0)!=27){}
//	}
	
//	frame = imread(FIREDEDIFF2);;
//	auto start = getTickCount();
//	det.mainDetect(frame, dst);
//	double fps = (double)(getTickCount() - start)/getTickFrequency();
//	printf("fps:%lf\n", 1/fps);
//	imshow("origin", frame);
//	imshow("result",dst);
//	while(waitKey(0)!=27){}

    return 0;
}
