#include "paddle_detection.h"


paddleDetector::paddleDetector(std::string model_file, std::string re_file, double imgLight, int yco, float thres, float rethres):
in_width(320),
in_height(320),
re_width(32),
re_height(32),
threshold(thres),
_threshold(rethres),
traceCx(in_width/2),
traceCy(in_height/2),
ycOffset(yco),
convertLight(imgLight),
save_cnt(0)
{
	categories.push_back("fire");
	// 1. Set MobileConfig
	config.set_model_from_file(model_file);
	config.set_power_mode(LITE_POWER_NO_BIND);
	config.set_threads(2);
	
	// 2. Create PaddlePredictor by MobileConfig
	predictor = CreatePaddlePredictor<MobileConfig>(config);

	//recognization
	reconfig.set_model_from_file(re_file);
	reconfig.set_power_mode(LITE_POWER_NO_BIND);
	config.set_threads(2);
	repredictor = CreatePaddlePredictor<MobileConfig>(reconfig);
}

paddleDetector::~paddleDetector(){}

std::vector<Object> paddleDetector::RunModel(cv::Mat &img){
	pre_initial(img);
	wScale = (float)img.cols/(float)in_width;
	hScale = (float)img.rows/(float)in_height;
	traceCx = img.cols/2;
	traceCy = img.rows/2+ycOffset;
	// 3. Prepare input data from image
	// input 0
	std::unique_ptr<Tensor> input_tensor0(std::move(predictor->GetInput(0)));
	input_tensor0->Resize({1, 2});
	auto* data0 = input_tensor0->mutable_data<float>();
	data0[0] = img.cols;
	data0[1] = img.rows;
	
	// input 1
	std::unique_ptr<Tensor> input_tensor1(std::move(predictor->GetInput(1)));
	input_tensor1->Resize({1, 3, in_height, in_width});
	auto* data1 = input_tensor1->mutable_data<float>();
	pre_process(img, in_width, in_height, data1);
	
	// input 2
	std::unique_ptr<Tensor> input_tensor2(std::move(predictor->GetInput(2)));
	input_tensor2->Resize({1, 2});
	auto* data2 = input_tensor2->mutable_data<float>();
	data2[0] = wScale;
	data2[1] = hScale;
	
	// 4. Run predictor
	predictor->Run();
	
	// 5. Get output and post process
	std::unique_ptr<const Tensor> output_tensor(
			std::move(predictor->GetOutput(0)));
	auto* outptr = output_tensor->data<float>();
	auto shape_out = output_tensor->shape();
	//yolov3总共输出?x6,第一个种类,第二个置信度,后四个为bbox的坐标信息
	int64_t cnt = 1;
	for (auto& i : shape_out) {
		cnt *= i;
	}
	//除6获取bbox个数
	auto rec_out = detect_object(outptr, static_cast<int>(cnt / 6), threshold, img);
	return rec_out;
}

void paddleDetector::pre_initial(cv::Mat& img){
	cv::Mat hsvDst;
	cvtColor(img, hsvDst, cv::COLOR_BGR2HSV);
	cv::Scalar avg = cv::mean(hsvDst);
	//printf("mean v:%f\n", avg.val[2]);
	float brightScale = convertLight/avg.val[2];
	if(avg.val[2]>100)convertScaleAbs(img, img, brightScale);
}

std::vector<Object> paddleDetector::detect_object(const float* data,
												  int count,
												  float thresh,
												  cv::Mat& image) {  // NOLINT
	if (data == nullptr) {
		return std::vector<Object>();
	}
	std::vector<Object> rect_out;
	//printf("count:%d\n", count);
	for (int iw = 0; iw < count; iw++) {
		//int oriw = image.cols;
		//int orih = image.rows;
		/*for(int i=0;i<6;i++){
			printf("data%d:%f\n",i,data[i]);
		}*/
		if (data[1] > thresh) {
			Object obj;
			int x = static_cast<int>(data[2]*wScale);
			int y = static_cast<int>(data[3]*hScale);
			int w = static_cast<int>(data[4]*wScale - data[2]*wScale + 1);
			int h = static_cast<int>(data[5]*hScale - data[3]*hScale + 1);
			int cx = static_cast<int>((data[2]*wScale+data[4]*wScale)/2);
			int cy = static_cast<int>((data[3]*wScale+data[5]*wScale)/2);
			cv::Rect rec_clip =
					cv::Rect(x, y, w, h) & cv::Rect(0, 0, image.cols, image.rows);
			obj.class_id = static_cast<int>(data[0]);
			obj.prob = data[1];
			obj.rec = rec_clip;
			
			//trace
			obj.diff_cx = traceCx-cx;
			obj.diff_cy = traceCy-cy;

			//recognization
			cv::Mat _image = image(rec_clip);
			bool reCondition = recognize(_image);
			
			if (w > 0 && h > 0 && obj.prob <= 1 && reCondition) {
				save_cnt++;
				char imgName[20];
				sprintf(imgName, "%d.jpg", save_cnt);
				cv::imwrite(imgName, image(rec_clip));
				
				rect_out.push_back(obj);
				cv::rectangle(image, rec_clip, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
				
				std::string str_prob = std::to_string(obj.prob);
				std::string text = std::string(categories[obj.class_id]) + ": " +
								   str_prob.substr(0, str_prob.find(".") + 4);
				int font_face = cv::FONT_HERSHEY_COMPLEX_SMALL;
				double font_scale = 1.f;
				int thickness = 1;
				cv::Size text_size =
						cv::getTextSize(text, font_face, font_scale, thickness, nullptr);
				float new_font_scale = w * 0.5 * font_scale / text_size.width;
				text_size = cv::getTextSize(
						text, font_face, new_font_scale, thickness, nullptr);
				cv::Point origin;
				origin.x = x + 3;
				origin.y = y + text_size.height + 3;
				cv::putText(image,
							text,
							origin,
							font_face,
							new_font_scale,
							cv::Scalar(0, 255, 255),
							thickness,
							cv::LINE_AA);
				
				/*std::cout << "detection, image size: " << image.cols << ", "
						  << image.rows
						  << ", detect object: " << categories[obj.class_id]
						  << ", score: " << obj.prob << ", location: x=" << x
						  << ", y=" << y << ", width=" << w << ", height=" << h
						  << std::endl;*/
			}
		}
		data += 6;
	}
	return rect_out;
}

bool paddleDetector::recognize(cv::Mat& img){
	// input 0
	std::unique_ptr<Tensor> input_tensor1(std::move(repredictor->GetInput(0)));
	input_tensor1->Resize({1, 3, re_height, re_width});
	auto* data1 = input_tensor1->mutable_data<float>();
	pre_reprocess(img, re_width, re_height, data1);

	// Run
	repredictor->Run();

	// Get output
	std::unique_ptr<const Tensor> output_tensor(
			std::move(repredictor->GetOutput(0)));
	auto* outptr = output_tensor->data<float>();
	int cnt=1;
	auto shape_out = output_tensor->shape();
	for (auto& i : shape_out) {
		cnt *= i;
	}
	//printf("cnt:%d\n", cnt);
	if(outptr[0]>outptr[1]){
		printf("true:%f,%f\n",outptr[0],outptr[1]);
		return true;
	}
	printf("false:%f,%f\n",outptr[0],outptr[1]);
	return false;
}

void paddleDetector::pre_process(const cv::Mat& img, int width, int height, float* data) {
	cv::Mat rgb_img;
	cv::cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);
	cv::resize(
			rgb_img, rgb_img, cv::Size(width, height), 0.f, 0.f, cv::INTER_CUBIC);
	cv::Mat imgf;
	rgb_img.convertTo(imgf, CV_32FC3, 1 / 255.f);
	std::vector<float> mean = {0.485f, 0.456f, 0.406f};
	std::vector<float> scale = {0.229f, 0.224f, 0.225f};
	const float* dimg = reinterpret_cast<const float*>(imgf.data);
	neon_mean_scale(dimg, data, width * height, mean, scale);
}
	
void paddleDetector::pre_reprocess(const cv::Mat& img, int width, int height, float* data){
	cv::Mat rgb_img;
	img.copyTo(rgb_img);
	//cv::cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);
	cv::resize(
			rgb_img, rgb_img, cv::Size(width, height), 0.f, 0.f, cv::INTER_CUBIC);
	cv::Mat imgf;
	rgb_img.convertTo(imgf, CV_32FC3, 1.f/255.f);
	std::vector<float> mean = {0.5f, 0.5f, 0.5f};
	std::vector<float> scale = {0.5f, 0.5f, 0.5f};
	const float* dimg = reinterpret_cast<const float*>(imgf.data);
	neon_mean_scale(dimg, data, width * height, mean, scale);
}

void paddleDetector::neon_mean_scale(const float* din, float* dout, int size, const std::vector<float> mean, const std::vector<float> scale){
	if (mean.size() != 3 || scale.size() != 3) {
		std::cerr << "[ERROR] mean or scale size must equal to 3\n";
		exit(1);
	}
	float32x4_t vmean0 = vdupq_n_f32(mean[0]);
	float32x4_t vmean1 = vdupq_n_f32(mean[1]);
	float32x4_t vmean2 = vdupq_n_f32(mean[2]);
	float32x4_t vscale0 = vdupq_n_f32(1.f / scale[0]);
	float32x4_t vscale1 = vdupq_n_f32(1.f / scale[1]);
	float32x4_t vscale2 = vdupq_n_f32(1.f / scale[2]);
	
	float* dout_c0 = dout;
	float* dout_c1 = dout + size;
	float* dout_c2 = dout + size * 2;
	
	int i = 0;
	//vld3q_f32获取连续的3个4数数组,对应Vec3b图像数据
	for (; i < size - 3; i += 4) {
		float32x4x3_t vin3 = vld3q_f32(din);
		float32x4_t vsub0 = vsubq_f32(vin3.val[0], vmean0);//B通道的四个数
		float32x4_t vsub1 = vsubq_f32(vin3.val[1], vmean1);//G通道的四个数
		float32x4_t vsub2 = vsubq_f32(vin3.val[2], vmean2);//R通道的四个数
		float32x4_t vs0 = vmulq_f32(vsub0, vscale0);
		float32x4_t vs1 = vmulq_f32(vsub1, vscale1);
		float32x4_t vs2 = vmulq_f32(vsub2, vscale2);
		//连续赋值4个值
		vst1q_f32(dout_c0, vs0);
		vst1q_f32(dout_c1, vs1);
		vst1q_f32(dout_c2, vs2);
		
		din += 12;
		dout_c0 += 4;
		dout_c1 += 4;
		dout_c2 += 4;
	}
	//最后不足三个的单独计算
	for (; i < size; i++) {
		*(dout_c0++) = (*(din++) - mean[0]) * scale[0];
		*(dout_c1++) = (*(din++) - mean[1]) * scale[1];
		*(dout_c2++) = (*(din++) - mean[2]) * scale[2];
	}
}
