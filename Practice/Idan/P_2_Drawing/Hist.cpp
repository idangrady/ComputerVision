// Hist.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>
#include <functional>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>s
#include<opencv2/core.hpp>
#include<Eigen/Dense>
//#include<opencv2/imgcodecs/include/opencv2/imgfhgcodecs.hpp>

using namespace cv;
using namespace std;
using namespace Eigen;

//
//bool areequal(const Mat img1, const Mat img2){
//	Mat temp;
//	if (img1.size != img2.size) {
//
//		return false;
//	}
//	else {
//		bitwise_xor(img1, img2, temp);
//		cout << temp.size() << endl;
//		temp= temp.reshape(-1,1);
//		cout << temp.size() << endl;
//		auto output = countNonZero(temp);
//		return output;
//	}
//	
//}
//void bitoperation() {
//	//const int* A = &hist_size;
//	int val = 10;
//	int val2 = 30;
//	const int* const A = &val;
//
//	cout << *A;
//}

class histogram {
public:
	Mat cal_histogram(Mat src) {
		Mat hist;
		Mat::zeros(256, 1, CV_32F);
		src.convertTo(src, CV_32F);
		double value = 0;
		for (int i = 0; i < src.rows; i++) {
			for (int n = 0; n < src.cols; n++) {
				auto value = src.at<float>(i, n);
				hist.at<float>(i, n) += 1;
			}
	}
		return hist;
	}
};

void draw_histogram(Mat hist) {
	Mat normlized;
	int hist_size = 256;
	int hist_w = 512, hist_h = 400;
	int bin_2 = cvRound((double)hist_w / hist_size);

	Mat hist_img(hist_w, hist_h,CV_8SC3, Scalar(0,0,0));
	Mat histogram_image(400, 500, CV_8UC3, Scalar(0, 0, 0));

	normalize(hist, hist, 0, 255, NORM_MINMAX,-1, Mat());
	//normalize(hist, normlized, 0, 400, NORM_MINMAX, -1, Mat()); // Check what is it about!

	for (int i = 0; i < hist_size; i++)
	{
//		rectangle(histogram_image, Point(2 * i, histogram_image.rows - normlized.at<float>(i)), Point(2 * (i + 1), histogram_image.rows), Scalar(255, 0, 0));
	}


}

void blend(Mat img1, Mat img2, float a) {
	Mat result;
	
	result = (a)* img1 + (1 - a) * img2;

	imshow("Blend", result);
}


int hist() {
	string path1 = "D:/github_/ComputerVision/Practice/Idan/P_1/course_1/resources/test.png";
	string path2 = "D:/github_/ComputerVision/Practice/Idan/P_1/course_1/resources/lambo.png";

	Mat img1 = imread(path1, IMREAD_COLOR);
	Mat img2 = imread(path2, IMREAD_COLOR);

	//imshow("Imgae1", img1);
	//imshow("Imgae2", img2);
	//Mat comp;


	//vector<Mat> bgr_planes;
	//split(img1, bgr_planes);
	Mat res, res1;
	resize(img1, res, Size(500, 500));
	resize(img2, res1, Size(500, 500));

	cout << res.size() << " " << res1.size();


	MatND histogram;
	int hisSize = 256;

	const int* channelNumber = { 0 };
	float channel_range[] = { 0, 256 };

	// check
	const float* channel_ranges = channel_range;
	int number_bins = hisSize;

	calcHist(&img1, 1, 0, Mat(), histogram, 1, &number_bins, &channel_ranges);

	int hist_w = 512, hist_h = 400;
	int bin_2 = cvRound((double)hist_h / 2);
	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

	imshow("Hist", histImage);
	waitKey(0);
	return 0;

}

