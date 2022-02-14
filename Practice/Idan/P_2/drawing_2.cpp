// drawing.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

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

#include <iostream>
Mat get_image(string path, int size) {
	Mat img = imread(path, IMREAD_COLOR);
	resize(img, img, Size(size, size));
	return img;
};

void draw_line() {
	Mat drawing = Mat::zeros(Size(200, 200), CV_8UC3);

	line(drawing, Point(0, 50), Point(50, 200), Scalar(0, 0, 255), 2, 8, 0);

	rectangle(drawing, Point(50, 50), Point(150, 150), Scalar(0, 255, 0), 2, 8, 0);
	imshow("drawing", drawing);
};

MatND hist(Mat img, int hist_size,int hist_w, int hist_h, int* channel_num, float min_range, float max_range,int num_bins) {
	
	MatND histogram;
	float channel_range[] = { min_range , max_range };
	const float* channel_rannges = channel_range;

	calcHist(&img, 1, 0, Mat(), histogram, 1, &num_bins, &channel_rannges); // check this

	// step of normalization
	Mat norm;

	Mat histImage(hist_w, hist_h, CV_8UC3, Scalar(0,0,0));
	normalize(histogram, histogram, 0, histImage.rows, NORM_MINMAX,-1, Mat());

	cout << histogram.size() << endl;
	for (int i = 1; i < hist_size; i++) {

		//line(histImgge, Point(num_bins* i, 0), Point(num_bins* i, hist_h - cvRound(norm.at<float>(i))), Scalar(0,0,0), 2, 8, 0);
		//checlk the difference
		int p_1 = num_bins * (i - 1);
		int p_2 = num_bins * (i);

		auto round1 = cvRound(histogram.at<float>(i - 1));
		auto round2 = cvRound(histogram.at<float>(i));

		auto p_12 = hist_h - round1;
		auto p_21 = hist_h - round2;

		line(histImage, Point(p_1, p_12), Point(p_2, p_21), Scalar(255, 0, 0), 2, 8, 0);
	}
	return histImage;



}

Mat equalitize_fuc(Mat src) {
	Mat dis;
	if (src.channels() > 1) 
	{
		cvtColor(src, src, COLOR_BGR2GRAY);
	}
	equalizeHist(src, dis);
	//imshow("Source", src);
	//imshow("dis", dis);

	return dis;
}


int main()
{
	// drawing hgistogram
	string path = "D:/github_/ComputerVision/Practice/Idan/P_1/course_1/resources/lambo.png";
	Mat img = get_image(path, 500);

	int histsize = 256;

	imshow("img_prior", img);

	//Mat histImgge = hist(img, 256, 512, 400, 0, 0, 256, cvRound((double)512 / 256));
	
	//Mat dis= equalitize_fuc(img);
	//imshow("dis", dis);
	//imshow("img", img);



	vector<Mat> bgr_planes;
	split(img, bgr_planes);

	float range[] = { 0, 256 }; //the upper boundary is exclusive
	const float* histRange[] = { range };

	int hist_size = 256;
	bool uniform = true, accumulate = false;

	Mat b_hist, g_hist, r_hist;
	calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &hist_size, histRange, uniform, accumulate);
	calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &hist_size, histRange, uniform, accumulate);
	calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &hist_size, histRange, uniform, accumulate);

	int hist_w = 512, hist_h = 400;
	int bin_w = cvRound((double)hist_w / histsize);
	cout << bin_w << endl;

	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
	normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

	// now we need to draw everything
	for (int i = 1; i < histsize; i++) {
		// everything at the moment is in the R channel: This is my Histogram
		auto value = b_hist.at<float>(i);
		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))), Scalar(255, 0, 0), 2, 8, 0);
		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
			Scalar(0, 255, 0), 2, 8, 0);
		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
			Scalar(0, 0, 255), 2, 8, 0
		);
	}
	imshow("histImage", histImage);

	waitKey(0);

	}

