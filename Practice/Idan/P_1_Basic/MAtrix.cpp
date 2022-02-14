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


int main() {
	const int max_value = 255;
	const int max_value_h = 360 / 2;
	int row_size = 3;
	int col_size = 3;

	Size matrix_size(row_size, col_size);
	cout << matrix_size << endl;

	Mat zero_matrix = Mat::zeros(matrix_size, CV_32FC1);
	cout << zero_matrix << endl;
	cout << zero_matrix.at<int>(1,1) << endl;

	for (int i = 0; i < (row_size ); i++) 
	{
		for (int n = 0; n < (row_size); n++)
		{
			zero_matrix.at<float>(n, i) = 1;
		}
	}
	cout << zero_matrix << endl;

	Mat I = Mat::eye(Size(3, 3), CV_32FC1);
	cout << I << endl;

	Mat result = I + I + Mat::ones(matrix_size, CV_32FC1) * 2 - (Mat_<float>(matrix_size) << 1, 1, 1, 0, 0, 0, 0, 0, 0);

	cout << result.size() << endl;
	cout<<result.type()<<endl;
	Mat second_once = Mat::ones(Size(3,2), CV_32FC1);
	cout << second_once.size() << endl;

	int a, b, c;
	a = 2; b = 7;
	c = (a > b) ? a: b;
	cout << c << endl;

	vector<int> lower = { 170,50,50 };
	int low_h = lower[0], low_s = lower[1],low_v = lower[2];
	int high_H = max_value_h, high_s = max_value,high_v = max_value;



	string path = "resources/test.png";
	string path2 = "resources/shapes.png";

	Mat img = imread(path, IMREAD_COLOR);
	Mat img2 = imread(path2, IMREAD_COLOR);

	resize(img, img ,Size(500, 500));
	resize(img2, img2, Size(500, 500));

	if (img.empty()) { cout << "Not Loaded" << endl;}
	
	//imshow("ImageBasic",img);
	//imshow("ImageBasic", img2);

	// perform aritmatic operations
	Mat img3, mask, range;
	
	// find min max
	double minVal, maxVal;
	Point minLoc, maxLoc;
	minMaxLoc(img, &minVal, &maxVal);
	cout << minVal << " " << maxVal << endl;


	//subtract(img, img2, img3, mask, -1);
	bitwise_and(img, img2, img3,mask);

	inRange(img, Scalar(low_h , low_s, low_v), Scalar(high_H, high_s, high_v), img3);
	
	imshow("Result", img3);
	
	
	waitKey(0);



	


}