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

void blur_kernel(Mat srs) {
	for (int i = 0; i < srs.rows; i++) {
		cout << srs.at<Vec3b>(i, 1) << endl;;
	}
}


int main(){

	string path = "resources/test.png";
	Mat img = imread(path);
	Mat imgGray;
	Mat ImgageBlur, Canny, Dilate;
	Mat Kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

	//Matrix<float, 1, 4 > b{0.25,.25,.25,.25};
	//cout<<b.reshaped(2,2)<<endl;
	//cvtColor(img, imgGray, COLOR_BGR2GRAY);

	//dilate(imgGray, Dilate, (3, 3));

	//blur_kernel(imgGray);

}

