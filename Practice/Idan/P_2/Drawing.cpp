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

#define w 400

using namespace cv;
using namespace std;
using namespace Eigen;


    //void MyEllipse(Mat img, double angle);
    //void MyFilledCircle(Mat img, Point center);
    //void MyPolygon(Mat img);
    //void MyLine(Mat img, Point start, Point end);


    //int main(void) {
    //    char atom_window[] = "Drawing 1: Atom";
    //    char rook_window[] = "Drawing 2: Rook";
    //    Mat atom_image = Mat::zeros(w, w, CV_8UC3);
    //    Mat rook_image = Mat::zeros(w, w, CV_8UC3);
    //    MyEllipse(atom_image, 90);
    //    MyEllipse(atom_image, 0);
    //    MyEllipse(atom_image, 45);
    //    MyEllipse(atom_image, -45);
    //    MyFilledCircle(atom_image, Point(w / 2, w / 2));
    //    MyPolygon(rook_image);
    //    rectangle(rook_image,
    //        Point(0, 7 * w / 8),
    //        Point(w, w),
    //        Scalar(0, 255, 255),
    //        Mat::ones(),
    //        LINE_8);
    //    MyLine(rook_image, Point(0, 15 * w / 16), Point(w, 15 * w / 16));
    //    MyLine(rook_image, Point(w / 4, 7 * w / 8), Point(w / 4, w));
    //    MyLine(rook_image, Point(w / 2, 7 * w / 8), Point(w / 2, w));
    //    MyLine(rook_image, Point(3 * w / 4, 7 * w / 8), Point(3 * w / 4, w));
    //    imshow(atom_window, atom_image);
    //    moveWindow(atom_window, 0, 200);
    //    imshow(rook_window, rook_image);
    //    moveWindow(rook_window, w, 200);
    //    waitKey(0);
    //    return(0);
    //}


void drawing() {

    Mat drawing;
    drawing = Mat::zeros(Size(300, 300), CV_8UC3);

    //line(drawing, Point(0, 50), Point(200, 50), Scalar(0, 255,0), 2, 1, 0);
    //rectangle(drawing, Point(0, 20), Point(200, 100), Scalar(255,0,0), 2, 1, 0);

    imshow("Image", drawing);
    

};