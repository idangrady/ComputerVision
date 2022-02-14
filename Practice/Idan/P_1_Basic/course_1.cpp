// course_1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>
#include <functional>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/core.hpp>
//#include<opencv2/imgcodecs/include/opencv2/imgcodecs.hpp>

using namespace cv;
using namespace std;


void print_img(string path) {
    Mat img = imread(path);

    imshow("Image", img);
    waitKey(0);
}

void show_video(string path) 
{
    cout << path << endl;
    VideoCapture cap(0);
    Mat img;

    while (true) {
        cap.read(img);
        imshow("Video",img);
        waitKey(20);
    }

}
int main()
{
    string img_path = "resources/test.png";
    string video_show = "resources/test_video.mp4";
    show_video(video_show);

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
