#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

const Size chessboardDim = Size(6, 9);
const float squareDim = 0.017f;


void createKnownBoardPositions(Size boardSize, float squareEdgelength, vector<Point3f>& corners) {

	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(Point3f(j * squareEdgelength, i * squareEdgelength, 0.0f));
		}
	}
}


void getChessBoardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false) {
	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		vector<Point2f> pointBuf;
		bool found = findChessboardCorners(*iter, chessboardDim, pointBuf, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

		if (found) {
			allFoundCorners.push_back(pointBuf);
		}
		if (showResults) {
			drawChessboardCorners(*iter, chessboardDim, pointBuf, found);
			imshow("Corners", *iter);
			waitKey(0);
		}
	}
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distortionCoff) {
	//2 dimensional because it's an image
	vector<vector<Point2f>> checkerboardImageSpacePoints;
	getChessBoardCorners(calibrationImages, checkerboardImageSpacePoints, false);

	//3 dimensional because it's the world
	vector<vector<Point3f>> worldSpaceCornerPoints(1);

	cout << "cameraCalibration 1";

	createKnownBoardPositions(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

	//rotation and translation
	vector<Mat> rVectors, tVectors;
	distortionCoff = Mat::zeros(8, 1, CV_64F);

	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distortionCoff, rVectors, tVectors);


}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients) {

	ofstream outStream(name);
	if (outStream) {
		uint16_t rows = cameraMatrix.rows;
		uint16_t columns = cameraMatrix.cols;

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
		}

		rows = distanceCoefficients.rows;
		columns = distanceCoefficients.cols;

		outStream << "distanceCoefficients" << endl;

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = distanceCoefficients.at<double>(r, c);
				outStream << value << endl;
			}
		}
		outStream.close();
		return true;
	}
	return false;
}


//void projectAxisPoints(vector<Mat> images) {
//	for fn
//}


void runCamera() {
	Mat frame;
	Mat drawToFrame;

	VideoCapture vid(0);
	if (!vid.isOpened()) { return; }

	int fps = 20;

	namedWindow("Webcam", WINDOW_AUTOSIZE);

	while (true) {
		if (!vid.read(frame))
			break;

		vector<Vec2f> foundPoints;
		bool found = false;

		found = findChessboardCorners(frame, chessboardDim, foundPoints, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		frame.copyTo(drawToFrame);
		drawChessboardCorners(drawToFrame, chessboardDim, foundPoints, found);
		if (found)
			imshow("Webcam with draws", drawToFrame);
		else
			imshow("webcam", frame);

		char character = waitKey(1000 / fps);

	}
}

vector<Mat> runImages() {
	string path = "Camera Roll/*.jpg";
	vector<String> images;
	glob(path, images);

	Mat frame;

	vector<Mat> imgs;

	for (int i = 0; i < images.size(); i++)
	{
		//cout << i;
		frame = imread(images[i]);
		imgs.push_back(frame);
	}

	cout << imgs.size();

	return imgs;

}


void main(int argv, char** argc) {

	Mat camMatrix = Mat::eye(3, 3, CV_64F);
	Mat distCoeff;

	vector<Mat> savedImages;

	vector<vector<Point2f>> markerCorners, rejectedCandidates;


	savedImages = runImages();

	cameraCalibration(savedImages, chessboardDim, squareDim, camMatrix, distCoeff);
	saveCameraCalibration("CameraCalibrationParams", camMatrix, distCoeff);


	//runCamera();

	waitKey(0);
}