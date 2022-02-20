#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/features2d.hpp>
#include<opencv2/photo.hpp>
#include<opencv2/core/mat.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/core/affine.hpp"
#include<opencv2/imgcodecs/include/opencv2/imgcodecs.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include<tuple>


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>



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

tuple<vector<vector<Point3f>>, vector<vector<Point2f>>, vector<vector<Mat>>> cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distortionCoff) {
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
	vector<vector<Mat>> Word_space_Plus_intrinsic;
	distortionCoff = Mat::zeros(8, 1, CV_64F);

	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distortionCoff, rVectors, tVectors);
	Word_space_Plus_intrinsic.push_back(rVectors);
	Word_space_Plus_intrinsic.push_back(tVectors);

	return make_tuple(worldSpaceCornerPoints, checkerboardImageSpacePoints, Word_space_Plus_intrinsic);
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

static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors, bool fisheye)
{
	vector<Point2f> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());
	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		if (fisheye)
		{
			fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
				distCoeffs);
		}
		else
		{
			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		}
		err = norm(imagePoints[i], imagePoints2, NORM_L2);
		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}
	return std::sqrt(totalErr / totalPoints);
}

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

vector<Mat> sep_T_R(Mat camera_Cal) {
	vector<Mat> output;
	Mat rvecsL = Mat::zeros(cv::Size(3, 3), CV_64FC1) , tvecsL= Mat::zeros(cv::Size(3, 1), CV_64FC1);
	for (int i = 0; i < camera_Cal.rows; i++) {
		for (int j = 0;  j < camera_Cal.cols; j++) {
			if (j > 1) {
				tvecsL.at<double>(0,j) = camera_Cal.at<double>(i,j);
			}
			else {
				rvecsL.at<double>(i,j) = camera_Cal.at<double>(i, j);
			}
		}
	}
	output.push_back(rvecsL);
	output.push_back(tvecsL);
	return(output);
}



vector<Mat> runImages() {
	string path = "D:/github_/ComputerVision/Practice/Vlad/Assignment_1_CV/Assignment_1_CV/Camera Roll/WIN_20220216_11_22_18_Pro.jpg";
	vector<String> images;
	glob(path, images);

	Mat frame, gray;

	vector<Mat> imgs;
	std::vector<cv::Point2f> corner_pts;

	for (int i = 0; i < images.size(); i++)
	{
		//cout << i;
		frame = imread(images[i]);
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		
		// add image only when the the board is detected
		bool success = findChessboardCorners(gray, Size(6, 9), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		if (success){
			//refine the image fixels
			TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
			cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

			imgs.push_back(frame);
		}
	}

	cout << imgs.size();

	return imgs;

}

static bool runCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
	vector<float>& reprojErrs, double& totalAvgErr, vector<Point3f>& newObjPoints,
	float grid_width, bool release_object)
{
	//! [fixed_aspect]
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	//! [fixed_aspect]

	vector<vector<Point3f> > objectPoints(1);
	calcBoardCornerPositions(boardSize, squareSize, objectPoints[0], s.calibrationPattern);
	objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
	newObjPoints = objectPoints[0];

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms;

	if (s.useFisheye) {
		Mat _rvecs, _tvecs;
		rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
			_tvecs, s.flag);

		rvecs.reserve(_rvecs.rows);
		tvecs.reserve(_tvecs.rows);
		for (int i = 0; i < int(objectPoints.size()); i++) {
			rvecs.push_back(_rvecs.row(i));
			tvecs.push_back(_tvecs.row(i));
		}
	}
	else {
		int iFixedPoint = -1;
		if (release_object)
			iFixedPoint = s.boardSize.width - 1;
		rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
			cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
	}; 

	if (release_object) {
		cout << "New board corners: " << endl;
		cout << newObjPoints[0] << endl;
		cout << newObjPoints[s.boardSize.width - 1] << endl;
		cout << newObjPoints[s.boardSize.width * (s.boardSize.height - 1)] << endl;
		cout << newObjPoints.back() << endl;
	}

	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	objectPoints.clear();
	objectPoints.resize(imagePoints.size(), newObjPoints);
	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
		distCoeffs, reprojErrs, s.useFisheye);

	return ok;
}

void main(int argv, char** argc) {

	Mat camMatrix = Mat::eye(3, 3, CV_64F);
	Mat distCoeff;

	vector<Mat> savedImages;

	vector<vector<Point2f>> markerCorners, rejectedCandidates;


	savedImages = runImages();
	vector<vector<Point3f>> word_coordinate;
	vector<vector<Point2f>> ImageSpacePoints;

	vector<vector<Mat>> Word_space_Plus_intrinsic;

	tie(word_coordinate, ImageSpacePoints, Word_space_Plus_intrinsic)= cameraCalibration(savedImages, chessboardDim, squareDim, camMatrix, distCoeff);
	//vector<Mat> outputcalibration = sep_T_R(camMatrix);
	vector<Mat> rvecsL = Word_space_Plus_intrinsic[0], tvecsL = Word_space_Plus_intrinsic[1];
	//rvecsL = outputcalibration[0];
	//tvecsL = outputcalibration[1];

	saveCameraCalibration("CameraCalibrationParams", camMatrix, distCoeff);
	cout << camMatrix << endl;
	cout << distCoeff << endl;
	
	bool ok = checkRange(camMatrix) && checkRange(distCoeff);
	
	if (ok == false) {
		cout << "Not Ok, there is a problem during the callibration" << endl;
	}
	else {

		vector<Mat> rvecs, tvecs;
		vector<float> reprojErrs;
		double totalAvgErr = 0;
		vector<Point3f> newObjPoints;

		bool ok = runCalibration(ImageSpacePoints, chessboardDim, Size(squareDim, squareDim), &camMatrix, &distCoeff, imagePoints, rvecs, tvecs, reprojErrs,
			totalAvgErr, newObjPoints, grid_width, release_object);
		cout << (ok ? "Calibration succeeded" : "Calibration failed")
			<< ". avg re projection error = " << totalAvgErr << endl;
		if (ok)
			saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
				totalAvgErr, newObjPoints);
		auto totalAvgErr = computeReprojectionErrors(objectPoints, imagePointsL, rvecsL, tvecsL, camMatrix, distCoeff);

	}
	////runCamera();

	waitKey(0);
}