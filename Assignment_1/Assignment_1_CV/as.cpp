#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>

#include <string>
#include <iostream>
#include <sstream>
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include<tuple>
#include <fstream>


using namespace cv;
using namespace std;

const Size chessboardDim = Size(6, 9);
const float squareDim = 0.017f;
Size Image_size = Size(500, 500);
const float grid_width = squareDim * (chessboardDim.width - 1);


void createKnownBoardPositions(vector<Point3f>& corners) {
	cout << "createKnownBoardPositions" << endl;
	for (int i = 0; i < chessboardDim.height; i++)
	{
		for (int j = 0; j < chessboardDim.width; j++)
		{
			corners.push_back(Point3f(j * squareDim, i * squareDim, 0.0f));
		}
	}
}



void getChessBoardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = true) {
	cout << "getChessBoardCorners" << endl;
	int i = 0;
	int found_num = 0;
	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		vector<Point2f> pointBuf;
		bool found = findChessboardCorners(*iter, chessboardDim, pointBuf, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (found) {
			cout << "found " << found_num<<"/"<<i<< endl;
			TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
			cornerSubPix(images[i], pointBuf, cv::Size(11, 11), cv::Size(-1, -1), criteria);
			allFoundCorners.push_back(pointBuf);
			found_num= found_num +1;
		}
		if (showResults) {
			drawChessboardCorners(*iter, chessboardDim, pointBuf, found);
			imshow("Corners", *iter);
			waitKey(0);
		}
		i = i + 1;
	}
	cout << "Remining pics:" << allFoundCorners.size() << endl;
}

void cameraCalibration(vector<Mat> calibrationImages, Mat& cameraMatrix, Mat& distortionCoff) {
	//2 dimensional because it's an image
	vector<vector<Point2f>> checkerboardImageSpacePoints;
	getChessBoardCorners(calibrationImages, checkerboardImageSpacePoints, true);

	//3 dimensional because it's the world
	//vector<vector<Point3f>> objectPoints(1);

	vector<vector<Point3f>> objectPoints(1);

	createKnownBoardPositions(objectPoints[0]);
	objectPoints.resize(checkerboardImageSpacePoints.size(), objectPoints[0]);

	//rotation and translation
	vector<Mat> rVectors, tVectors;
	distortionCoff = Mat::zeros(8, 1, CV_64F);

	calibrateCamera(objectPoints, checkerboardImageSpacePoints, chessboardDim, cameraMatrix, distortionCoff, rVectors, tVectors);
}

//TODO: change this to save all cam params to XML file
bool saveCameraCalibration(string name, Mat parameter) {

	ofstream  outStream(name);
	if (outStream) {
		uint16_t rows = parameter.rows;
		uint16_t columns = parameter.cols;
		//cout << "original \n";
		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = parameter.at<double>(r, c);
				outStream << value << endl;
				//cout << value << "\n ";
			}
		}
		outStream.close();
		return true;
	}
	return false;
}

//code gotten from https://github.com/rohithjayarajan/chessboard-camera-pose/blob/master/app/CameraPose.cpp used only for comparison

//void drawAxes(cv::Mat& src_, cv::Mat& dst_,
//	std::vector<cv::Point2d>& imgPts_,
//	std::vector<cv::Point2f>& cornersSP_) {
//	src_.copyTo(dst_);
//	cv::arrowedLine(dst_, cornersSP_[0], imgPts_[0], cv::Scalar(0, 0, 255), 2,
//		cv::LINE_AA, 0);
//	cv::arrowedLine(dst_, cornersSP_[0], imgPts_[1], cv::Scalar(0, 255, 0), 2,
//		cv::LINE_AA, 0);
//	cv::arrowedLine(dst_, cornersSP_[0], imgPts_[2], cv::Scalar(255, 0, 0), 2,
//		cv::LINE_AA, 0);
//}
//
//void helper_RT_ImgPoints(cv::Mat& src_, cv::Mat& rvec_,
//	cv::Mat& tvec_,
//	std::vector<cv::Point2f>& corners_,
//	std::vector<cv::Point3d>& boardPts_, Mat cameraMatrix_, Mat distCoefcout_) {
//	cv::TermCriteria termcrit(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
//		30, 0.001);
//	cv::cornerSubPix(src_, corners_, cv::Size(11, 11), cv::Size(-1, -1),
//		termcrit);
//	cv::solvePnP(boardPts_, corners_, cameraMatrix_,
//		distCoefcout_, rvec_, tvec_);
//}
//
//bool poseEstimationImage(Mat frame_, Mat cameraMatrix_, Mat distCoefcout_) {
//	cv::Mat frame, gray, rvec, tvec, outputFrame;
//	std::vector<cv::Point2f> corners;
//	std::vector<cv::Point2d> imgPts;
//	std::vector<cv::Point3d> boardPts, axis;
//	for (int i = 0; i < chessboardDim.height; i++) {
//		for (int j = 0; j < chessboardDim.width; j++) {
//			boardPts.push_back(Point3d(j * squareDim, i * squareDim, 0));
//		}
//	}
//	axis.push_back(cv::Point3d(3 * squareDim, 0.0, 0.0));
//	axis.push_back(cv::Point3d(0.0, 3 * squareDim, 0.0));
//	axis.push_back(cv::Point3d(0.0, 0.0, -3 * squareDim));
//	//imshow("Image with corners_", frame_);
//	cvtColor(frame_, gray, COLOR_BGR2GRAY);
//	cv::findChessboardCorners(
//		gray, chessboardDim, corners,
//		cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
//	helper_RT_ImgPoints(gray, rvec, tvec, corners, boardPts, cameraMatrix_, distCoefcout_);/////////////////////////////////////////
//	cv::projectPoints(axis, rvec, tvec, cameraMatrix_, distCoefcout_,
//		imgPts, cv::noArray(), 0);
//	drawAxes(frame, outputFrame, imgPts, corners);
//	std::cout << "rotation " << std::endl;
//	std::cout << "x: " << rvec.at<double>(0, 0) * (180) << " degree"
//		<< std::endl
//		<< "y: " << rvec.at<double>(1, 0) * (180 ) << " degree"
//		<< std::endl
//		<< "z: " << rvec.at<double>(2, 0) * (180 ) << " degree"
//		<< std::endl;
//	std::cout << "translation " << std::endl;
//	std::cout << "x: " << tvec.at<double>(0, 0) << " meter" << std::endl
//		<< "y: " << tvec.at<double>(1, 0) << " meter" << std::endl
//		<< "z: " << tvec.at<double>(2, 0) << " meter" << std::endl;
//	cv::imshow("poseOP", outputFrame);
//	cv::waitKey(0);
//	return true;
//}


//bool calibrateCameraAndSaveParams(vector<Mat> calibrationImages, Mat& cameraMatrix, Mat& distortionCoff) {
//	cameraCalibration(calibrationImages, cameraMatrix, distortionCoff);
//	//cout << "$$$$$$$$$$$$$$$$$$$$$$$$$\n";
//	//cout << cameraMatrix.size();
//	//cout << "$$$$$$$$$$$$$$$$$$$$$$$$$\n";
//	saveCameraCalibration("cameraMatrix.txt", cameraMatrix);
//	saveCameraCalibration("distortionCoefficients.txt", distortionCoff);
//	return true;
//}

static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoefcout,
	vector<float>& perViewErrors, vector<double> &images_result)
{
	cout << "computeReprojectionErrors" << endl;

	vector<Point2f> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	
	perViewErrors.resize(objectPoints.size());
	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoefcout, imagePoints2);
		err = norm(imagePoints[i], imagePoints2, NORM_L2);
		//cout << "Image"<<i << " " << err << endl;
		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
		images_result.insert(images_result.begin(), err); // add them to the vector of results
	}
	double Mean_Projection_error = sqrt(totalErr / totalPoints);
	//cout << "Mean_Projection_error " << Mean_Projection_error << endl;
	//cout << "selected:" << endl;

	// sort the vector
	//for (int i = 0; i < images_result.size(); i++) {
	//	double *min = &images_result[i];
	//	for (int n = i; n < images_result.size(); n++) {
	//		double *current = &images_result[n];
	//		if (*current< *min) {
	//			auto c = current;
	//			current = min;
	//			min = c;
	//		}
	//	}
	//}

	//for (int i = 0; i < images_result.size(); i++) {
	//	auto curr_image_value = images_result[i];
	//	if (curr_image_value < Mean_Projection_error) {
	//		included.insert(included.begin(), true);
	//		//cout << "Image "<<i <<" :" << curr_image_value << endl;
	//	}
	//}

	return Mean_Projection_error;
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


vector<Mat> runImages() {
	cout << "runImages" << endl;
	string path = "D:/github_/Computer_Vision/Vlad_OnIdanPC/Pics/*.jpg";
	vector<String> images;
	Mat drawToFrame;

	glob(path, images);

	Mat frame, gray;
	vector<Mat> imgs;
	cout << "Imported Images: " << images.size() << endl;
	for (int i = 0; i < images.size(); i++)
	{
		frame = imread(images[i]);
		cvtColor(frame, frame, COLOR_BGR2GRAY);
		imgs.push_back(frame);
	}

	cout << "Detected Chessboard: " << imgs.size() << endl;;

	return imgs;

}


static bool runCalibration_new(vector<Mat> savedImages,  Mat& cameraMatrix, 
	Mat& distCoefcout,float grid_width, bool release_object, bool useFisheye)
{
	cout << "runCalibration_new" << endl;
	vector<Point3f> newObjPoints;
	vector<Mat> rVectors, tVectors;
	vector<Mat> rvecs, tvecs;

	
	//distCoefcout = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point2f>> imagePoints; // found corners for the chess on the image
	getChessBoardCorners(savedImages, imagePoints, false);

	//vector<vector<Point3f>> objectPoints(1);
	//createKnownBoardPositions(objectPoints[0]);

	vector<vector<Point3f>> objectPoints(1);
	createKnownBoardPositions(objectPoints[0]);

	//objectPoints[0][chessboardDim.width - 1].x = objectPoints[0][0].x + grid_width;
	//newObjPoints = objectPoints[0];

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//objectPoints[0][chessboardDim.width ].x = objectPoints[0][0].x + grid_width;
	//newObjPoints = objectPoints[0];



	//Find intrinsic and extrinsic camera parameters
	double rms=0;
	double totalAvgErr;
	vector<float>reprojErrs;
	Mat distortionCoff = Mat::zeros(8, 1, CV_64F);

	if (useFisheye) {
		Mat _rvecs, _tvecs;
		rms = fisheye::calibrate(objectPoints, imagePoints, chessboardDim, cameraMatrix, distCoefcout, _rvecs,
			_tvecs);

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
			iFixedPoint = chessboardDim.width - 1;

		cout << "Recalibration " << endl;
		rms = calibrateCamera(objectPoints, imagePoints, chessboardDim, cameraMatrix, distortionCoff, rVectors, tVectors);
	}
	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoefcout);
	vector<double> images_result;
	//objectPoints.clear();
	//objectPoints.resize(imagePoints.size(), newObjPoints);
	vector<vector<Point3f>> Remove_Object; vector<int> Remove_Indexed ;
	Mat rVectors_current, tVectors_current;
	for (int i = 0; i < imagePoints.size(); i++) {
		
		vector<Point3f> currentObject; vector<Point2f>currentImage;
		currentObject = objectPoints[objectPoints.size()-1];
		currentImage = imagePoints[imagePoints.size()-1];
		rVectors_current = rVectors[rVectors.size()-1];
		tVectors_current = tVectors[tVectors.size()-1];


		cout << imagePoints.size() << endl;

		//remove
		objectPoints.pop_back();
		imagePoints.pop_back();
		rVectors.pop_back();
		tVectors.pop_back();

		//print
		cout << imagePoints.size() << endl;
		double previous = rms;
		totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rVectors, tVectors, cameraMatrix,
			distortionCoff, reprojErrs, images_result);
		cout << totalAvgErr << endl;
		if (totalAvgErr > previous) {
			cout << "Value Decrease" << endl;
			objectPoints.insert(objectPoints.begin(), currentObject);
			imagePoints.insert(imagePoints.begin(),currentImage);
			cout << totalAvgErr << endl;
			tVectors.insert(tVectors.begin(), tVectors_current);
			rVectors.insert(rVectors.begin(), rVectors_current);
			previous = totalAvgErr;
		}
		//else {
		//	cout << "decline" << endl;
		//	objectPoints.push_back(currentObject);
		//	imagePoints.push_back(currentImage);

		//	//objectPoints.insert(Remove_Object.begin(), currentObject);
		
	}
	Mat distortionCoff_after = Mat::zeros(8, 1, CV_64F);
	Mat cameraMatrix_new = Mat::eye(3, 3, CV_64F);
	cout << "Left:" << imagePoints.size() << endl;
	cout << "Recalibration " << endl;
	cout << objectPoints.size() << endl;
	auto newrms = calibrateCamera(objectPoints, imagePoints, chessboardDim, cameraMatrix_new, distortionCoff_after, rVectors, tVectors);
	cout << "RMS after recalibration: " << newrms << endl;
	//recalibration after removal


	return ok;
}


//
//static bool runCalibration(vector<Mat> savedImages, Size& imageSize, vector<vector<Point3f>> objectPoints, Mat& cameraMatrix, Mat& distCoefcout,
//	vector<vector<Point2f>> imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
//	vector<float>& reprojErrs, double& totalAvgErr, vector<Point3f>& newObjPoints,
//	float grid_width, bool release_object, bool useFisheye)
//{
//	vector<vector<Point2f>> checkerboardImageSpacePoints;
//	getChessBoardCorners(savedImages, checkerboardImageSpacePoints, false);
//
//
//	cout << objectPoints[0] << endl;
//	createKnownBoardPositions(objectPoints[0]);
//	objectPoints.resize(imagePoints.size(), objectPoints[0]);
//
//
//	//objectPoints[0][chessboardDim.width ].x = objectPoints[0][0].x + grid_width;
//	//newObjPoints = objectPoints[0];
//
//
//
//	//Find intrinsic and extrinsic camera parameters
//	double rms = 0;
//
//	if (useFisheye) {
//		Mat _rvecs, _tvecs;
//		rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoefcout, _rvecs,
//			_tvecs);
//
//		rvecs.reserve(_rvecs.rows);
//		tvecs.reserve(_tvecs.rows);
//		for (int i = 0; i < int(objectPoints.size()); i++) {
//			rvecs.push_back(_rvecs.row(i));
//			tvecs.push_back(_tvecs.row(i));
//		}
//	}
//	else {
//		int iFixedPoint = -1;
//		if (release_object)
//			//iFixedPoint = chessboardDim.width - 1;
//
//			vector<vector<Mat>> Word_space_Plus_intrinsic;
//		Mat distortionCoff = Mat::zeros(8, 1, CV_64F);
//		vector<Mat> rVectors, tVectors;
//
//		calibrateCamera(objectPoints, checkerboardImageSpacePoints, chessboardDim, cameraMatrix, distortionCoff, rVectors, tVectors);
//
//		rms = calibrateCameraRO(checkerboardImageSpacePoints, imagePoints, imageSize, iFixedPoint,
//			cameraMatrix, distCoefcout, rvecs, tvecs, newObjPoints);
//	}
//
//	if (release_object) {
//		cout << "New board corners: " << endl;
//		cout << newObjPoints[0] << endl;
//		cout << newObjPoints[chessboardDim.width - 1] << endl;
//		cout << newObjPoints[chessboardDim.width * (chessboardDim.height - 1)] << endl;
//		cout << newObjPoints.back() << endl;
//	}
//
//	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;
//
//	bool ok = checkRange(cameraMatrix) && checkRange(distCoefcout);
//
//	objectPoints.clear();
//	objectPoints.resize(imagePoints.size(), newObjPoints);
//	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
//		distCoefcout, reprojErrs);
//
//	return ok;
//}

//
//bool runCalibrationAndSave(Size imageSize, Mat& cameraMatrix, Mat& distCoefcout,
//	vector<vector<Point2f> > imagePoints, float grid_width, bool release_object)
//{
//	vector<Mat> rvecs, tvecs;
//	vector<float> reprojErrs;
//	double totalAvgErr = 0;
//	vector<Point3f> newObjPoints;
//
//	bool ok = runCalibration(imageSize, cameraMatrix, distCoefcout, imagePoints, rvecs, tvecs, reprojErrs,
//		totalAvgErr, newObjPoints, grid_width, release_object);
//	cout << (ok ? "Calibration succeeded" : "Calibration failed")
//		<< ". avg re projection error = " << totalAvgErr << endl;
//
//	if (ok)
//		saveCameraParams(imageSize, cameraMatrix, distCoefcout, rvecs, tvecs, reprojErrs, imagePoints,
//			totalAvgErr, newObjPoints);
//	return ok;
//}

void main(int argv, char** argc) {

	Mat camMatrix = Mat::eye(3, 3, CV_64F);
	Mat distCoeff= Mat::zeros(8, 1, CV_64F);

	vector<Mat> savedImages;

	//vector<vector<Point2f>> markerCorners, rejectedCandidates;

	//  OFFLINE  //
	//return only the pictures that a cessboard is found within them
	savedImages = runImages();

	runCalibration_new(savedImages, camMatrix, distCoeff, 0,true,false);


	//runCamera();

	//calibrateCameraAndSaveParams(savedImages, camMatrix, distCoeff);

	cout << "Calibrated! ";

	//  ONLINE  //
	// 
	//TODO: change this to read all cam params from XML file


	fstream  cameraMatrix;
	vector<float> aux;
	Mat cameraMatrixValues = Mat::eye(3, 3, CV_64F);
	cameraMatrix.open("cameraMatrix.txt", ios::in);
	if (cameraMatrix) {
		double value;

		// read the elements in the file into a vector  
		while (cameraMatrix >> value) {
			aux.push_back(value);
		}
	}
	int z = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cameraMatrixValues.at<double>(i, j) = aux[z];
			z++;
		}
	}

	for (int r = 0; r < 3; r++)
	{
		for (int c = 0; c < 3; c++)
		{
			double value = cameraMatrixValues.at<double>(r, c);
			//cout << value << "\n ";
		}
	}


	//cout << cameraMatrixValues;

	cameraMatrix.close();


	fstream distortionCoefficients;
	Mat distortionCoefficientsValues;
	distortionCoefficients.open("distortionCoefficients.txt", ios::in);
	if (distortionCoefficients) {
		double value;

		// read the elements in the file into a vector  
		while (distortionCoefficients >> value) {
			distortionCoefficientsValues.push_back(value);
		}
	}

	//cout << distortionCoefficientsValues;

	distortionCoefficients.close();


	Mat img;
	savedImages[20].copyTo(img);
	//poseEstimationImage(img, cameraMatrixValues, distortionCoefficientsValues);


	Mat imgCorners;
	Mat imgGray;
	vector<Point2f> corners;
	vector<Vec2f> corners2;
	vector<Point3f> objectPoints;
	vector<Point2f> imagePoints;
	Mat rVectors, tVectors, rotMtx;
	bool foundCorner;
	cvtColor(img, imgGray, COLOR_BGR2GRAY);
	TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);

	foundCorner = findChessboardCorners(img, chessboardDim, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);


	if (foundCorner) {
		drawChessboardCorners(img, chessboardDim, corners, foundCorner);
		//imshow("Image with corners", imgCorners);
		cornerSubPix(imgGray, corners, Size(11, 11), Size(-1, -1), criteria);

		//createKnownBoardPositions(objectPoints);

		solvePnP(objectPoints, corners, cameraMatrixValues, distortionCoefficientsValues, rVectors, tVectors);

		cout << "rVectors\n";
		cout << rVectors;

		//cout << "------objectPoints-------";
		//cout << objectPoints.size();
		//cout << "------corners-------";
		//cout << corners.size();
		//cout << "------cameraMatrixValues-------";
		//cout << cameraMatrixValues.size();
		//cout << "-----distortionCoefficientsValues--------";
		//cout << distortionCoefficientsValues.size();


		vector<Point3f> axis;

		axis.push_back(Point3f(3.0 * squareDim, 0.0, 0.0));
		axis.push_back(Point3f(0.0, 3.0 * squareDim, 0.0));
		axis.push_back(Point3f(0.0, 0.0, -3.0 * squareDim));
		//axis.resize(-1,3);
		projectPoints(axis, rVectors, tVectors, cameraMatrixValues, distortionCoefficientsValues, imagePoints);

		/*	cout << "\n \n";
			cout << imagePoints;*/


		arrowedLine(img, corners[0], imagePoints[0], Scalar(255, 0, 0), 2, LINE_8);
		arrowedLine(img, corners[0], imagePoints[1], Scalar(0, 255, 0), 2, LINE_8);
		arrowedLine(img, corners[0], imagePoints[2], Scalar(0, 0, 255), 2, LINE_8);
		imshow("Output", img);
		//Rodrigues(rVectors, rotMtx);
		//cout << rotMtx.size() << " \n";
		//cout << rotMtx;

	}

	waitKey(0);
}