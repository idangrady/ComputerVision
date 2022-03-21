#include <cstdlib>
#include <string>

#include "utilities/General.h"
#include "VoxelReconstruction.h"


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/video/background_segm.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <random>



using namespace cv;
using namespace std;

const Size chessboardDim = Size(8, 6);
const float squareDim = 115;
Mat frame;
vector<Point2f> TouchPoints;

using namespace nl_uu_science_gmt;



void createKnownBoardPositions(vector<Point3f>& corners) {

	for (int i = 0; i < chessboardDim.height; i++)
	{
		for (int j = 0; j < chessboardDim.width; j++)
		{
			corners.push_back(Point3f(j * squareDim, i * squareDim, 0.0f));
		}
	}
}


static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints, const vector<vector<Point2f> >& imagePoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const Mat& cameraMatrix, const Mat& distCoefcout, vector<float>& perViewErrors, vector<double>& images_result)
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



void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		Point2f curpoint;
		curpoint.x = x;
		curpoint.y = y;
		TouchPoints.push_back(curpoint);
	}
}


void print_image(string path) {
	VideoCapture cap(path);
	!cap.read(frame);
	if (!cap.isOpened()) { return; }


	namedWindow("My Window", 1);
	Point2f points;
	//set the callback function for any mouse event
	setMouseCallback("My Window", CallBackFunc, NULL);
	//show the image
	imshow("My Window", frame);
	Mat canvas;
	while (TouchPoints.size() < 4) {
		waitKey(10);
		canvas = frame.clone();
		if (TouchPoints.size() > 0)
			for (size_t c = 0; c < TouchPoints.size(); c++)
			{
				circle(canvas, TouchPoints[c], 4, Color_MAGENTA, 1, 8);
			}
		//circle(frame, TouchPoints.back(), 4, Color_MAGENTA, 1, 8);

	}
}


void transform_point(Point2f A, Point2f B, vector<Point2f>& new_loc, int sagment) {
	Point2f d;
	d.x = (B.x - A.x) / sagment;
	d.y = (B.y - A.y) / sagment;

	//for (int i = sagment; i >=0; i--) {
	//	cout << A + i * d << endl;
	//	new_loc.push_back(A + i * d);
	//}
	for (int i = 0; i <= sagment; i++) {
		cout << A + i * d << endl;
		new_loc.push_back(A + i * d);
	}

	//for (int i = 0 ; i< sagment; i++) {
	//	cout << new_loc[i] << endl;
	//}
}
void mouse_callback(int event, int x, int y, int flags, void* param)
{
	switch (event)
	{
	case CV_EVENT_MOUSEMOVE: {
		// 
		cout << "CV_EVENT_MOUSEMOVE" << endl;
	}
						   break;
	case CV_EVENT_LBUTTONDOWN: {
		//left
		cout << "Left-Mouse--- Add Point" << (x, y) << endl;
		Point2f curpoint;
		curpoint.x = x;
		curpoint.y = y;
		TouchPoints.push_back(curpoint);
		break;
	}

	}
}


bool save_parameters2(string name, vector<Point2f> Points) {
	FileStorage fs;
	cout << name << endl;
	fs.open(name, FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "CornersAmount" << (int)Points.size();
		for (size_t b = 0; b < Points.size(); ++b)
		{
			stringstream corner_id;
			corner_id << "Corner_" << b;
			fs << corner_id.str() << Points[b];
		}
		fs.release();
		return true;
	}
}



void draw_board(string fullpath, string path, int i) {
	//point 1 is begins 
	// 1	2 
	// 3	4

	// ()---- -- - ---- -- -()
	print_image(fullpath);

	Mat img = imread(fullpath);
	//cvConvert(img, img, COLOR_BGR2GRAY);
	vector<Point2f> object_Points;
	vector<Point2f> points;
	//get_points(points);

	Point2f point1 = TouchPoints[TouchPoints.size() - 4];
	Point2f point2 = TouchPoints[TouchPoints.size() - 3];
	Point2f point3 = TouchPoints[TouchPoints.size() - 2];
	Point2f point4 = TouchPoints[TouchPoints.size() - 1];

	vector< Point2f> points_together;
	points_together.push_back(point4);	points_together.push_back(point3);	points_together.push_back(point2);	points_together.push_back(point1);



	//Point2f point1 = Size(4,4);
	//Point2f point2 = Size(1,6);
	//Point2f point4 = Size(1, 1);


	cout << "Point1: " << point1 << endl;
	cout << "Point2: " << point2 << endl;
	cout << "Point3: " << point3 << endl;
	cout << "Point4: " << point4 << endl;


	vector<Point2f> d_1_4t;
	vector<Point2f >d_2_3t;



	transform_point(point1, point4, d_1_4t, int(chessboardDim.height - 1)); // 
	transform_point(point2, point3, d_2_3t, int(chessboardDim.height - 1));
	cout << d_1_4t << endl;
	cout << d_2_3t << endl;

	bool patternfound = findCirclesGrid(img, chessboardDim, points_together);

	cout << chessboardDim.height << " chessboardDim.height" << endl;
	cout << chessboardDim.width << " chessboardDim.width" << endl;

	for (int i = 0; i < chessboardDim.height; i++) {
		// rows
		Point2f Cur_A = d_1_4t[i]; Point2f Cur_B = d_2_3t[i];
		cout << "Cur_A " << Cur_A << endl;
		cout << "Cur_B " << Cur_B << endl;
		cout << "Row" << endl;
		vector<Point2f> d_A_B;
		transform_point(Cur_A, Cur_B, d_A_B, chessboardDim.width - 1);
		for (int j = 0; j < chessboardDim.width; j++) {

			object_Points.push_back(d_A_B[j]);
		}
	}



	//for (int i = 0; i <= chessboardDim.height; i++) {
	//	// rows
	//	cout << "Row" << endl;
	//	for (int j = 0; j < chessboardDim.width; j++) {
	//		auto var = point1 + j * x_t;
	//		cout << var << endl;
	//		object_Points.push_back(var);
	//	}
	//	point1 = point1 + y_t;
	//}
	drawChessboardCorners(frame, chessboardDim, object_Points, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
	imshow("Img", frame);
	string name = "data/cam" + to_string(i) + "/" + "boardcorners" + ".xml";
	cout << name << endl;
	cout << i << endl;
	save_parameters2(name, object_Points);
	object_Points.clear();
	TouchPoints.clear();


}






void cameraCalibrationAndLowQualityDetection(vector<Mat> calibrationImages, vector<vector<Point2f>> imagePoints, Mat& cameraMatrix, Mat& distortionCoff, bool release_object, bool useFisheye) {
	//2 dimensional because it's an image
	vector<Mat> rVectors, tVectors;
	vector<Mat> rvecs, tvecs;

	//3 dimensional because it's the world
	vector<vector<Point3f>> objectPoints(1);

	createKnownBoardPositions(objectPoints[0]);
	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms = 0;
	double totalAvgErr;
	vector<float>reprojErrs;


	if (useFisheye) {
		Mat _rvecs, _tvecs;
		rms = fisheye::calibrate(objectPoints, imagePoints, chessboardDim, cameraMatrix, distortionCoff, _rvecs,
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

	bool ok = checkRange(cameraMatrix) && checkRange(distortionCoff);
	vector<double> images_result;
	//objectPoints.clear();
	//objectPoints.resize(imagePoints.size(), newObjPoints);
	vector<vector<Point3f>> Remove_Object; vector<int> Remove_Indexed;
	Mat rVectors_current, tVectors_current;
	for (int i = 0; i < imagePoints.size(); i++) {

		vector<Point3f> currentObject; vector<Point2f>currentImage;
		currentObject = objectPoints[objectPoints.size() - 1];
		currentImage = imagePoints[imagePoints.size() - 1];
		rVectors_current = rVectors[rVectors.size() - 1];
		tVectors_current = tVectors[tVectors.size() - 1];


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
			imagePoints.insert(imagePoints.begin(), currentImage);
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
	cameraMatrix = Mat::zeros(8, 1, CV_64F);
	distortionCoff = Mat::eye(3, 3, CV_64F);
	cout << "Left:" << imagePoints.size() << endl;
	cout << "Recalibration " << endl;
	cout << objectPoints.size() << endl;
	auto newrms = calibrateCamera(objectPoints, imagePoints, chessboardDim, cameraMatrix, distortionCoff, rVectors, tVectors);
	cout << "RMS after recalibration: " << newrms << endl;
	//recalibration after removal


}

// Code gotten from https://www.youtube.com/watch?v=l4gGX-5_5q0 3 video tutorial
/// <summary>
/// runs poseEstimation to draw XYZ axis and a 3D cube on frames captured by the computer camera, if chessboard is detected
/// In total 50 images can be used for calibration. However, a minimum of 25 correct images of the chessboard have to be found, and a max of 25 can be undetected, and might get 
/// </summary>
/// <param name="cameraMatrixValues"></param>
/// <param name="distortionCoefficientsValues"></param>
vector<Mat> runVideoFrames(string path, vector<vector<Point2f>>& allFoundCorners, Mat& cameraMatrixValues, Mat& distortionCoefficientsValues, vector<Mat>& rVectors, vector<Mat>& tVectors, int delay) {
	Mat frame;

	vector<Point2f> foundPoints;
	vector<Mat> imgs;
	vector<Mat> imgsChessboardFound;
	vector<Mat> imgsIgnore;
	vector<Mat> imgsDiscard;
	bool found;

	Mat drawToFrame;
	VideoCapture cap(path);
	if (!cap.isOpened()) { return imgsChessboardFound; }

	//int fps = 20;

	int imgCounter = 0;
	int imgIgnored = 0;
	int treshold = 25;
	int ignore = 0;


	// Randomly select 25 frames
	default_random_engine generator;
	uniform_int_distribution<int>distribution(0, cap.get(CAP_PROP_FRAME_COUNT));

	while (imgCounter < treshold)
	{
		int fid = distribution(generator);
		cap.set(CAP_PROP_POS_FRAMES, fid);
		Mat frame;
		cap >> frame;
		if (frame.empty())
			continue;
		found = findChessboardCorners(frame, chessboardDim, foundPoints, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (found) {
			allFoundCorners.push_back(foundPoints);
			imgsChessboardFound.push_back(frame);
			imgCounter++;
			cout << imgCounter << " found\n";
		}
		else {
			imgIgnored++;
			if (imgIgnored < treshold) {
				cout << imgIgnored << " image corners not found!\n";
				imgsIgnore.push_back(frame);
			}
		}


	}

	cap.release();


	cout << imgsChessboardFound.size() << ": images corners found\n";
	cout << imgsIgnore.size() << "images not found\n";
	cout << "Images corners found:" << imgsChessboardFound.size();
	return imgsChessboardFound;


}

// https://stackoverflow.com/questions/34382832/how-to-read-vectorvectorpoint3f-from-xml-file-with-filestorage could change to this implementation
void save_parameters(string name, Mat camMatrix, Mat distCoeff) {
	FileStorage fs(name, FileStorage::WRITE);
	fs << "CameraMatrix" << camMatrix;
	fs << "DistortionCoeffs" << distCoeff;
	fs.release();
}


inline bool checkIfExists(const std::string& name) {
	ifstream f(name.c_str());
	return f.good();
}


// source: https://stackoverflow.com/questions/16737298/what-is-the-fastest-way-to-transpose-a-matrix-in-c


//BACKGROUND CODE
int computeMedian(vector<int> elements)
{
	nth_element(elements.begin(), elements.begin() + elements.size() / 2, elements.end());

	//sort(elements.begin(),elements.end());
	return elements[elements.size() / 2];
}

Mat compute_median(std::vector<Mat> vec)
{
	// Note: Expects the image to be CV_8UC3
	Mat medianImg(vec[0].rows, vec[0].cols, CV_8UC3, Scalar(0, 0, 0));


	for (int row = 0; row < vec[0].rows; row++)
	{
		for (int col = 0; col < vec[0].cols; col++)
		{
			vector<int> elements_B;
			vector<int> elements_G;
			vector<int> elements_R;

			for (int imgNumber = 0; imgNumber < vec.size(); imgNumber++)
			{
				int B = vec[imgNumber].at<Vec3b>(row, col)[0];
				int G = vec[imgNumber].at<Vec3b>(row, col)[1];
				int R = vec[imgNumber].at<Vec3b>(row, col)[2];


				elements_B.push_back(B);
				elements_G.push_back(G);
				elements_R.push_back(R);
			}

			medianImg.at<Vec3b>(row, col)[0] = computeMedian(elements_B);
			medianImg.at<Vec3b>(row, col)[1] = computeMedian(elements_G);
			medianImg.at<Vec3b>(row, col)[2] = computeMedian(elements_R);
		}
	}
	return medianImg;
}

/// <summary>
/// Creates the background.png by averaging the pixels of 25 frames from background.avi
/// Source: https://learnopencv.com/simple-background-estimation-in-videos-using-opencv-c-python/;
/// </summary>
/// <param name="path"></param>
void get_bg_image(string path) {
	Mat frame;
	vector<Mat> frames;

	string path_video = path + "background.avi";

	VideoCapture cap(path_video);
	if (!cap.isOpened()) { return; }

	// Randomly select 25 frames
	default_random_engine generator;
	uniform_int_distribution<int>distribution(0,
		cap.get(CAP_PROP_FRAME_COUNT));

	for (int i = 0; i < 25; i++)
	{
		int fid = distribution(generator);
		cap.set(CAP_PROP_POS_FRAMES, fid);
		Mat frame;
		cap >> frame;
		if (frame.empty())
			continue;
		frames.push_back(frame);
	}
	// Calculate the median along the time axis
	Mat medianFrame = compute_median(frames);

	// Display median frame
	imwrite(path + "background.png", medianFrame);


}


int main(int argc, char** argv) {

	if (false) {
		int delay = 10;


		for (int i = 1; i <= 4; i++)
		{
			string path1 = "data/cam" + to_string(i) + "/intrinsics.avi";
			cout << path1;
			Mat camMatrix1 = Mat::eye(3, 3, CV_64F);
			Mat distCoeff1 = Mat::zeros(8, 1, CV_64F);
			Mat distCoeff1T = Mat::zeros(1, 5, CV_64F);
			vector<Mat> rVectors1, tVectors1;
			vector<vector<Point2f>> checkerboardImageSpacePoints1;
			vector<Mat> savedImages1;

			savedImages1 = runVideoFrames(path1, checkerboardImageSpacePoints1, camMatrix1, distCoeff1, rVectors1, tVectors1, delay);
			cameraCalibrationAndLowQualityDetection(savedImages1, checkerboardImageSpacePoints1, camMatrix1, distCoeff1, true, false);

			transpose(distCoeff1, distCoeff1T);

			save_parameters("data/cam" + to_string(i) + "/intrinsics.xml", camMatrix1, distCoeff1T);
			get_bg_image("data/cam" + to_string(i) + "/");
		}
	}

	if (false) {
		for (int i = 1; i <= 4; i++) {
			string video = "/checkerboard.avi";
			string xmlFile = "/boardcorners.xml";
			string path = "data/cam" + to_string(i);

			if (!checkIfExists(path + xmlFile)) {
				VideoCapture cap(path);
				draw_board(path + video, path, i);
			}
		}
	}

	VoxelReconstruction::showKeys();
	VoxelReconstruction vr("data" + std::string(PATH_SEP), 4);
	vr.run(argc, argv);

	return EXIT_SUCCESS;
}

