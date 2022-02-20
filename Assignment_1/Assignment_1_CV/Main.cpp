//#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
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


void createKnownBoardPositions(vector<Point3f>& corners) {

	for (int i = 0; i < chessboardDim.height; i++)
	{
		for (int j = 0; j < chessboardDim.width; j++)
		{
			corners.push_back(Point3f(j* squareDim, i* squareDim, 0.0f));
		}
	}
}

static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoefcout,
	vector<float>& perViewErrors, vector<double>& images_result)
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

	return Mean_Projection_error;
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
		if (release_object)
		cout << "Recalibration " << endl;
		rms = calibrateCamera(objectPoints, imagePoints, chessboardDim, cameraMatrix, distortionCoff, rVectors, tVectors);
	}
	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distortionCoff);
	vector<double> images_result;

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



//TODO: change this to save all cam params to XML file
bool saveCameraCalibration(string name, Mat parameter) {
	
	ofstream outStream(name);
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


vector<Mat> runImages(vector<vector<Point2f>>& allFoundCorners) {
	string path = "Camera Roll/*.jpg";
	vector<String> images;
	glob(path, images);
	
	Mat frame;

	vector<Point2f> foundPoints;
	vector<Mat> imgs;
	vector<Mat> imgsChessboardFound;
	vector<Mat> imgsIgnore;
	vector<Mat> imgsDiscard;
	bool found;

	for (int i = 0; i < images.size(); i++)
	{
		//cout << i;
		frame = imread(images[i]);

		found = findChessboardCorners(frame, chessboardDim, foundPoints, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (found) {
			allFoundCorners.push_back(foundPoints);
			imgsChessboardFound.push_back(frame);
		}
		else {
			cout << i << " image corners not found!\n";
			imgsIgnore.push_back(frame);
		}
	}

	if (!imgsIgnore.empty()) {
		for (vector<Mat>::iterator iter = imgsIgnore.begin(); iter != imgsIgnore.end(); iter++)
		{
			//solution gotten from https://stackoverflow.com/questions/24341114/simple-illumination-correction-in-images-opencv-c
			
			Mat lab_image;

			cvtColor(*iter, lab_image, COLOR_BGR2Lab);
			vector<Mat> lab_planes(3);
			split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

			// apply the CLAHE algorithm to the L channel
			Ptr<cv::CLAHE> clahe = createCLAHE();
			clahe->setClipLimit(4);
			Mat dst;
			clahe->apply(lab_planes[0], dst);

			// Merge the the color planes back into an Lab image
			dst.copyTo(lab_planes[0]);
			merge(lab_planes, lab_image);

			// convert back to RGB
			Mat image_clahe;
			cvtColor(lab_image, image_clahe, COLOR_Lab2BGR);

			found = findChessboardCorners(image_clahe, chessboardDim, foundPoints, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
			if (found) {
				imgsChessboardFound.push_back(image_clahe);
				allFoundCorners.push_back(foundPoints);
			}
			else {
				cout << " image corners not found after -clahe-!\n";

				imgsDiscard.push_back(image_clahe);
			}

		}
	}

	cout << imgsChessboardFound.size();

	return imgsChessboardFound;
	
}


bool calibrateCameraAndSaveParams(vector<Mat> calibrationImages, vector<vector<Point2f>> checkerboardImageSpacePoints, Mat& cameraMatrix, Mat& distortionCoff) {
	cameraCalibrationAndLowQualityDetection(calibrationImages, checkerboardImageSpacePoints, cameraMatrix, distortionCoff, true, false);

	saveCameraCalibration("cameraMatrix.txt", cameraMatrix);
	saveCameraCalibration("distortionCoefficients.txt", distortionCoff);
	return true;
}


void poseEstimation(Mat img, vector<Point2f> corners, bool foundCorner, Mat cameraMatrixValues, Mat distortionCoefficientsValues) {
	
	Mat imgCorners;
	vector<Point3f> objectPoints;
	Mat imgGray;	
	vector<Point2f> imagePoints, imagePointsCube;
	Mat rVectors, tVectors, rotMtx;
	cvtColor(img, imgGray, COLOR_BGR2GRAY);

	TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001);
	
	drawChessboardCorners(img, chessboardDim, corners, foundCorner);
	//imshow("Image with corners", imgCorners);
	cornerSubPix(imgGray, corners, Size(11, 11), Size(-1, -1), criteria);

	createKnownBoardPositions(objectPoints);

	solvePnP(objectPoints, corners, cameraMatrixValues, distortionCoefficientsValues, rVectors, tVectors);

	cout << "rVectors\n";
	cout << rVectors;



	vector<Point3f> axis;
	vector<Point3f> cube;

	axis.push_back(Point3f(3.0 * squareDim, 0.0, 0.0));
	axis.push_back(Point3f(0.0, 3.0 * squareDim, 0.0));
	axis.push_back(Point3f(0.0, 0.0, -3.0 * squareDim));

	projectPoints(axis, rVectors, tVectors, cameraMatrixValues, distortionCoefficientsValues, imagePoints);

	arrowedLine(img, corners[0], imagePoints[0], Scalar(255, 0, 0), 2, LINE_8);
	arrowedLine(img, corners[0], imagePoints[1], Scalar(0, 255, 0), 2, LINE_8);
	arrowedLine(img, corners[0], imagePoints[2], Scalar(0, 0, 255), 2, LINE_8);


	cube.push_back(Point3f(0.0, 0.0, 0.0));
	cube.push_back(Point3f(0.0, 2.0 * squareDim, 0.0));
	cube.push_back(Point3f(2.0 * squareDim, 2.0 * squareDim, 0.0));
	cube.push_back(Point3f(2.0 * squareDim, 0.0, 0.0));
	cube.push_back(Point3f(0.0, 0.0, -2.0 * squareDim));
	cube.push_back(Point3f(0.0, 2.0 * squareDim, -2.0 * squareDim));
	cube.push_back(Point3f(2.0 * squareDim, 2.0 * squareDim, -2.0 * squareDim));
	cube.push_back(Point3f(2.0 * squareDim, 0.0, -2.0 * squareDim));

	projectPoints(cube, rVectors, tVectors, cameraMatrixValues, distortionCoefficientsValues, imagePointsCube);

	cout << imagePointsCube.size() << "\n";

	//BASE
	for (int i = 0, j = 1; i < 3 && j < 4; i++, j++)
	{
		line(img, imagePointsCube[i], imagePointsCube[j], Scalar(0, 255, 255), 2, LINE_8);
		if (j == 3) {
			line(img, imagePointsCube[0], imagePointsCube[j], Scalar(0, 255, 255), 2, LINE_8);
		}
	}


	//PILLARS
	for (int i = 0, j = 4; i < 4 && j < imagePointsCube.size(); i++, j++)
	{
		line(img, imagePointsCube[i], imagePointsCube[j], Scalar(0, 255, 255), 2, LINE_8);

	}

	//TOP
	for (int i = 4, j = 5; i < imagePointsCube.size() - 1 && j < imagePointsCube.size(); i++, j++)
	{
		line(img, imagePointsCube[i], imagePointsCube[j], Scalar(0, 255, 255), 2, LINE_8);
		if (j == imagePointsCube.size() - 1) {
			line(img, imagePointsCube[4], imagePointsCube[j], Scalar(0, 255, 255), 2, LINE_8);

		}
	}

	imshow("Output", img);

}



void runCamera(Mat cameraMatrixValues, Mat distortionCoefficientsValues) {
	Mat frame;
	Mat drawToFrame;

	VideoCapture vid(0);
	if (!vid.isOpened()) { return; }

	int fps = 20;

	namedWindow("Webcam", WINDOW_AUTOSIZE);

	while (true) {
		if (!vid.read(frame))
			break;

		vector<Point2f> foundPoints;
		bool found = false;

		found = findChessboardCorners(frame, chessboardDim, foundPoints, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (found)
			poseEstimation(frame, foundPoints, found, cameraMatrixValues, distortionCoefficientsValues);
		else
			imshow("webcam", frame);

		char character = waitKey(1000 / fps);

	}
}



void main(int argv, char** argc) {

	Mat camMatrix = Mat::eye(3,3,CV_64F);
	Mat distCoeff = Mat::zeros(8, 1, CV_64F);
	vector<vector<Point2f>> checkerboardImageSpacePoints;

	vector<Mat> savedImages;


	//  OFFLINE  //

	//savedImages = runImages(checkerboardImageSpacePoints);

	//uncomment to calibrate on given images
	//calibrateCameraAndSaveParams(savedImages, checkerboardImageSpacePoints, camMatrix, distCoeff);

	cout << "Calibrated! ";

	//  ONLINE  //
	// 
	//TODO: change this to read all cam params from XML file

	fstream cameraMatrix;
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

	//uncomment to run on camera
	runCamera(cameraMatrixValues, distortionCoefficientsValues);


	Mat img;
	savedImages[20].copyTo(img);

	vector<Point2f> corners;
	vector<Vec2f> corners2;

	bool foundCorner;


	foundCorner = findChessboardCorners(img, chessboardDim, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);


	if (foundCorner) {
		poseEstimation(img, corners, foundCorner, cameraMatrixValues, distortionCoefficientsValues);
	}

	waitKey(0);
}