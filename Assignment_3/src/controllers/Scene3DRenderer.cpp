/*
 * Scene3DRenderer.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: coert
 */

#include "Scene3DRenderer.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <stddef.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <sstream>

#include "../utilities/General.h"

using namespace std;
using namespace cv;


namespace nl_uu_science_gmt
{

/**
 * Constructor
 * Scene properties class (mostly called by Glut)
 * camera 1 - frame 1380
 */
Scene3DRenderer::Scene3DRenderer(
		Reconstructor &r, const vector<Camera*> &cs) :
				m_reconstructor(r),
				m_cameras(cs),
				m_num(4),
				m_sphere_radius(1850)
{
	m_width = 640;
	m_height = 480;
	m_quit = false;
	m_paused = false;
	m_rotate = false;
	m_camera_view = true;
	m_show_volume = true;
	m_show_grd_flr = true;
	m_show_cam = true;
	m_show_org = true;
	m_show_arcball = false;
	m_show_info = true;
	m_fullscreen = false;

	// Read the checkerboard properties (XML)
	FileStorage fs;
	fs.open(m_cameras.front()->getDataPath() + ".." + string(PATH_SEP) + General::CBConfigFile, FileStorage::READ);
	if (fs.isOpened())
	{
		fs["CheckerBoardWidth"] >> m_board_size.width;
		fs["CheckerBoardHeight"] >> m_board_size.height;
		fs["CheckerBoardSquareSize"] >> m_square_side_len;
	}
	fs.release();

	m_current_camera = 0;
	m_previous_camera = 0;

	m_number_of_frames = m_cameras.front()->getFramesAmount();
	m_current_frame = 0;
	m_previous_frame = -1;



	const int H = 0;
	const int S = 0;
	const int V = 0;//ADAPTIVE_THRESH_GAUSSIAN_C;
	const int erosionS = 3;//2;
	const int dilationS = 2;//3;
	m_h_threshold = H;
	m_ph_threshold = H;
	m_s_threshold = S;
	m_ps_threshold = S;
	m_v_threshold = V;
	m_pv_threshold = V;
	erosion_size = erosionS;
	dilation_size = dilationS;


	createTrackbar("Frame", VIDEO_WINDOW, &m_current_frame, m_number_of_frames - 2);
	createTrackbar("H", VIDEO_WINDOW, &m_h_threshold, 255);
	createTrackbar("S", VIDEO_WINDOW, &m_s_threshold, 255);
	createTrackbar("V", VIDEO_WINDOW, &m_v_threshold, 255);
	createTrackbar("Erosion", VIDEO_WINDOW, &erosion_size, 21);
	createTrackbar("Dilation", VIDEO_WINDOW, &dilation_size, 21);

	createFloorGrid();
	setTopView();
}

/**
 * Deconstructor
 * Free the memory of the floor_grid pointer vector
 */
Scene3DRenderer::~Scene3DRenderer()
{
	for (size_t f = 0; f < m_floor_grid.size(); ++f)
		for (size_t g = 0; g < m_floor_grid[f].size(); ++g)
			delete m_floor_grid[f][g];
}

/**
 * Process the current frame on each camera
 */
bool Scene3DRenderer::processFrame()
{
	for (size_t c = 0; c < m_cameras.size(); ++c)
	{
		if (m_current_frame == m_previous_frame + 1)
		{
			m_cameras[c]->advanceVideoFrame();
		}
		else if (m_current_frame != m_previous_frame)
		{
			m_cameras[c]->getVideoFrame(m_current_frame);
		}
		assert(m_cameras[c] != NULL);

		processForeground(m_cameras[c]);
	}
	return true;
}

/**
 * Separate the background from the foreground
 * ie.: Create an 8 bit image where only the foreground of the scene is white (255)
 */
void Scene3DRenderer::processForeground(
		Camera* camera)
{
	assert(!camera->getFrame().empty());
	Mat hsv_image;
	vector<Mat> channels;
	Mat blur_img;
	//cvtColor(camera->getFrame(), hsv_image, CV_BGR2HSV);  // from BGR to HSV color space

	// Background subtraction H
	Mat tmp, foreground, background, mask;
	double area;
	vector<vector<Point> > contours;
	vector< Vec4i > hierarchy;
	vector< Vec4i > hierarchy2;
	vector<vector<Point> > contours2;
	vector< Vec4i > hierarchy3;
	vector<vector<Point> > contours3;
	Mat kernel_d = getStructuringElement(MORPH_RECT, Size((2 * dilation_size) + 1, (2 * dilation_size) + 1), Point(dilation_size, dilation_size));
	Mat kernel_e = getStructuringElement(MORPH_RECT, Size((2 * erosion_size) + 1, (2 * erosion_size) + 1), Point(erosion_size, erosion_size));

	/*
	BACKGROUND SUBTRACTION WITH COLOR CHANNELS
	*/
	cvtColor(camera->getFrame(), hsv_image, CV_BGR2HSV);  // from BGR to HSV color space
	GaussianBlur(hsv_image, blur_img, Size(7, 7), 0);
	split(blur_img, channels);  // Split the HSV-channels for further analysis

	absdiff(channels[0], camera->getBgHsvChannels().at(0), tmp);
	threshold(tmp, foreground, m_h_threshold, 255, CV_THRESH_BINARY + THRESH_OTSU);

	// Background subtraction S
	absdiff(channels[1], camera->getBgHsvChannels().at(1), tmp);
	threshold(tmp, background, m_s_threshold, 255, THRESH_BINARY + THRESH_OTSU);
	bitwise_and(foreground, background, foreground);

	// Background subtraction V
	absdiff(channels[2], camera->getBgHsvChannels().at(2), tmp);
	threshold(tmp, background, m_v_threshold, 255, CV_THRESH_BINARY + THRESH_OTSU);
	bitwise_or(foreground, background, foreground);
	
	

	/*
	BACKGROUND SUBTRACTION WITH createBackgroundSubtractorMOG2
	

	cvtColor(camera->getFrame(), hsv_image, CV_BGR2RGB);  // from BGR to HSV color space
	GaussianBlur(hsv_image, blur_img, Size(7, 7), 0);

	camera->getBGSub()->apply(blur_img, mask, 0);
	threshold(mask, foreground, 200, 255, CV_THRESH_BINARY);
	
	//remove outliers*/

	// EROSION, REMOVE SMALL OUTLIERS THEN DILATION
	erode(foreground, foreground, kernel_e);

	findContours(foreground, contours2, hierarchy2, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	for (int i = 0; i < contours2.size(); i = hierarchy2[i][0]) // iterate through each contour.
	{
		if (hierarchy2[i][2] < 0) {
			drawContours(foreground, contours2, (int)i, Scalar(255, 255, 255), 4, 2);

		}
		area = contourArea(contours2[i]);
		if (area > 5000) {
			drawContours(foreground, contours2, (int)i, Scalar(255, 255, 255), 2, 2);
		}
	}
	dilate(foreground, foreground, kernel_d);


	findContours(foreground, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	for (int i = 0; i < contours.size(); i = hierarchy[i][0]) // iterate through each contour.
	{
		area = contourArea(contours[i]);
		if (area < 3000) {
			drawContours(foreground, contours, (int)i, Scalar(255, 255, 255), 2, 2);
		}
	}

	camera->setForegroundImage(foreground);
}

/**
 * Set currently visible camera to the given camera id
 */
void Scene3DRenderer::setCamera(
		int camera)
{
	m_camera_view = true;

	if (m_current_camera != camera)
	{
		m_previous_camera = m_current_camera;
		m_current_camera = camera;
		m_arcball_eye.x = m_cameras[camera]->getCameraPlane()[0].x;
		m_arcball_eye.y = m_cameras[camera]->getCameraPlane()[0].y;
		m_arcball_eye.z = m_cameras[camera]->getCameraPlane()[0].z;
		m_arcball_up.x = 0.0f;
		m_arcball_up.y = 0.0f;
		m_arcball_up.z = 1.0f;
	}
}

/**
 * Set the 3D scene to bird's eye view
 */
void Scene3DRenderer::setTopView()
{
	m_camera_view = false;
	if (m_current_camera != -1)
		m_previous_camera = m_current_camera;
	m_current_camera = -1;

	m_arcball_eye = vec(0.0f, 0.0f, 10000.0f);
	m_arcball_centre = vec(0.0f, 0.0f, 0.0f);
	m_arcball_up = vec(0.0f, 1.0f, 0.0f);
}

/**
 * Create a LUT for the floor grid
 */
void Scene3DRenderer::createFloorGrid()
{
	const int size = m_reconstructor.getSize() / m_num;
	const int z_offset = 3;

	// edge 1
	vector<Point3i*> edge1;
	for (int y = -size * m_num; y <= size * m_num; y += size)
		edge1.push_back(new Point3i(-size * m_num, y, z_offset));

	// edge 2
	vector<Point3i*> edge2;
	for (int x = -size * m_num; x <= size * m_num; x += size)
		edge2.push_back(new Point3i(x, size * m_num, z_offset));

	// edge 3
	vector<Point3i*> edge3;
	for (int y = -size * m_num; y <= size * m_num; y += size)
		edge3.push_back(new Point3i(size * m_num, y, z_offset));

	// edge 4
	vector<Point3i*> edge4;
	for (int x = -size * m_num; x <= size * m_num; x += size)
		edge4.push_back(new Point3i(x, -size * m_num, z_offset));

	m_floor_grid.push_back(edge1);
	m_floor_grid.push_back(edge2);
	m_floor_grid.push_back(edge3);
	m_floor_grid.push_back(edge4);
}

} /* namespace nl_uu_science_gmt */
