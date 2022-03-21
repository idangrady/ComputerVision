/*
 * Reconstructor.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: coert
 */

#include "Reconstructor.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types_c.h>
#include <cassert>
#include <iostream>

#include "../utilities/General.h"

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

	/**
	 * Constructor
	 * Voxel reconstruction class
	 *
	 * " I think the parallelization can be better, and the indexing can be better."
	 *
	 */

	void EuclidienDis(Point3f& A, Point3f& B, float& d) {
		Point3f diff = (A - B);
		d = sqrt(diff.x * diff.x + diff.y * diff.y);
	}

	void fill_mat(Mat& idVoxel, Mat& disVoxel, Point center) {
		// check for the center.. 

		vector<int> additive = { -1,0, 1 };

		for (int i = 0; i < 2; i++) {
			int x = center.x + additive[i];
			int y = center.y;
		}

		for (int i = 0; i <= 2; i++) {
			int x = center.x;
			int y = center.y + additive[i];
		}

	}

	Reconstructor::Reconstructor(
		const vector<Camera*>& cs) :
		m_cameras(cs),
		m_height(2048),//2048
		m_step(32)//32
	{
		for (size_t c = 0; c < m_cameras.size(); ++c)
		{
			if (m_plane_size.area() > 0)
				assert(m_plane_size.width == m_cameras[c]->getSize().width && m_plane_size.height == m_cameras[c]->getSize().height);
			else
				m_plane_size = m_cameras[c]->getSize();
		}

		const size_t edge = 2 * m_height;
		m_voxels_amount = (edge / m_step) * (edge / m_step) * (m_height / m_step);

		initialize();
	}

	/**
	 * Deconstructor
	 * Free the memory of the pointer vectors
	 */
	Reconstructor::~Reconstructor()
	{
		for (size_t c = 0; c < m_corners.size(); ++c)
			delete m_corners.at(c);
		for (size_t v = 0; v < m_voxels.size(); ++v)
			delete m_voxels.at(v);
	}

	//float computeMedian(vector<int> elements)
	//{
	//	nth_element(elements.begin(), elements.begin() + elements.size() / 2, elements.end());

	//	//sort(elements.begin(),elements.end());
	//	//return float((elements[elements.size() / 2]) / 255);
	//	return (elements[elements.size() / 2]);
	//}

	/**
	 * Create some Look Up Tables
	 * 	- LUT for the scene's box corners
	 * 	- LUT with a map of the entire voxelspace: point-on-cam to voxels
	 * 	- LUT with a map of the entire voxelspace: voxel to cam points-on-cam
	 */
	void Reconstructor::initialize()
	{
		// Cube dimensions from [(-m_height, m_height), (-m_height, m_height), (0, m_height)]
		const int xL = -(m_height / 2);
		const int xR = m_height + (m_height / 2);
		const int yL = -m_height;
		const int yR = m_height;
		const int zL = 0;
		const int zR = m_height;
		const int plane_y = (yR - yL) / m_step;
		const int plane_x = (xR - xL) / m_step;
		const int plane = plane_y * plane_x;

		// Save the 8 volume corners
		// bottom
		m_corners.push_back(new Point3f((float)xL, (float)yL, (float)zL));
		m_corners.push_back(new Point3f((float)xL, (float)yR, (float)zL));
		m_corners.push_back(new Point3f((float)xR, (float)yR, (float)zL));
		m_corners.push_back(new Point3f((float)xR, (float)yL, (float)zL));

		// top
		m_corners.push_back(new Point3f((float)xL, (float)yL, (float)zR));
		m_corners.push_back(new Point3f((float)xL, (float)yR, (float)zR));
		m_corners.push_back(new Point3f((float)xR, (float)yR, (float)zR));
		m_corners.push_back(new Point3f((float)xR, (float)yL, (float)zR));

		// Acquire some memory for efficiency
		cout << "Initializing " << m_voxels_amount << " voxels ";
		m_voxels.resize(m_voxels_amount);

		int z;
		int pdone = 0;
		int voxelidx = 0;
#pragma omp parallel for schedule(static) private(z) shared(pdone)
		for (z = zL; z < zR; z += m_step)
		{
			const int zp = (z - zL) / m_step;
			int done = cvRound((zp * plane / (double)m_voxels_amount) * 100.0);

#pragma omp critical
			if (done > pdone)
			{
				pdone = done;
				cout << done << "%..." << flush;
			}

			int y, x;
			for (y = yL; y < yR; y += m_step)
			{
				const int yp = (y - yL) / m_step;

				for (x = xL; x < xR; x += m_step)
				{
					const int xp = (x - xL) / m_step;

					// Create all voxels
					Voxel* voxel = new Voxel;
					voxel->x = x;
					voxel->y = y;
					voxel->z = z;
					voxel->label = -1;
					voxel->camera_projection = vector<Point>(m_cameras.size());
					voxel->valid_camera_projection = vector<int>(m_cameras.size(), 0);
					voxel->distanceCamera = std::vector<float>(m_cameras.size());
					const int p = zp * plane + yp * plane_x + xp;  // The voxel's index
					voxel->id = voxelidx;
					voxelidx = voxelidx + 1;

					for (size_t c = 0; c < m_cameras.size(); ++c)
					{
						Point point = m_cameras[c]->projectOnView(Point3f((float)x, (float)y, (float)z));

						// Save the pixel coordinates 'point' of the voxel projection on camera 'c'
						voxel->camera_projection[(int)c] = point;

						// If it's within the camera's FoV, flag the projection
						if (point.x >= 0 && point.x < m_plane_size.width && point.y >= 0 && point.y < m_plane_size.height)
							voxel->valid_camera_projection[(int)c] = 1;

						float d = 0;
						Point3f Voxel_loc = Point3f((float)x, (float)y, (float)z);
						Point3f cam_loc = m_cameras[c]->getCameraLocation();
						EuclidienDis(cam_loc, Voxel_loc, d);
						voxel->distanceCamera[c] = d;
					}

					//Writing voxel 'p' is not critical as it's unique (thread safe)

					m_voxels[p] = voxel;
				}
			}
		}

		cout << "done!" << endl;
	}


	int getIndex(vector<vector<cv::Point>> v, vector<cv::Point> K)
	{
		auto it = find(v.begin(), v.end(), K);

		// If element was found
		if (it != v.end())
		{

			// calculating the index
			// of K
			int index = it - v.begin();
			return index;
		}
		else {
			// If the element is not
			// present in the vector
			return -1;
		}
	}

	/**
	 * Count the amount of camera's each voxel in the space appears on,
	 * if that amount equals the amount of cameras, add that voxel to the
	 * visible_voxels vector
	 */
	void Reconstructor::update()
	{
		m_visible_voxels.clear();

		shortest_voxel_Id.clear();
		Shortest_Voxel_distance.clear();
		std::vector<Voxel*> visible_voxels;
		int voxel_id = 0;
		vector<vector<cv::Point>> projection;

		int v;
#pragma omp parallel for schedule(static) private(v) shared(visible_voxels)


		// get the size of the frame
		for (int i = 0; i < (int)m_cameras.size(); i++) {
			Mat curr_frame = m_cameras[i]->getFrame();
			Mat shortest_v_id = Mat(curr_frame.rows, curr_frame.cols, CV_32FC3, Scalar(0, 0, 0));
			Mat shortest_v_dis = Mat(curr_frame.rows, curr_frame.cols, CV_32FC3, Scalar(0, 0, 0));
			shortest_voxel_Id.push_back(shortest_v_id);
			Shortest_Voxel_distance.push_back(shortest_v_dis);
		}

		for (v = 0; v < (int)m_voxels_amount; ++v)
		{
			int camera_counter = 0;
			Voxel* voxel = m_voxels[v];


			for (size_t c = 0; c < m_cameras.size(); ++c)
			{
				if (voxel->valid_camera_projection[c])
				{
					const Point point = voxel->camera_projection[c];

					//If there's a white pixel on the foreground image at the projection point, add the camera
					if (m_cameras[c]->getForegroundImage().at<uchar>(point) == 255) ++camera_counter;
				}
			}

			// If the voxel is present on all cameras
			if (camera_counter == m_cameras.size())
			{
#pragma omp critical //push_back is critical

				// Note: The index of the voxel would stay the same
				// More over, the distance would stay the same
				// To check , if the current pixel projection occluded. 

				// steps: check if 

				// get projection
				for (size_t c = 0; c < m_cameras.size(); ++c)
				{
					if (voxel->valid_camera_projection[c])
					{
						const Point point = voxel->camera_projection[c];
						//distance
						const float cur_dis = voxel->distanceCamera[c];
						// is larget shorter than the current one 

						Mat curr_frame_mat = m_cameras[c]->getFrame();
						if (point.x < curr_frame_mat.rows && point.y < curr_frame_mat.cols) {
							visible_voxels.push_back(voxel);

							if (shortest_voxel_Id[c].at<float>(point.x, point.y) != 0 && cur_dis < Shortest_Voxel_distance[c].at<float>(point.x, point.y))
							{
								// make the update
								Shortest_Voxel_distance[c].at<float>(point.x, point.y) = cur_dis;
								shortest_voxel_Id[c].at<float>(point.x, point.y) = voxel->id;
								//voxel->id = visible_voxels.size() - 1;
							}

							else if (Shortest_Voxel_distance[c].at<float>(point.x, point.y) == 0 && shortest_voxel_Id[c].at<float>(point.x, point.y) == 0) {
								// make the update
								Shortest_Voxel_distance[c].at<float>(point.x, point.y) = cur_dis;
								shortest_voxel_Id[c].at<float>(point.x, point.y) = voxel->id;
								//voxel->id = visible_voxels.size() - 1;
							}
						}

					}
				}

			}
		}


		std::vector<Voxel*> visible_voxelsNotOccluded;

		for (int i = 0; i < visible_voxels.size(); i++) {
			//if (checkOcclusion(shortest_voxel_Id, Shortest_Voxel_distance, visible_voxels[i]->id, visible_voxels[i]->distanceCamera, visible_voxels[i]->cornes_vamera_projection)) {
			for (int c = 0; c < 4; c++) {
				Point project = visible_voxels[i]->camera_projection[c];
				if (project.x < shortest_voxel_Id[c].rows && project.y < shortest_voxel_Id[c].cols) {
					if (shortest_voxel_Id[c].at<float>(project.x, project.y) == visible_voxels[i]->id) {
						visible_voxelsNotOccluded.push_back(visible_voxels[i]);
						break;
					}
				}

		}
		}
			m_visible_voxels.insert(m_visible_voxels.end(), visible_voxelsNotOccluded.begin(), visible_voxelsNotOccluded.end());

		
	}
}
/* namespace nl_uu_science_gmt */