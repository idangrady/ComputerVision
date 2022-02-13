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

struct Image
{
	uint8_t* data = NULL; // pichture data
	size_t size= 0;
	int w;
	int h;
	int channels;

	Image(const char* filename);
	Image(int w, int h, int channels);
	Image(const Image& img);
	~Image();

	bool read(const char* filename);

	
};