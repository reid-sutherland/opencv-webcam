#ifndef VIEWIMAGE_H
#define VIEWIMAGE_H
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include "Util.h"


#pragma once
class ViewImage
{
public:
	ViewImage(void);
	~ViewImage(void);
	void displayVideo(cv::Mat, cv::Mat);
	void writeOnImages(cv::Mat& src, std::string msg);
	void writeOnImages(cv::Mat& src, std::string msg, int posX, int posY);
	void writeOnImages(cv::Mat& src, std::string msg, int posX, int posY, int thickness, double fontScale);
	void writeOnImages(cv::Mat& src, std::string msg, int posX, int posY, int thickness, double fontScale, int RColor, int GColor, int BColor);
	void combineImageH(cv::Mat&, cv::Mat, cv::Mat);
	void combineImageV(cv::Mat&, cv::Mat, cv::Mat);
	void combineImage(cv::Mat&, cv::Mat, cv::Mat, cv::Mat, cv::Mat);
	void rotate(cv::Mat& src, double angle, cv::Mat& dst);
};

#endif
