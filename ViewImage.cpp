#include "ViewImage.h"


ViewImage::ViewImage(void)
{
}


ViewImage::~ViewImage(void)
{
}

void ViewImage::displayVideo(cv::Mat frameLeft, cv::Mat frameRight)
{

}


void ViewImage::writeOnImages(cv::Mat& src, std::string msg){

	//Paramètre du video writer
	double fontScale = 0.7;
	int thickness = 1;
	int posX = 5;
	int posY = 50;

	std::stringstream ss;
	ss << msg;
	writeOnImages(src, ss.str(), posX, posY, thickness, fontScale, 250, 250, 0);

}


void ViewImage::writeOnImages(cv::Mat& src, std::string msg, int posX, int posY){

	//Paramètre du video writer
	double fontScale = 0.7;
	int thickness = 2;
	
	std::stringstream ss;
	ss << msg;
	writeOnImages(src, ss.str(), posX, posY, thickness, fontScale, 250, 250, 0);

}


void ViewImage::writeOnImages(cv::Mat& src, std::string msg, int posX, int posY, int thickness, double fontScale){

	//Paramètre du video writer
	std::stringstream ss;
	ss << msg;
	writeOnImages(src, ss.str(), posX, posY, thickness, fontScale, 250, 250, 0);

}


void ViewImage::writeOnImages(cv::Mat& src, std::string msg, int posX, int posY, int thickness, double fontScale, int RColor, int GColor, int BColor){

	//Paramètre du video writer
	int fontFace = cv::FONT_HERSHEY_SIMPLEX;

	int y0 = posY;
	cv::Size textSize = cv::getTextSize(msg, fontFace, fontScale, thickness, 0);
	int dy = textSize.height+5;
	int i = 0;
	std::vector<std::string> strs;
	boost::split(strs, msg, boost::is_any_of("\n"));
	for (std::vector<std::string>::iterator sMsg = strs.begin();  sMsg != strs.end(); sMsg++, i++ )
	{
		posY = y0 + i*dy;
		cv::Point textOrg(posX, posY);
		cv::putText(src, *sMsg, textOrg, fontFace, fontScale,  CV_RGB(RColor, GColor, BColor), thickness);
	}	
}


void ViewImage::combineImageH(cv::Mat& imgToMerge, cv::Mat frame_left, cv::Mat frame_right)
{
	// Get dimension of final image
	int rows = cv::max(frame_left.rows, frame_right.rows);
	int cols = frame_left.cols + frame_right.cols;
	if (frame_left.channels() == 1)
	{
		cv::cvtColor(frame_left, frame_left, CV_GRAY2RGB);
	}

	if (frame_right.channels() == 1)
	{
		cv::cvtColor(frame_right, frame_right, CV_GRAY2RGB);
	}

	// Create a black image
	cv::Mat3b combine(rows, cols, cv::Vec3b(0,0,0));

	// Copy images in correct position
	frame_left.copyTo(combine(cv::Rect(0, 0, frame_left.cols, frame_left.rows)));
	frame_right.copyTo(combine(cv::Rect(frame_left.cols, 0, frame_right.cols, frame_right.rows)));
	imgToMerge = combine;
}

void ViewImage::combineImageV(cv::Mat& imgToMerge, cv::Mat frame_up, cv::Mat frame_down)
{
	// Get dimension of final image
	int rows = frame_up.rows + frame_down.rows;
	int cols = cv::max(frame_up.cols, frame_down.cols);
	if (frame_up.channels() == 1)
	{
		cv::cvtColor(frame_up, frame_up, CV_GRAY2RGB);
	}

	if (frame_down.channels() == 1)
	{
		cv::cvtColor(frame_down, frame_down, CV_GRAY2RGB);
	}

	// Create a black image
	cv::Mat3b combine(rows, cols, cv::Vec3b(0,0,0));

	// Copy images in correct position
	frame_up.copyTo(combine(cv::Rect(0, 0, frame_up.cols, frame_up.rows)));

	frame_down.copyTo(combine(cv::Rect(0, frame_up.rows, frame_down.cols, frame_down.rows)));
	//cv::hconcat(mat, cols, mat);
	imgToMerge = combine;
}


void ViewImage::combineImage(cv::Mat& imgToMerge, cv::Mat frame_left1, cv::Mat frame_right1, cv::Mat frame_left2, cv::Mat frame_right2)
{
	//printf("Starting combine1 start\n");
	// Get dimension of final image
	//int rows = cv::max(frame_left1.rows + frame_right1.rows, frame_left2.rows + frame_right2.rows);
	//int cols = cv::max(frame_left1.cols + frame_left2.cols, frame_right1.cols + frame_right2.cols);
	cv::Mat frame_up;
	cv::Mat frame_down;
	combineImageH(frame_up, frame_left1, frame_right1);
	combineImageH(frame_down, frame_left2, frame_right2);

	combineImageV(imgToMerge, frame_up, frame_down);

	// Create a black image
	//cv::Mat3b combine(rows, cols, cv::Vec3b(0,0,0));
	
	//printf("Starting combine start\n");
	// Copy images in correct position
	//frame_left1.copyTo(combine(cv::Rect(0, 0, frame_left1.cols, frame_left1.rows)));
	//frame_right1.copyTo(combine(cv::Rect(0, frame_left1.cols,  frame_right1.rows, frame_right1.cols + frame_left1.cols)));
	//frame_left2.copyTo(combine(cv::Rect(frame_left1.rows, 0, frame_left1.rows + frame_left2.rows, frame_left2.cols)));
	//frame_right2.copyTo(combine(cv::Rect(frame_left1.rows, frame_left1.cols, frame_right2.rows + frame_left1.rows, frame_left1.cols + frame_right2.cols)));
	//printf("Starting resize\n");
	cv::resize(imgToMerge, imgToMerge, frame_left1.size());
}

/**
 * Rotate an image (source: http://opencv-code.com/quick-tips/how-to-rotate-image-in-opencv/)
 */
void ViewImage::rotate(cv::Mat& src, double angle, cv::Mat& dst)
{
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(len/2., len/2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

    cv::warpAffine(src, dst, r, cv::Size(len, len));
}
