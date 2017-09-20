/*
 * MyCalibration.h
 *
 *  Created on: 26 june. 2017
 *  Author: Arthur Hamelin
 */

#ifndef MYCALIBRATION_H_
#define MYCALIBRATION_H_


// Opencv Header
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Local
#include "StereoCalibration.h"
#include "CameraView.h"

// System
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctime>
#include <iostream>
#include <map>

using namespace std;

class MyCalibration
{
private:
    StereoCalibration* stereoCalibration;
    bool stopThread, startThread;

public:
    // Constructor
    MyCalibration();

    // Methods
    int createCalibration(vector<int> deviceIDs, map<int, CameraView*> cameraViewMap);
    int createVRCalibration(vector<int> deviceIDs, map<int, CameraView*> cameraViewMap);
    void chessboardCaptureThread();
    int chessboardDetection(cv::Mat& frame1, cv::Mat& frame2);
    int calibrateCameras();

};

#endif /* MYCALIBRATION_H_ */
