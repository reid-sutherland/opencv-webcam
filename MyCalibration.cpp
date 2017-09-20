/*
 * MyCalibration.cpp
 *
 *  Created on: 26 june. 2017
 *  Author: Arthur Hamelin
 */

#include "MyCalibration.h"

#include <iostream>
#include <pthread.h>

cv::Mat res_left;
cv::Mat res_right;
cv::Mat frame1;
cv::Mat frame2;

// Constructor
MyCalibration::MyCalibration()
    : stereoCalibration(StereoCalibration::instance())
{

}

// This method use StereoCalibration instance to calibrate cameras (made for 2 cameras)
int MyCalibration::createCalibration(vector<int> deviceIDs, map<int, CameraView*> cvMap) {
    cout << "\n***Starting chessboard detection now...***" << endl;
    cout << "Press c to capture a frame. Note: at least 5 frames are required to calibrate cameras." << endl << endl;

    stereoCalibration->reset();

    while (true) {
        cvMap[deviceIDs[0]]->getFrame(frame1);
        cvMap[deviceIDs[1]]->getFrame(frame2);
        if( frame1.empty() || frame2.empty())
        {
            cout << " --(!) No captured frame -- Break!" << endl;
        }
        else {
            // Apply the classifier to the frame
            chessboardDetection(frame1, frame2);
            imshow( "Left Camera", res_left );
            imshow( "Right Camera", res_right );
        }

        char c = (char) cv::waitKey(200);
        if ((int) c == 27 ) { break; } // escape
        if (c == 'c' || c == 'C') {   // c or C
            stereoCalibration->stereoChessDetection(frame1, frame2, res_left, res_right);
        }
    }

    destroyAllWindows();

    cout << "***Chessboard detection done.***" << endl;

    //calibrateCameras();
    if (stereoCalibration->stereoCalib(true, false) == -1) {
        cout << "***Calibration FAILED.***\n" << endl;
        return -1;
    }
    else {
        cout << "***Calibration done.***\n" << endl;
        return 0;
    }
}

// This method is used for calibrating a VR camera, and includes automatic frame capturing
int MyCalibration::createVRCalibration(vector<int> deviceIDs, map<int, CameraView*> cvMap) {
    cout << "\n***Starting chessboard detection now...***" << endl;
    cout << "Press any key to start automatically capturing frames every second." << endl << endl;

    stereoCalibration->reset();
    stopThread = false;
    startThread = false;

    thread *chessThread = new thread(&MyCalibration::chessboardCaptureThread, this);

    while (true) {
        cvMap[deviceIDs[0]]->getFrame(frame1);
        cvMap[deviceIDs[1]]->getFrame(frame2);
        if( frame1.empty() || frame2.empty())
        {
            cout << " --(!) No captured frame -- Continue!" << endl;
            continue;
        }

        // Apply the classifier to the frame
        chessboardDetection(frame1, frame2);
        imshow( "Left Camera", res_left );
        imshow( "Right Camera", res_right );

        int c = waitKey(30);
        if (c > -1 && !startThread) {   // Any key
            startThread = true;
        }
        else if (c == 27) {    // Escape
            stopThread = true;
            chessThread->join();
            break;
        }
    }

    destroyAllWindows();

    cout << "***Chessboard detection done.***" << endl;

    //calibrateCameras();
    if (stereoCalibration->stereoCalib(true, true) == -1) {
        cout << "***Calibration FAILED.***\n" << endl;
        return -1;
    }
    else {
        cout << "***Calibration done.***\n" << endl;
        return 0;
    }
}

void MyCalibration::chessboardCaptureThread() {
    // Wait for threadStart to be set
    while (!startThread) {
        this_thread::yield();
    }
    // Wait for threadStop to be set
    while (!stopThread) {
        this_thread::sleep_for(chrono::seconds(1));
        stereoCalibration->stereoChessDetection(frame1, frame2, res_left, res_right);
    }
}

// This method show on screen the chessboard detection, so people can capture specific frames for the calibration.
int MyCalibration::chessboardDetection(cv::Mat& frame1, cv::Mat& frame2){

    cv::Mat gray_left;
    cv::Mat gray_right;
    bool found_left = false; //value noting if checkerboard found in image
    bool found_right = false; //value noting if checkerboard found in image
    cv::Size board_sz = cv::Size(9,6);
    std::vector<cv::Point2f> corners_left;
    std::vector<cv::Point2f> corners_right;

    // Copy of the original frame if nothing is detected
    res_left = frame1.clone();
    res_right = frame2.clone();

    cv::cvtColor(frame1, gray_left, CV_BGR2GRAY);
    cv::cvtColor(frame2, gray_right, CV_BGR2GRAY);
    found_left = cv::findChessboardCorners(gray_left, board_sz, corners_left,
                                           cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
                                           //CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    found_right = cv::findChessboardCorners(gray_right, board_sz, corners_right,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found_left && found_right)//Sub pixel optimization for corner locations.
    {
        //! adjusts the corner locations with sub-pixel accuracy to maximize the certain cornerness criteria
        cv::cornerSubPix(gray_left, corners_left, cv::Size(11,11), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));
        cv::cornerSubPix(gray_right, corners_right, cv::Size(11,11), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));

        //! draw checkerboard corners and lines on RGB images.
        cv::drawChessboardCorners(res_left, board_sz, corners_left, found_left);
        cv::drawChessboardCorners(res_right, board_sz, corners_right, found_right);
    }

    return 0;
}

// This method just call the calibration method from the stereocalibration instance.
int MyCalibration::calibrateCameras() {
    if (stereoCalibration->stereoCalib(true) == -1) {     //true - always save result
        std::cout << "***Calibration FAILED.***\n" << std::endl;
        return -1;
    }
    else{
        std::cout << "***Calibration done.***\n" << std::endl;
        return 0;
    }
}
