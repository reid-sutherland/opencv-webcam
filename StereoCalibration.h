/*
 * StereoCalibration.h
 *
 *  Created on: 19 sept. 2015
 *      Author: erman
 */

#ifndef STEREOCALIBRATION_H_
#define STEREOCALIBRATION_H_

// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// System
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctime>
#include <iostream>

// Local
#include "FilenameManagement.h"

class StereoCalibration {

private:

    //! Corners detected
    std::vector<std::vector<cv::Point3f> > objectPoints;
    // ARRAY AND VECTOR STORAGE:
    std::vector<std::vector<cv::Point2f> > imagePoints_left;// Corners Point Left
    std::vector<std::vector<cv::Point2f> > imagePoints_right;// Corners Point right

    //! Number of images
    int nimages;

    //! Intrinsics parameters
    cv::Mat CM1;// Camera matrix 1
    cv::Mat CM2;// Camera matrix 2
    cv::Mat D1; // Distortion coefficient 1
    cv::Mat D2; // Distortion coefficient 2

    //! Extrinsics parameters
    cv::Mat R; //Rotation matrix
    cv::Mat T; //Translation vector

    cv::Mat R1; // 3-by-3 row-aligned rectification rotations for the left and
    cv::Mat R2; // right image planes as derived in the preceding equations.
    cv::Mat P1; // 3-by-4 left and right projection equations Pl and P2.
    cv::Mat P2;
    cv::Mat Q; //Reprojection matrix

    //! Fundamental matrix
    cv::Mat E; //Essential matrix
    cv::Mat F; //Fundamental matrix

    //! IF BY CALIBRATED (BOUGUET'S METHOD) instead of  HARTLEY'S METHOD
    bool useCalibrated;

    cv::Rect validRoi[2];

    //! Chessboard Information
    int min_poses; // Number of images needed for calibration
    int board_w;// Chessbord weight
    int board_h;// Chessboad Height
    cv::Size framesize;// Size of the chessboard images used to perform the calibration.

    std::string calib_filename;// Calibration fileName Parameters

    //! Set if calibration parameter is loaded
    bool calib_param_loaded;

    //! Calibration error
    double calib_error;

    //! Singleton implementation
    static StereoCalibration m_instance;
    int instantiated;
    StereoCalibration& operator=(const StereoCalibration&){}

    //! get list of filename in a file on disk.
    bool readStringList( const std::string& filename, std::vector<std::string>& fileNameList );

    //! Constructor
    StereoCalibration();

public:

    StereoCalibration (const StereoCalibration&){}
    virtual ~StereoCalibration();

    static StereoCalibration& Instance();

    //! Perform calibration
    int stereoCalib(bool saveResult=false);

    //! function to save calibration parameters in the calib_filename
    bool saveCalib();

    //! reload calibration parameters
    bool loadCalib();

    //! To change calibration path name
    void setCalibFilename(std::string calib_filename);

    //! Dectect the chessboard on the two first frame, and draw the result on the two other.
    int stereoChessDetection(cv::Mat& frame_left, cv::Mat& frame_right, cv::Mat& gray_left, cv::Mat& gray_right);

    //! Other function
    void printQMatrix();

    //! Add a new point
    void addPointCorners(cv::Point3f objectPoint, cv::Point2f imgPt_left, cv::Point2f imgPt_right);

    //! Get the value of calibration_param_loaded
    bool isCalibrationParamLoaded() const { return this->calib_param_loaded;}
    bool isInstantiated() const
    {
        return (this->instantiated == 10 && min_poses == 5
                 && board_w == 9 && board_h == 6);    //&& framesize != cv::Size());
    }

    //! Rectify two images
    int rectifyStereoImg(cv::Mat imgLeft, cv::Mat imgRight,
                          cv::Mat& rectImgLeft, cv::Mat& rectImgRight);
    //! Setters
    void setWidth(int w) { board_w = w; }
    void setHeight(int h) { board_h = h; }
    void setMinPoses(int poses) { min_poses = poses; }
    void setCM1(cv::Mat _CM1) { CM1 = _CM1; }
    void setCM2(cv::Mat _CM2) { CM2 = _CM2; }

    //! Init Method
    void init()
    {
        calib_filename = CALIB_DEFAULT_FILENAME;
        min_poses = 5;
        board_w = 9;
        board_h = 6;
        CM1 = cv::Mat(3, 3, CV_64FC1);
        CM2 = cv::Mat(3, 3, CV_64FC1);
        instantiated = 10;
        useCalibrated = true;
    }

    //! Calib parameters
    void reset()
    {
        nimages = 0;
        imagePoints_left.clear();
        imagePoints_right.clear();
        objectPoints.clear();
    }

    //! getters
    int getMinPoses() const{ return min_poses; }
    int getWeight() const{ return board_w; }
    int getHeight() const{ return board_h; }
    cv::Mat getQMatrix() const{ return Q; }
    cv::Size getFrameSize() const{ return framesize; }
    double getCalibError() const{ return calib_error; }

};

#endif /* STEREOCALIBRATION_H_ */
