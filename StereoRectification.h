/*
 * StereoRectification.h
 *
 *  Created on: 19 sept. 2015
 *      Author: erman
 */

#ifndef STEREORECTIFICATION_H_
#define STEREORECTIFICATION_H_

// Opencv Header
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctime>
#include <iostream>
#include "StereoCalibration.h"
#include "FilenameManagement.h"


class StereoRectification {

private:
    cv::Rect validRoi[2];
    std::string retroProjectionMatFilename;

public:

    cv::Mat map1x;
    cv::Mat map1y;
    cv::Mat map2x;
    cv::Mat map2y;
    cv::Mat Q;

    //! Constructor
    StereoRectification(std::string retroProjectionMatFilename = QMATRICE_DEFAULT_FILENAME);
    virtual ~StereoRectification();

    //! fonction to save rectification parameters in the calib_filename
    void rectify(StereoCalibration& sc, bool saveQMatrice = false);

    //! save rectification matrice Q
    int saveQMatrice();

    //! reload rectification parameters
    int loadQMatrice();

    //! Rectify a stereo image
    void rectifyStereoImg(StereoCalibration& sc, cv::Mat imgLeft, cv::Mat imgRight, cv::Mat& imgLeftRect, cv::Mat& imgRightRect, cv::Mat& Q);
    void rectifyStereoImg(StereoCalibration& sc, cv::Mat imgLeft, cv::Mat imgRight, cv::Mat& imgLeftRect, cv::Mat& imgRightRect);

};

#endif /* STEREORECTIFICATION_H_ */
