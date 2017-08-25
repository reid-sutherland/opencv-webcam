/*
 * Disparity.h
 *
 *  Created on: 19 sept. 2015
 *      Author: erman
 */

#ifndef DISPARITY_H_
#define DISPARITY_H_

// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/viz/viz3d.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/core.hpp>

// System
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctime>
#include <iostream>

// Local
#include "FilenameManagement.h"
#include "StereoCalibration.h"

class Disparity {

private:
    //! Global Disparity Parameter
    int SADWindowSize;/* Match block size. It must be an odd number between 3 .. 11 */
    int numberOfDisparities;/* Minimum disparity minus Maximum Disparity: always greather than 0 and divisible by 16 */

    //! Filter parameter
    bool downscale;
    std::string filterType;//used post-filtering (wls_conf or wls_no_conf)
    std::string method;
    double wls_lambda;//parameter of post-filtering
    double wls_sigma;//parameter of post-filtering
    double vis_mult;//coefficient used to scale disparity map visualizations

    StereoCalibration& stereoCalibration;
    cv::Rect computeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance);

public:

    cv::Mat raw_disp;
    cv::Mat filtered_disp;
    double filterTime;
    double macthingTime;

    Disparity();

	virtual ~Disparity();
    //int computeNormDisp(cv::Mat imgLeft, cv::Mat imgRight, bool rectify=false);
    int computeDispMap(cv::Mat imLeft, cv::Mat imRight, bool rectify=false);

	//getters and setters
    int getSADWindowSize() const {return SADWindowSize;}
    int getNumberOfDisparities() const {return numberOfDisparities;}
    std::string getFilterType() const {return filterType;}
    double getWlsLambda() const {return wls_lambda;}
    double getWlsSigma() const {return wls_sigma;}
    double getVisMult() const {return vis_mult;}
    bool isdownscale() const {return downscale;}
    std::string getMethod() const {return method;}

    void setSADWindowSize(int wsize) { this->SADWindowSize = (wsize<=0 || wsize%2!=1 ? this->SADWindowSize : wsize); }
    void setNumberOfDisparities(int numberOfDisparities) { this->numberOfDisparities = numberOfDisparities<=0 ? this->numberOfDisparities : (((int)(numberOfDisparities/16))*16); }
    void setFilterType(std::string filterType) { this->filterType = filterType.compare("wls_conf") != 0 ? "wls_no_conf" : filterType; }
    void setWlsLambda(double wls_lambda) { this->wls_lambda = wls_lambda; }
    void setWlsSigma(double wls_sigma) { this->wls_sigma = wls_sigma; }
    void setVisMult(double vis_mult) { this->vis_mult = vis_mult; }
    void setDownscale(bool downscale) { this->downscale = downscale; }
    void setMethod(std::string method) { this->method = method.compare("sgbm") != 0 ? "bm" : method; }

	//Other function
    int saveDisparityMap(std::string file_path);
    int saveDisparityParam(std::string file_path=DISPA_DEFAULT_FILENAME);
    int loadDisparityParam(std::string file_path=DISPA_DEFAULT_FILENAME);


};

#endif /* DISPARITY_H_ */
