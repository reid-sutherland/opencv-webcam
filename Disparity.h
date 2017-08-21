/*
 * Disparity.h
 *
 *  Created on: 19 sept. 2015
 *      Author: erman
 */

#ifndef DISPARITY_H_
#define DISPARITY_H_

// Opencv Header
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

#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctime>
#include <iostream>
#include "FilenameManagement.h"
#include "StereoCalibration.h"

class Disparity {

private:
    //! Global Disparity Parameter
    int SADWindowSize;/* Match block size. It must be an odd number between 3 .. 11 */
    int numberOfDisparities;/* Minimum disparity minus Maximum Disparity: always greather than 0 and divisible by 16 */

    //! Filter parameter
    bool no_downscale;
    std::string filterType;//used post-filtering (wls_conf or wls_no_conf)
    std::string method;
    double wls_lambda;//parameter of post-filtering
    double wls_sigma;//parameter of post-filtering
    double vis_mult;//coefficient used to scale disparity map visualizations

    StereoCalibration& calibrationObject;
    cv::Rect computeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance);

public:

    cv::Mat raw_disp;
    cv::Mat filtered_disp;
    double filterTime;
    double macthingTime;

    Disparity();

	virtual ~Disparity();
    int computeNormDisp(cv::Mat imgLeft, cv::Mat imgRight, bool rectify=false);
//    int computeNormDisp(cv::Mat& filtered_disp, cv::Mat imgLeft, cv::Mat imgRight, bool rectify=true);

//    int computeDispMap(cv::Mat imLeft, cv::Mat imRight, cv::Mat &filtered_disp_vis, cv::Mat &raw_disp_vis,
//                       int max_disp=160, int wsize=-1, cv::String filter="wls_no_conf", cv::String algo="bm",
//                       bool no_downscale=false, double lambda=8000.0, double sigma=1.5, double vis_mult=1.0,
//                       bool rectify = false);
    int computeDispMap(cv::Mat imLeft, cv::Mat imRight, bool rectify=false);

	//getter and setter
    int getSADWindowSize() const {return SADWindowSize;}
    int getNumberOfDisparities() const {return numberOfDisparities;}
    std::string getFilterType() const {return filterType;}
    double getWlsLambda() const {return wls_lambda;}
    double getWlsSigma() const {return wls_sigma;}
    double getVisMult() const {return vis_mult;}
    bool isdownscale() const {return !no_downscale;}
    std::string getMethod() const {return method;}

    void setSADWindowSize(int wsize){this->SADWindowSize = (wsize<=0 || wsize%2!=1 ? this->SADWindowSize : wsize);}
    void setNumberOfDisparities(int numberOfDisparities) {this->numberOfDisparities = numberOfDisparities<=0 ? this->numberOfDisparities : (((int)(numberOfDisparities/16))*16);}
    void setFilterType(std::string filterType){this->filterType = filterType.compare("wls_conf") != 0 ? "wls_no_conf" : filterType;}
    void setWlsLambda(double wls_lambda){this->wls_lambda = wls_lambda;}
    void setWlsSigma(double wls_sigma){this->wls_sigma = wls_sigma;}
    void setVisMult(double vis_mult){this->vis_mult = vis_mult;}
    void setDownscale(bool no_downscale){this->no_downscale = !no_downscale;}
    void setMethod(std::string method){this->method = method.compare("sgbm") != 0 ? "bm" : method;}

	//Other function
    void printParameter();

    int saveDisparityMap(std::string file_path);
    int saveDisparityParam(std::string file_path=DISPA_DEFAULT_FILENAME);
    int loadDisparityParam(std::string file_path=DISPA_DEFAULT_FILENAME);


};

#endif /* DISPARITY_H_ */
