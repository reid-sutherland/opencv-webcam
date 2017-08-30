/*
 * Disparity.cpp
 *
 *  Created on: 19 sept. 2015
 *  Author: erman
 */

//#define DEBUG
//#define STAT

#include "Disparity.h"

using namespace cv;

Disparity::Disparity()
    : method("bm"), wls_lambda(8000.0),
      filterType("wls_conf"), wls_sigma(1.5), vis_mult(1.0),
      numberOfDisparities(80), SADWindowSize(-1), downscale(false),
      stereoCalibration(StereoCalibration::Instance())
{
    // Note: downscaling only works if bm and wls_conf
    // bm, wls_conf, 1.5, 160, true is terrible
    // bm, wls_conf, 1.5, 160, false is okay but rather blurry
    // bm, wls_no_conf, 1.5, 160, is pretty good, a little buggy
    // bm, wls_conf, 1.5, 80, best so far
}

Disparity::~Disparity() {

}

/*  this function is seemingly pointless
int Disparity::computeNormDisp(cv::Mat imgLeft, cv::Mat imgRight, bool rectify)
{
    return computeDispMap(imgLeft, imgRight, rectify);
}
*/

int Disparity::computeDispMap(cv::Mat imLeft, cv::Mat imRight, bool rectify)
{
    if (imLeft.empty() )
    {
        std::cerr << "[computeDispMap] Error: Left image is empty." << std::endl;
        return -1;
    }

    if (imRight.empty() )
    {
        std::cerr << "[computeDispMap] Error: Right image is empty." << std::endl;
        return -1;
    }

    if(rectify)
    {
        stereoCalibration.rectifyStereoImg(imLeft, imRight, imLeft, imRight);
    }

    Mat leftFrame, rightFrame;
    Mat semi_filtered_disp;
    Mat left_disp, right_disp;

    #ifdef DEBUG
    cv::Mat conf_map;
    conf_map = cv::Mat(imLeft.rows,imLeft.cols,CV_8U);
    conf_map = cv::Scalar(255);
    #endif

    Rect ROI;
    Ptr<ximgproc::DisparityWLSFilter> wls_filter;

    #ifdef STAT
    double matching_time, filtering_time;
    #endif

    if(SADWindowSize<0) //user provided window_size value
    {
        if (method == "sgbm")
            SADWindowSize = 3; //default window size for SGBM
        else if(downscale && method == "bm" && filterType == "wls_conf")
            SADWindowSize = 7; //default window size for BM on downscaled views (downscaling is performed only for wls_conf)
        else
            SADWindowSize = 15; //default window size for BM on full-sized views
    }

    if (numberOfDisparities<=0 || numberOfDisparities%16!=0)
    {
        std::cerr << "[computeDispMap] Error: Incorrect max_disparity value: it should be positive and divisible by 16." << std::endl;
        return -1;
    }

    if (SADWindowSize<=0 || SADWindowSize%2!=1)
    {
        std::cerr << "[computeDispMap] Error: Incorrect window_size value: it should be positive and odd." << std::endl;
        return -1;
    }
    if (filterType == "wls_conf") // filtering with confidence (significantly better quality than wls_no_conf)
    {
        if (downscale)
        {
            // downscale the views to speed-up the matching stage, as we will need to compute both left
            // and right disparity maps for confidence map computation
            //! [downscale]
            numberOfDisparities/=2;
            if(numberOfDisparities%16!=0)
                numberOfDisparities += 16-(numberOfDisparities%16);
            resize(imLeft, leftFrame, Size(), 0.5, 0.5);
            resize(imRight, rightFrame, Size(), 0.5, 0.5);
            //! [downscale]
        }
        else
        {
            leftFrame  = imLeft.clone();
            rightFrame = imRight.clone();
        }

        if (method == "bm")
        {
            //! [matching]
            Ptr<StereoBM> left_matcher = StereoBM::create(numberOfDisparities, SADWindowSize);
            wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);

            cvtColor(leftFrame, leftFrame,  COLOR_BGR2GRAY);
            cvtColor(rightFrame, rightFrame, COLOR_BGR2GRAY);

                #ifdef STAT
                matching_time = (double)cv::getTickCount();
                #endif

            left_matcher->compute(leftFrame, rightFrame, left_disp);
            right_matcher->compute(rightFrame, leftFrame, right_disp);

                #ifdef STAT
                matching_time = ((double)cv::getTickCount() - matching_time)/cv::getTickFrequency();
                #endif
            //! [matching]
        }
        else if (method == "sgbm")
        {
            cv::Ptr<cv::StereoSGBM> left_matcher  = cv::StereoSGBM::create(0,numberOfDisparities, SADWindowSize);
            left_matcher->setP1(24*SADWindowSize*SADWindowSize);
            left_matcher->setP2(96*SADWindowSize*SADWindowSize);
            left_matcher->setPreFilterCap(63);
            left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
            wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
            cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

                #ifdef STAT
                matching_time = (double)cv::getTickCount();
                #endif

            left_matcher->compute(leftFrame, rightFrame, left_disp);
            right_matcher->compute(rightFrame, leftFrame, right_disp);

                #ifdef STAT
                matching_time = ((double)cv::getTickCount() - matching_time)/cv::getTickFrequency();
                #endif
        }
        else
        {
            std::cerr << "[computeDispMap] Error: Unsupported algorithm." << std::endl;
            return -1;
        }

        //! [filtering]
        wls_filter->setLambda(wls_lambda);
        wls_filter->setSigmaColor(wls_sigma);

            #ifdef STAT
            filtering_time = (double)cv::getTickCount();
            #endif

        wls_filter->filter(left_disp, leftFrame, semi_filtered_disp, right_disp);

            #ifdef STAT
            filtering_time = ((double)cv::getTickCount() - filtering_time)/cv::getTickFrequency();
            #endif
        //! [filtering]

            #ifdef DEBUG
            conf_map = wls_filter->getConfidenceMap();
            #endif

        // Get the ROI that was used in the last filter call:
        ROI = wls_filter->getROI();
        if (downscale)
        {
            // upscale raw disparity and ROI back for a proper comparison:
            cv::resize(left_disp,left_disp,cv::Size(),2.0,2.0);
            left_disp = left_disp*2.0;
            ROI = cv::Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
        }
    }
    else if (filterType == "wls_no_conf")
    {
        /* There is no convenience function for the case of filtering with no confidence, so we
        will need to set the ROI and matcher parameters manually */

        leftFrame  = imLeft.clone();
        rightFrame = imRight.clone();

        if (method == "bm")
        {
            cv::Ptr<cv::StereoBM> matcher  = cv::StereoBM::create(numberOfDisparities,SADWindowSize);
            matcher->setTextureThreshold(0);
            matcher->setUniquenessRatio(0);
            cv::cvtColor(leftFrame, leftFrame, cv::COLOR_BGR2GRAY);
            cv::cvtColor(rightFrame, rightFrame, cv::COLOR_BGR2GRAY);
            ROI = computeROI(leftFrame.size(), matcher);
            wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
            wls_filter->setDepthDiscontinuityRadius((int)ceil(0.33*SADWindowSize));

                #ifdef STAT
                matching_time = (double)cv::getTickCount();
                #endif
            matcher->compute(leftFrame, rightFrame, left_disp);
                #ifdef STAT
                matching_time = ((double)cv::getTickCount() - matching_time)/cv::getTickFrequency();
                #endif
        }
        else if (method == "sgbm")
        {
            cv::Ptr<cv::StereoSGBM> matcher  = cv::StereoSGBM::create(0,numberOfDisparities,SADWindowSize);
            matcher->setUniquenessRatio(0);
            matcher->setDisp12MaxDiff(1000000);
            matcher->setSpeckleWindowSize(0);
            matcher->setP1(24*SADWindowSize*SADWindowSize);
            matcher->setP2(96*SADWindowSize*SADWindowSize);
            matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
            ROI = computeROI(leftFrame.size(), matcher);
            wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
            wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*SADWindowSize));

                #ifdef STAT
                matching_time = (double)cv::getTickCount();
                #endif
            matcher->compute(leftFrame, rightFrame, left_disp);
                #ifdef STAT
                matching_time = ((double)cv::getTickCount() - matching_time)/cv::getTickFrequency();
                #endif
        }
        else
        {
            std::cerr << "[computeDispMap] Error: Unsupported algorithm." << std::endl;
            return -1;
        }

        wls_filter->setLambda(wls_lambda);
        wls_filter->setSigmaColor(wls_sigma);
            #ifdef STAT
            filtering_time = (double)cv::getTickCount();
            #endif
        wls_filter->filter(left_disp, leftFrame, semi_filtered_disp, cv::Mat(), ROI);
            #ifdef STAT
            filtering_time = ((double)cv::getTickCount() - filtering_time)/cv::getTickFrequency();
            #endif
    }
    else
    {
        std::cerr << "[computeDispMap] Error: Unsupported filter." << std::endl;
        return -1;
    }

    #ifdef DEBUG
    //collect and print all the stats:
    std::cout.precision(2);
    #ifdef STAT
    std::cout<<"Matching time:  "<<matching_time<<"s"<<std::endl;
    std::cout<<"Filtering time: "<<filtering_time<<"s"<<std::endl;
    #endif
    std::cout<<std::endl;
    #endif

    //! Get filter disparity
    ximgproc::getDisparityVis(semi_filtered_disp, filtered_disp, vis_mult);

    //! Get raw disparity
    ximgproc::getDisparityVis(left_disp, raw_disp, vis_mult);

    return 0;
}

cv::Rect Disparity::computeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();
#ifdef DEBUG
    std::cout << "min_disparity: " << min_disparity << "\n";
    std::cout << "num_disparities: " << num_disparities << "\n";
    std::cout << "block_size: " << block_size << "\n";
#endif
    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    cv::Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
    return r;
}

int Disparity::saveDisparityMap(std::string file_path)
{
    const std::string filename_Norm = ("norm_" + file_path);
    if (file_path != "." || file_path != " ")
    {
        cv::imwrite(file_path, this->raw_disp);
        cv::imwrite(filename_Norm, this->filtered_disp);
        return 1;
    }
    return 0;
}

int Disparity::saveDisparityParam(std::string file_path)
{
    if (file_path != "." || file_path != " ")
    {
        cv::FileStorage fs(file_path, cv::FileStorage::WRITE);
        if (!fs.isOpened())
        {
            std::cerr << "[saveDisparityParam] " << file_path.c_str() << " could not be opened." << std::endl;
            return 0;
        }
        else
        {
            fs << "SADWindowSize" << this->SADWindowSize;
            fs << "numberOfDisparities" << this->numberOfDisparities;
            fs << "downscale" << this->downscale;
            fs << "filterType" << this->filterType;
            fs << "method" << this->method;
            fs << "wls_lambda" << this->wls_lambda;
            fs << "wls_sigma" << this->wls_sigma;
            fs << "vis_mult" << this->vis_mult;

            fs.release();
        }
        return 1;
    }
    return 0;
}

int Disparity::loadDisparityParam(std::string file_path)
{
    if (file_path != "." || file_path != " ")
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            std::cerr << "[loadDisparityParam] " << file_path.c_str() << " could not be opened." << std::endl;
            return 0;
        }
        else
        {
            bool ds;
            fs["downscale"] >> ds;
            setSADWindowSize(fs["SADWindowSize"]);
            setNumberOfDisparities(fs["numberOfDisparities"]);
            setDownscale(ds);
            setFilterType(fs["filterType"]);
            setMethod(fs["method"]);
            setWlsLambda(fs["wls_lambda"]);
            setWlsSigma(fs["wls_sigma"]);
            setVisMult(fs["vis_mult"]);

            fs.release();
        }
        return 1;
    }
    return 0;
}
