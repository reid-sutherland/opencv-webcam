/*
 * Disparity.cpp
 *
 *  Created on: 19 sept. 2015
 *  Author: erman
 */

#include "Disparity.h"

Disparity::Disparity()
    :method("bm"), wls_lambda(8000.0),
      filterType("wls_no_conf"), wls_sigma(1.5), vis_mult(1.0),
      numberOfDisparities(160), SADWindowSize(-1), no_downscale(false),
      calibrationObject(StereoCalibration::Instance())
{
//    loadDisparityParam();
}

Disparity::~Disparity() {
	// TODO Auto-generated destructor stub
}


int Disparity::computeNormDisp(cv::Mat imgLeft, cv::Mat imgRight, bool rectify)
{
    return computeDispMap(imgLeft, imgRight, rectify);
}

int Disparity::computeDispMap(cv::Mat imLeft, cv::Mat imRight, bool rectify)

{
    if (imLeft.empty() )
    {
        //ccLog::Error("[computeDispMap] Left image is empty.");
        return -1;
    }

    if (imRight.empty() )
    {
        //ccLog::Error("[computeDispMap] Right image is empty.");
        return -1;
    }

    if(rectify)
    {
        calibrationObject.rectifyStereoImg(imLeft, imRight, imLeft, imRight);
    }

#ifdef DEBUG
    printParameter();
#endif

    cv::Mat left_for_matcher, right_for_matcher;
    cv::Mat semi_filtered_disp;
    cv::Mat left_disp,right_disp;

#ifdef DEBUG
    cv::Mat conf_map;
    conf_map = cv::Mat(imLeft.rows,imLeft.cols,CV_8U);
    conf_map = cv::Scalar(255);
#endif
    cv::Rect ROI;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
#ifdef STAT
    double matching_time, filtering_time;
#endif

    if(SADWindowSize<0) //user provided window_size value
    {
        if(method.compare("sgbm")==0)
            SADWindowSize = 3; //default window size for SGBM
        else if(!no_downscale && method.compare("bm")==0 && filterType.compare("wls_conf")==0)
            SADWindowSize = 7; //default window size for BM on downscaled views (downscaling is performed only for wls_conf)
        else
            SADWindowSize = 15; //default window size for BM on full-sized views
    }

    if(numberOfDisparities<=0 || numberOfDisparities%16!=0)
    {

#ifdef DEBUG
        std::cout<<"Incorrect max_disparity value: it should be positive and divisible by 16";
#endif
        //ccLog::Error("[computeDispMap] Incorrect max_disparity value: it should be positive and divisible by 16.");
        return -1;
    }
    if(SADWindowSize<=0 || SADWindowSize%2!=1)
    {
#ifdef DEBUG
        std::cout<<"Incorrect window_size value: it should be positive and odd";
#endif
        //ccLog::Error("[computeDispMap] Incorrect window_size value: it should be positive and odd.");
        return -1;
    }
    if(filterType.compare("wls_conf")==0) // filtering with confidence (significantly better quality than wls_no_conf)
    {
        if(!no_downscale)
        {
            // downscale the views to speed-up the matching stage, as we will need to compute both left
            // and right disparity maps for confidence map computation
            //! [downscale]
            numberOfDisparities/=2;
            if(numberOfDisparities%16!=0)
                numberOfDisparities += 16-(numberOfDisparities%16);
            cv::resize(imLeft ,left_for_matcher, cv::Size(),0.5,0.5);
            cv::resize(imRight,right_for_matcher, cv::Size(),0.5,0.5);
            //! [downscale]
        }
        else
        {
            left_for_matcher  = imLeft.clone();
            right_for_matcher = imRight.clone();
        }

        if(method.compare("bm")==0)
        {
            //! [matching]
            cv::Ptr<cv::StereoBM> left_matcher = cv::StereoBM::create(numberOfDisparities,SADWindowSize);
            wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
            cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

            cv::cvtColor(left_for_matcher,  left_for_matcher,  cv::COLOR_BGR2GRAY);
            cv::cvtColor(right_for_matcher, right_for_matcher, cv::COLOR_BGR2GRAY);

#ifdef STAT
            matching_time = (double)cv::getTickCount();
#endif
            left_matcher->compute(left_for_matcher, right_for_matcher,left_disp);
            right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
#ifdef STAT
            matching_time = ((double)cv::getTickCount() - matching_time)/cv::getTickFrequency();
#endif
            //! [matching]
        }
        else if(method.compare("sgbm")==0)
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
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
#ifdef STAT
            matching_time = ((double)cv::getTickCount() - matching_time)/cv::getTickFrequency();
#endif
        }
        else
        {
#ifdef DEBUG
            std::cout<<"Unsupported algorithm";
#endif
            //ccLog::Error("[computeDispMap] Unsupported algorithm.");
            return -1;
        }

        //! [filtering]
        wls_filter->setLambda(wls_lambda);
        wls_filter->setSigmaColor(wls_sigma);
#ifdef STAT
        filtering_time = (double)cv::getTickCount();
#endif
        wls_filter->filter(left_disp,imLeft, semi_filtered_disp,right_disp);
#ifdef STAT
        filtering_time = ((double)cv::getTickCount() - filtering_time)/cv::getTickFrequency();
#endif
        //! [filtering]
#ifdef DEBUG
        conf_map = wls_filter->getConfidenceMap();
#endif

        // Get the ROI that was used in the last filter call:
        ROI = wls_filter->getROI();
        if(!no_downscale)
        {
            // upscale raw disparity and ROI back for a proper comparison:
            cv::resize(left_disp,left_disp,cv::Size(),2.0,2.0);
            left_disp = left_disp*2.0;
            ROI = cv::Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
        }
    }
    else if(filterType.compare("wls_no_conf")==0)
    {
        /* There is no convenience function for the case of filtering with no confidence, so we
        will need to set the ROI and matcher parameters manually */

        left_for_matcher  = imLeft.clone();
        right_for_matcher = imRight.clone();

        if(method.compare("bm")==0)
        {
            cv::Ptr<cv::StereoBM> matcher  = cv::StereoBM::create(numberOfDisparities,SADWindowSize);
            matcher->setTextureThreshold(0);
            matcher->setUniquenessRatio(0);
            cv::cvtColor(left_for_matcher,  left_for_matcher, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right_for_matcher, right_for_matcher, cv::COLOR_BGR2GRAY);
            ROI = computeROI(left_for_matcher.size(),matcher);
            wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
            wls_filter->setDepthDiscontinuityRadius((int)ceil(0.33*SADWindowSize));

#ifdef STAT
            matching_time = (double)cv::getTickCount();
#endif
            matcher->compute(left_for_matcher,right_for_matcher,left_disp);
#ifdef STAT
            matching_time = ((double)cv::getTickCount() - matching_time)/cv::getTickFrequency();
#endif
        }
        else if(method.compare("sgbm")==0)
        {
            cv::Ptr<cv::StereoSGBM> matcher  = cv::StereoSGBM::create(0,numberOfDisparities,SADWindowSize);
            matcher->setUniquenessRatio(0);
            matcher->setDisp12MaxDiff(1000000);
            matcher->setSpeckleWindowSize(0);
            matcher->setP1(24*SADWindowSize*SADWindowSize);
            matcher->setP2(96*SADWindowSize*SADWindowSize);
            matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
            ROI = computeROI(left_for_matcher.size(),matcher);
            wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
            wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*SADWindowSize));

#ifdef STAT
            matching_time = (double)cv::getTickCount();
#endif
            matcher->compute(left_for_matcher,right_for_matcher,left_disp);
#ifdef STAT
            matching_time = ((double)cv::getTickCount() - matching_time)/cv::getTickFrequency();
#endif
        }
        else
        {
#ifdef DEBUG
            std::cout<<"Unsupported algorithm";
#endif
            //ccLog::Error("[computeDispMap] Unsupported algorithm.");
            return -1;
        }

        wls_filter->setLambda(wls_lambda);
        wls_filter->setSigmaColor(wls_sigma);
#ifdef STAT
        filtering_time = (double)cv::getTickCount();
#endif
        wls_filter->filter(left_disp,imLeft,semi_filtered_disp,cv::Mat(),ROI);
#ifdef STAT
        filtering_time = ((double)cv::getTickCount() - filtering_time)/cv::getTickFrequency();
#endif
    }
    else
    {
#ifdef DEBUG
        std::cout<<"Unsupported filter";
#endif
        //ccLog::Error("[computeDispMap] Unsupported filter.");
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
    cv::ximgproc::getDisparityVis(semi_filtered_disp, filtered_disp,vis_mult);

    //! Get raw disparity
    cv::ximgproc::getDisparityVis(left_disp, raw_disp,vis_mult);

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
    const std::string filename = file_path;
    const std::string filename_Norm = ("norm_" + file_path);
    if (file_path != "." || file_path != " ")
    {
        cv::imwrite(filename, this->raw_disp);
        cv::imwrite(filename_Norm, this->filtered_disp);
        return 1;
    }
    return 0;
}

int Disparity::saveDisparityParam(std::string file_path)
{
    const std::string filename = file_path;
    if (file_path != "." || file_path != " ")
    {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        if (!fs.isOpened())
        {
            //ccLog::Warning("[saveDisparityParam] %s could not be opened.", filename.c_str());
            return 0;
        }
        else
        {
            fs << "SADWindowSize" << this->SADWindowSize;
            fs << "numberOfDisparities" << this->numberOfDisparities;
            fs << "no_downscale" << this->no_downscale;
            fs << "filterType" << this->filterType;
            fs << "method" << this->method;
            fs << "wls_lambda" << this->wls_lambda;
            fs << "wls_sigma" << this->wls_sigma;
            fs << "vis_mult" << this->vis_mult;

            fs.release();
            //ccLog::Print("[saveDisparityParam] Disparity matrices saved to %s.", filename.c_str());
        }
        return 1;
    }
    return 0;
}

int Disparity::loadDisparityParam(std::string file_path)
{
    const std::string filename = file_path;
    if (file_path != "." || file_path != " ")
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            //ccLog::Warning("[loadDisparityParam] %s could not be opened.", filename.c_str());
            return 0;
        }
        else
        {
            bool no_ds;
            fs["no_downscale"] >> no_ds;
            setSADWindowSize(fs["SADWindowSize"]);
            setNumberOfDisparities(fs["numberOfDisparities"]);
            setDownscale(no_ds);
            setFilterType(fs["filterType"]);
            setMethod(fs["method"]);
            setWlsLambda(fs["wls_lambda"]);
            setWlsSigma(fs["wls_sigma"]);
            setVisMult(fs["vis_mult"]);
//            fs["SADWindowSize"] >> this->SADWindowSize;
//            fs["numberOfDisparities"] >> this->numberOfDisparities;
//            fs["no_downscale"] >> this->no_downscale;
//            fs["filterType"] >> this->filterType;
//            fs["method"] >> this->method;
//            fs["wls_lambda"] >> this->wls_lambda;
//            fs["wls_sigma"] >> this->wls_sigma;
//            fs["vis_mult"] >> this->vis_mult;

            fs.release();
            //ccLog::Print("[loadDisparityParam] Disparity parameter loaded successfully to %s.", filename.c_str());
        }
        return 1;
    }
    return 0;
}

void Disparity::printParameter()
{

    /*
    ccLog::Print("[Disparity] --- Disparity Parameters ---.");
    ccLog::Print("[Disparity] Numberofdisparity: %d.", this->numberOfDisparities);
    ccLog::Print("[Disparity] SADWindowSize: %d.", this->SADWindowSize);
    ccLog::Print("[Disparity] no_downscale: %d.", this->no_downscale);
    ccLog::Print("[Disparity] filterType: %s.", this->filterType.c_str());
    ccLog::Print("[Disparity] method: %s.", this->method.c_str());
    ccLog::Print("[Disparity] wls_lambda: %lf.", this->wls_lambda);
    ccLog::Print("[Disparity] wls_sigma: %lf.", this->wls_sigma);
    ccLog::Print("[Disparity] vis_mult: %lf.", this->vis_mult);

    ccLog::Print("[Disparity] --- End of displaying parameters ---.");
    */
}

