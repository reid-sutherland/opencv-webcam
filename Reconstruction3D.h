/*
 * Reconstruction3D.h
 *
 *  Created on: 19 sept. 2015
 *      Author: erman
 */

#ifndef RECONSTRUCTION3D_H_
#define RECONSTRUCTION3D_H_

// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// System
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctime>
#include <iostream>

// Local
#include "StereoCalibration.h"
#include "StereoRectification.h"
#include "Disparity.h"
#include "Util.h"
#include "ViewImage.h"

class Reconstruction3D {

private:
    //Global Reconstruction3D Parameterss
    Disparity& disp;    //This makes disp a reference to disparityObject in main
    double maxDepth;
    double alpha;
    int bufsize;
    bool handleMissingValues;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;
    bool print;

public:

	//Constructor
    //Reconstruction3D();
    Reconstruction3D(Disparity& disparity);
	virtual ~Reconstruction3D();

    int buildPointCloud(cv::Mat& img_rgb_left, cv::Mat& img_rgb_right, StereoCalibration& sc);

    int computeDistance(cv::Mat Q, cv::Mat img_disparity, cv::Mat &);

    int customProject3d(cv::Mat Q, cv::Mat img_rgb, cv::Mat img_disparity,
                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr);

    //Other function
    void printQMatrix(cv::Mat Q);

    Disparity& getDisparityObj() {return disp;}

    //! getters and setters
    /**
     * Maximum depth value. Typical value range form 10.0 to 10000. The default value is: 1000.0
    */
    double getMaxDepth() const {return maxDepth;}
    void setMaxDepth(double maxDepth){this->maxDepth = maxDepth;}

    /**
     * Depth scaling, allow us to increase the depth of generated 3D image.
     * Typical values range from 0.5 to 10.0. The default value is 1.0
    */
    double getAlpha() const {return alpha;}
    void setAlpha(double alpha){this->alpha = alpha;}

    /**
     * The number of image to use while generating a point cloud.
    */
    int getBufSize() const {return bufsize;}
    void setBufSize(int bufsize){this->bufsize = bufsize;}

    /**
     * It's the parameter use to handle missing value on disparity map.
     * When it's false, the missing value are ignore.
    */
    bool isHandleMissingValues() const {return handleMissingValues;}
    void setHandleMissingValues(bool handleMissingValues){this->handleMissingValues = handleMissingValues;}

    /**
     * The point cloud data.
    */
    void retrievePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const {*cloud = *point_cloud_ptr;}



};

#endif /* RECONSTRUCTION3D_H_ */
