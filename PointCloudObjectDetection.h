/*
 *  PointCloudDisplay.h
 *
 *  Created on: 13 july. 2017
 *  Author: Arthur Hamelin
 */
 
#ifndef POINTCLOUDOBJECTDETECTION_H_
#define POINTCLOUDOBJECTDETECTION_H_

// PCL Library

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>

// System

#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctime>
#include <iostream>

class PointCloudObjectDetection{

public:

  // Constructor
  PointCloudObjectDetection();
  
  // Methods
  int detectionPlaneSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
  int detectionEuclidianClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

};

#endif /* POINTCLOUDOBJECTDETECTION_H_  */
