/*
 *  PointCloudDisplay.h
 *
 *  Created on: 13 july. 2017
 *  Author: Arthur Hamelin
 */
 
#ifndef POINTCLOUDOBJECTDETECTION_H_
#define POINTCLOUDOBJECTDETECTION_H_

//PCL
#include <pcl/io/pcd_io.h>

class PCObjectDetection{

public:

  // Constructor
  PCObjectDetection();
  
  // Methods
  int detectionPlaneSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
  int detectionEuclidianClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

};

#endif /* POINTCLOUDOBJECTDETECTION_H_  */
