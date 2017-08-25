/*
 * PointCloudDetectedObject.h
 *
 *  Created on: 26 july 2017
 *  Author: Arthur Hamelin
 */
 
#ifndef POINTCLOUDDETECTEDOBJECT_H_
#define POINTCLOUDDETECTEDOBJECT_H_

// PCL Library
#include <pcl/io/pcd_io.h>

// System
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctime>
#include <iostream>

class PCDetectedObject{

private:

  //Attribute
  int objectHeight;
  int objectWidth;
  double objectDistanceFromObjective;
  pcl::PointCloud<pcl::PointXYZRGB> objectCloud;
  
  //Setter
  void setObjectHeight(int height);
  void setObjectWidth(int width);
  void setObjectDistanceFromObjective();
  void setObjectCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud);

public:

  // Constructor
  PCDetectedObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
  
  // Methods
  
  //Getter
  int getObjectHeight();
  int getObjectWidth();
  double getObjectDistanceFromObjective();

};

#endif /* POINTCLOUDDETECTEDOBJECT_H_  */
