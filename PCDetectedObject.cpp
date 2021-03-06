/*
 * PointCloudDetectedObject.cpp
 *
 *  Created on: 26 july 2017
 *  Author: Arthur Hamelin
 */

#include "PCDetectedObject.h"

// Constructor
PCDetectedObject::PCDetectedObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){

  this->setObjectCloud(*cloud);
  this->setObjectDistanceFromObjective();
  this->setObjectHeight(cloud->height);
  this->setObjectWidth(cloud->width);

}

// This method set the point cloud to be the point cloud of the detected object
void PCDetectedObject::setObjectCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud){
  this->objectCloud = cloud;
}

// This method set the height of the object.
// For now this method set the height of the point cloud without calculating the actual height of the object.
void PCDetectedObject::setObjectHeight(int height){
  this->objectHeight = height;
}

// This method set the width of the object.
// For now this method set the width of the point cloud without calculating the actual height of the object.
void PCDetectedObject::setObjectWidth(int width){
  this->objectWidth =  width;
}

// More than just a simple setter, this will compute the minimal distance from the objective.
void PCDetectedObject::setObjectDistanceFromObjective(){

  pcl::PointCloud<pcl::PointXYZRGB> cloud = this->objectCloud;
  double minDistance=0.0;

  // Initializing
  minDistance=hypot(cloud.points[0].z, cloud.points[0].x);
  for (int i=1; i< cloud.points.size(); i++){
      if(hypot(cloud.points[i].z, cloud.points[i].x)<minDistance){
        // keep updating the minimum Distant point
        minDistance=hypot(cloud.points[i].z, cloud.points[i].x);
      }
    }

  this->objectDistanceFromObjective = minDistance;
}

// This method return the height of the detected object.
int PCDetectedObject::getObjectHeight(){
  return this->objectHeight;
}

// This method return the height of the detected object.
int PCDetectedObject::getObjectWidth(){
  return this->objectWidth;
}

// This method return the distance of the detected object from the camera objective.
double PCDetectedObject::getObjectDistanceFromObjective(){
  return this->objectDistanceFromObjective;
}
