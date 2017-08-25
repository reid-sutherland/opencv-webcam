/*
 * PointCloudObjectDetection.cpp
 *
 *  Created on: 13 july 2017
 *  Author: Arthur Hamelin
 */

#include "PCObjectDetection.h"
#include "PCDetectedObject.h"

//PCL
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>

// Constructor
PCObjectDetection::PCObjectDetection(){

}

// This method is used to remove the planes from the pointcloud given.
// It will be usefull to detect several objects inside the given point cloud.
int PCObjectDetection::detectionPlaneSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
  //std::cout << "input point cloud " << cloud->points.size()<< std::endl;

  // Get the plane model, if present.
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
  segmentation.setInputCloud(cloud);
  segmentation.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType (pcl::SAC_RANSAC);
  segmentation.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
  segmentation.setEpsAngle((20*3.14)/180*M_PI);
  segmentation.setDistanceThreshold(0.01);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  segmentation.segment(*planeIndices, *coefficients);

  if (planeIndices->indices.empty())
          std::cout << "Could not find a plane in the scene." << std::endl;
  else
  {
    // Copy the points of the plane to a new cloud.
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(planeIndices);
    extract.filter(*plane);
    //std::cout << "plane point cloud " << plane->points.size()<< std::endl;

    // Retrieve the convex hull.
    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    hull.setInputCloud(plane);
    // Make sure that the resulting hull is bidimensional.
    hull.setDimension(2);
    hull.reconstruct(*convexHull);
    //std::cout << "convexHull point cloud " << convexHull->points.size()<< std::endl;

    // Redundant check.
    if (hull.getDimension() == 2)
    {
      // Prism object.
      pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
      prism.setInputCloud(cloud);
      prism.setInputPlanarHull(convexHull);
      // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
      // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
      // Set 1cm for the minimum value for the first parameter to erase the plane where the object is on.
      // Otherwise you'll still get the plane and the object.
      prism.setHeightLimits(0.01f, 0.5f);
      pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

      prism.segment(*objectIndices);

      // Get and show all points retrieved by the hull.
      extract.setIndices(objectIndices);
      extract.filter(*objects);
      //std::cout << "objects point cloud " << objects->points.size()<< std::endl;

      // Saving the Segmentation PointCloud (that can be returned).
      pcl::io::savePCDFileASCII("/home/reid/opencv-webcam/applicationFiles/PlanarSeg/planarSeg.pcd", *objects);

      pcl::visualization::CloudViewer viewerObjects("Objects on table");
      viewerObjects.showCloud(objects);
      while (!viewerObjects.wasStopped())
      {
        // Do nothing but wait.
      }
    }
    else std::cout << "The chosen hull is not planar." << std::endl;
  }

  return 1;

}

// This method is used to detect different clusters inside the point cloud.
// Due to their min size, I assume that all clusters detected are objects we anted to detect inside the point cloud.
int PCObjectDetection::detectionEuclidianClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){

  // kd-tree object for searches.
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kdtree->setInputCloud(cloud);

  // Euclidean clustering object.
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
  // Set cluster tolerance to 2cm (small values may cause objects to be divided
  // in several clusters, whereas big values may join objects in a same cluster).
  clustering.setClusterTolerance(0.02); // 2cm
  // Set the minimum and maximum number of points that a cluster can have.
  clustering.setMinClusterSize(3000);
  clustering.setMaxClusterSize(25000);
  clustering.setSearchMethod(kdtree);
  clustering.setInputCloud(cloud);
  std::vector<pcl::PointIndices> clusters;
  clustering.extract(clusters);
  printf("Starting computing clusters. \n");
  // For every cluster...
  int currentClusterNum = 1;
  for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
  {
    printf("Cluster number : %d \n",currentClusterNum);
    // ...add all its points to a new cloud...
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++){
      cluster->points.push_back(cloud->points[*point]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    // ...and save it to disk.
    if (cluster->points.size() <= 0){
      printf("Error cluster is empty \n");
      break;
      }
    std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;

    // Saving Cluster to repository (can be returned)
    std::string fileName = "/home/arthur/Documents/Internship/stereovis/applicationFiles/Cluster/cluster" + boost::to_string(currentClusterNum) + ".pcd";
    pcl::io::savePCDFileASCII(fileName, *cluster);
    std::cout << "Cluster saved succesfully. \n" << std::endl;

    // Create a detectedObject from the cluster and estimate distance to it and its size.
    PCDetectedObject object = PCDetectedObject(cluster);
    std::cout << "Distance from objective is : " << object.getObjectDistanceFromObjective() << " meters." << std::endl;



    currentClusterNum++;
  }

  return 1;
}
