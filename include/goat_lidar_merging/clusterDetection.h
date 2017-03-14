#pragma once

#include <algorithm>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>

class clusterDetection
{
public:
  clusterDetection(const unsigned int minPoints, const unsigned int radiusMM, const unsigned int starSize, const unsigned int cubeSize);

  void cluster(const sensor_msgs::PointCloud2 &cloud2);
  static bool isUnwanted(const pcl::PointXYZ &point);
  static bool wasRemoved(const pcl::PointXYZ &point);
  void removeUnwantedPoints(pcl::PointCloud<pcl::PointXYZ> &cloud);

  void sensor_msgs::PointCloud2 get_small_objects() const;
  void sensor_msgs::PointCloud2 get_big_objects() const;

  static pcl::PointCloud<pcl::PointXYZ> big_objects, small_objects; //temp clouds
  static double small_squared_Distance, big_squared_Distance;
private:
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr nonClumpedPoints;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
};
