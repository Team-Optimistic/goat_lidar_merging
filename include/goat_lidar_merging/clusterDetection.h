#ifndef cluster_h
#define cluster_h

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
    clusterDetection(unsigned int minPoints,unsigned int radiusMM);
    const sensor_msgs::PointCloud2 cluster(const sensor_msgs::PointCloud2 &cloud2);
    static bool isUnwanted(const pcl::PointXYZ &point);
    void removeUnwantedPoints(pcl::PointCloud<pcl::PointXYZ> &cloud);
    static pcl::PointCloud<pcl::PointXYZ> objects; //temp clouds
    static double squaredDistance;


private:
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr nonClumpedPoints; 
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;


};

#endif
