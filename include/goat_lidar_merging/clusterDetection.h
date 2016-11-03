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
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/geometry.h>


class clusterDetection
{
  public:
    clusterDetection(unsigned int minPoints,unsigned int radiusMM);
    void cluster(sensor_msgs::PointCloud2 &cloud2);
    static bool alreadyKnown(const pcl::PointXYZ &point);
    void removeRedundantPoints(pcl::PointCloud<pcl::PointXYZ> &cloud);
    static pcl::PointCloud<pcl::PointXYZ> objects; //temp clouds
    static double squaredDistance;

    
  private:
	pcl::ConditionalEuclideanClustering<pcl::PointXYZ> * cec;
  pcl::PointCloud<pcl::PointXYZ> nonClumpedPoints;

};

#endif