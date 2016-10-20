#ifndef cluster_h
#define cluster_h

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>

class clusterDetection
{
  public:
    clusterDetection(unsigned int minPoints,unsigned int radiusMM);
    void cluster(sensor_msgs::PointCloud2 &cloud2);

    
  private:
  	pcl::PointCloud<pcl::PointXYZ> nonClumpedPoints;

};

#endif
