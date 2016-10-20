#include "goat_lidar_merging/clusterDetection.h"

clusterDetection::clusterDetection(unsigned int minPoints,unsigned int radiusMM){



}

void clusterDetection::cluster(sensor_msgs::PointCloud2 &cloud2){
    pcl::PointCloud<pcl::PointXYZ> newScan; //temp clouds
	fromROSMsg(cloud2,newScan);
	nonClumpedPoints+=newScan;

}
