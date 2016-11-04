#include "goat_lidar_merging/clusterDetection.h"
double clusterDetection::squaredDistance;
pcl::PointCloud<pcl::PointXYZ> clusterDetection::objects;


clusterDetection::clusterDetection(unsigned int minPoints,unsigned int radiusMM){
	tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);


	squaredDistance = (4.0*radiusMM * radiusMM)*1e-6;
	extractor->setClusterTolerance (radiusMM);
	extractor->setMinClusterSize (minPoints);
}

constexpr float computeSquared(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
{
	return (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y);
}

bool clusterDetection::alreadyKnown(const pcl::PointXYZ &point){
	for(unsigned i=0; i <objects.size();i++){
		if(computeSquared(point, objects.at(i)) < squaredDistance)
			return true;
	}
	return false;
}
//remove points of objects already detectedct
void clusterDetection::removeRedundantPoints(pcl::PointCloud<pcl::PointXYZ> &cloud){
	std::remove_if(cloud.begin(),cloud.end(),clusterDetection::alreadyKnown);
}


void clusterDetection::cluster(sensor_msgs::PointCloud2 &cloud2){
	std::vector<pcl::PointIndices> cluster_indices;


    pcl::PointCloud<pcl::PointXYZ> newScan; //temp clouds
    fromROSMsg(cloud2,newScan);
    removeRedundantPoints(newScan);
    *nonClumpedPoints+=newScan;//new scna now only contains points that add new knowledge

    tree->setInputCloud(nonClumpedPoints);

    extractor->setSearchMethod (tree);
	extractor->setInputCloud (nonClumpedPoints);
	extractor->extract (cluster_indices);
  	//extractor->getRemovedClusters (small_clusters, large_clusters);
}
