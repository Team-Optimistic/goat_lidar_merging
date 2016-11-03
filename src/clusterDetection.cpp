#include "goat_lidar_merging/clusterDetection.h"
double clusterDetection::squaredDistance;
pcl::PointCloud<pcl::PointXYZ> clusterDetection::objects;


clusterDetection::clusterDetection(unsigned int minPoints,unsigned int radiusMM){
	cec = new pcl::ConditionalEuclideanClustering<pcl::PointXYZ> (true);

	squaredDistance = (4.0*radiusMM * radiusMM)*1e-6;
  	//cec->setConditionFunction (&customRegionGrowing);
	cec->setClusterTolerance (radiusMM);
	cec->setMinClusterSize (minPoints);
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
void clusterDetection::removeRedundantPoints(pcl::PointCloud<pcl::PointXYZ> &cloud){
	std::remove_if(cloud.begin(),cloud.end(),clusterDetection::alreadyKnown);
	}


	void clusterDetection::cluster(sensor_msgs::PointCloud2 &cloud2){
    pcl::PointCloud<pcl::PointXYZ> newScan; //temp clouds
    fromROSMsg(cloud2,newScan);
    nonClumpedPoints+=newScan;

	//cec->setInputCloud (nonClumpedPoints);

	//cec->segment (*clusters);
  	//cec->getRemovedClusters (small_clusters, large_clusters);
}
