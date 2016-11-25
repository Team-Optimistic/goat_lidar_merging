#include "goat_lidar_merging/clusterDetection.h"
double clusterDetection::squaredDistance;
pcl::PointCloud<pcl::PointXYZ> clusterDetection::objects;


clusterDetection::clusterDetection(unsigned int minPoints,unsigned int radiusMM):
nonClumpedPoints(new pcl::PointCloud<pcl::PointXYZ>)
{
	tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);


	squaredDistance = (4.0*radiusMM * radiusMM)*1e-6;
	std::cout << squaredDistance << std::endl;
	extractor.setClusterTolerance (radiusMM * 1e-3);
	extractor.setMinClusterSize (minPoints);
}

constexpr float computeSquared(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
{
	return (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y);
}

bool clusterDetection::isUnwanted(const pcl::PointXYZ &point){
    const float field_margin = 0.1;
    const float field_length = 3.6576;
	for(unsigned i=0; i <objects.size();i++){
		if(computeSquared(point, objects.at(i)) < squaredDistance)
			return true;
	}
    if(point.x < 0 + field_margin || point.y < 0 + field_margin)
        return true;
    if(point.x > field_length - field_margin || point.y >  field_length/2 - field_margin)
        return true;
	return false;
}
//remove points of objects already detectedct
void clusterDetection::removeUnwantedPoints(pcl::PointCloud<pcl::PointXYZ> &cloud){
	cloud.erase(std::remove_if(cloud.begin(),cloud.end(),clusterDetection::isUnwanted),cloud.end());
}

const sensor_msgs::PointCloud2 clusterDetection::cluster(const sensor_msgs::PointCloud2 &cloud2){
	std::vector<pcl::PointIndices> cluster_indices;
    pcl::PointIndices::Ptr points_in_clusters (new pcl::PointIndices ());


    pcl::PointCloud<pcl::PointXYZ> newScan; //temp clouds
    fromROSMsg(cloud2,newScan);
    int oldPoints = newScan.size();
    clusterDetection::removeUnwantedPoints(newScan);
   //std::cout << "removed  " << oldPoints - newScan.size()  <<std::endl;
    (*nonClumpedPoints)+=newScan;//new scna now only contains points that add new knowledge
    //std::cout <<"total points in objects " << points_in_clusters->indices.size() <<std::endl;

    tree->setInputCloud(nonClumpedPoints);

    extractor.setSearchMethod(tree);
    extractor.setInputCloud(nonClumpedPoints);
    extractor.extract(cluster_indices);

    int j = 0;
	//std::cout << "objects detected this loop  " << cluster_indices.size() << std::endl;
	//std::cout << "points in this loop  " << nonClumpedPoints->size() << std::endl;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {   
    	objects.points.push_back(nonClumpedPoints->points[*it->indices.begin()]);
    	
        points_in_clusters->indices.insert(points_in_clusters->indices.begin(),it->indices.begin(),it->indices.end());
    	//std::cout <<"object found at  "<< nonClumpedPoints->points[*it->indices.begin()].x << "  " << nonClumpedPoints->points[*it->indices.begin()].y <<"  "<<std::endl;
    	//std::cout <<"this cluster contains " << it->indices.size() <<std::endl;
    }
   // std::cout << "object list size  " << objects.size() << std::endl;
    sensor_msgs::PointCloud2 rosObjectList;
    toROSMsg(objects,rosObjectList);
    return rosObjectList;
}
