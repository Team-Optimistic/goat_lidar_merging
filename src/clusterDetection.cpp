#include "goat_lidar_merging/clusterDetection.h"
double clusterDetection::big_squared_Distance;
double clusterDetection::small_squared_Distance;

pcl::PointCloud<pcl::PointXYZ> clusterDetection::big_objects; 
pcl::PointCloud<pcl::PointXYZ> clusterDetection::small_objects; 

clusterDetection::clusterDetection(unsigned int minPoints,unsigned int radiusMM, unsigned int starSize, unsigned int cubeSize):
nonClumpedPoints(new pcl::PointCloud<pcl::PointXYZ>)
{
	tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);


	small_squared_Distance = (starSize * starSize)*1e-6;
    big_squared_Distance = (cubeSize * cubeSize) *1e-6;
    extractor.setClusterTolerance (radiusMM * 1e-3);
    extractor.setMinClusterSize (minPoints);
}

constexpr float computeSquared(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
{
	return (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y);
}

bool clusterDetection::wasRemoved(const pcl::PointXYZ &point){
    if(point.z > 5.0)
        return true;
    return false;
}
bool clusterDetection::isUnwanted(const pcl::PointXYZ &point){
    const float field_margin = 0.1;
    const float field_length = 3.6576;
    for(unsigned i=0; i <big_objects.size();i++){
      if(computeSquared(point, big_objects.at(i)) < big_squared_Distance)
         return true;
 }
 for(unsigned i=0; i <small_objects.size();i++){
    if(computeSquared(point, small_objects.at(i)) < small_squared_Distance)
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
    pcl::ExtractIndices<pcl::PointXYZ> eifilter (false);

    pcl::PointCloud<pcl::PointXYZ> newScan; //temp clouds
    fromROSMsg(cloud2,newScan);
    int oldPoints = newScan.size();
    clusterDetection::removeUnwantedPoints(newScan);
    (*nonClumpedPoints)+=newScan;//new scna now only contains points that add new knowledge

    tree->setInputCloud(nonClumpedPoints);
    std::cout << "objects pre loop  " << cluster_indices.size() << std::endl;

    extractor.setSearchMethod(tree);
    extractor.setInputCloud(nonClumpedPoints);
    extractor.extract(cluster_indices);
    std::cout << "objects detected this loop  " << cluster_indices.size() << std::endl;
    eifilter.setInputCloud(nonClumpedPoints);


    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {   
        float x =0;
        float y =0;
        int points =0;
        for(int i = 0; i<it->indices.size(); i++){
            x += nonClumpedPoints->points[it->indices[i]].x;
            y += nonClumpedPoints->points[it->indices[i]].y;
            points++;
        }
        pcl::PointXYZ new_Object;
        new_Object.x = x/points;
        new_Object.y = y/points;
        float biggestDistance =0.0;
        for(int i = 0; i<it->indices.size(); i++){
            float currentDist = computeSquared(nonClumpedPoints->points[it->indices[i]],new_Object);
            if(currentDist>biggestDistance)
                biggestDistance = currentDist; 
        }
        if(biggestDistance > small_squared_Distance){
            new_Object.z = 2;
            big_objects.points.push_back(new_Object);
        }else{
            new_Object.z = 1;
            small_objects.points.push_back(new_Object);
        }
        points_in_clusters->indices.insert(points_in_clusters->indices.begin(),it->indices.begin(),it->indices.end());

    }
    eifilter.setIndices(points_in_clusters);
    eifilter.setNegative (true);
    eifilter.setUserFilterValue (1337.0);
    eifilter.filterDirectly (nonClumpedPoints);
    //std::cout << "initial Points  " << nonClumpedPoints->size() << std::endl;

    nonClumpedPoints->erase(std::remove_if(nonClumpedPoints->begin(),nonClumpedPoints->end(),clusterDetection::wasRemoved),nonClumpedPoints->end());

    std::cout << "points after  " << nonClumpedPoints->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ> pclROSObjectLists = big_objects + small_objects; //temp clouds


    sensor_msgs::PointCloud2 rosObjectList;
    toROSMsg(pclROSObjectLists,rosObjectList);
    return rosObjectList;
}
