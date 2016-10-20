#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


#include "goat_lidar_merging/clusterDetection.h"



bool Message = false;
sensor_msgs::LaserScan scan_in;
//sensor_msgs::

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan){
  scan_in= *scan;
  Message = true;
}



int main(int argc, char** argv){
  ROS_INFO("test");
  ros::init(argc, argv, "merger");
  clusterDetection detector(5,10);
  
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/scan", 10, scanCallback);
  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("cloud", 10);
  tf::TransformListener listener_;
  laser_geometry::LaserProjection projector_;


  ros::Rate rate(100.0);

  while (node.ok()){
    if(Message){
      Message = false;
      if(!listener_.waitForTransform(scan_in.header.frame_id,"/world", scan_in.header.stamp 
        + ros::Duration().fromSec(scan_in.ranges.size()*scan_in.time_increment),ros::Duration(1.0))){
          ROS_INFO("Gave up");

        continue;
      }
      sensor_msgs::PointCloud cloud;
      sensor_msgs::PointCloud2 cloud2;
      
      projector_.transformLaserScanToPointCloud("/world",scan_in,
        cloud,listener_);
      sensor_msgs::convertPointCloudToPointCloud2(cloud,cloud2);
      cloud_pub.publish(cloud2);
      detector.cluster(cloud2);



    }
    ros:: spinOnce();
    rate.sleep();


  }
  return 0;
};
