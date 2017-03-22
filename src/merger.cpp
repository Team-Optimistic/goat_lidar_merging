#pragma GCC diagnostic ignored "-Wignored-attributes"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include "goat_lidar_merging/clusterDetection.h"

bool Message = false;
sensor_msgs::LaserScan scan_in;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan)
{
  scan_in = *scan;
  scan_in.range_min = 0.42;
  Message = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "merger");
  clusterDetection detector(15,50,125,280);

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("lidar/scan", 10, scanCallback);
  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("goat/cloud", 100);
  ros::Publisher small_objects_pub = node.advertise<sensor_msgs::PointCloud2>("goat/small_objects", 10);
  ros::Publisher big_objects_pub = node.advertise<sensor_msgs::PointCloud2>("goat/big_objects", 10);

  tf::TransformListener listener_;
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud cloud;
  sensor_msgs::PointCloud2 cloud2;
  sensor_msgs::PointCloud2 objectsCloud;

  ros::Rate rate(500.0);
  int seen_messages = 0;
  while (node.ok())
  {
    if(Message)
    {
      Message = false;
      try
      {
        if (!listener_.waitForTransform(scan_in.header.frame_id,"/field", scan_in.header.stamp +
          ros::Duration().fromSec(scan_in.ranges.size()*scan_in.time_increment),ros::Duration(1.0)))
        {
          ROS_INFO("Gave up");
          continue;
        }
        

        projector_.transformLaserScanToPointCloud("/field",scan_in, cloud,listener_);
        sensor_msgs::convertPointCloudToPointCloud2(cloud,cloud2);
        cloud2.header.frame_id = "/field";
        cloud_pub.publish(cloud2);
        detector.add_Cloud(cloud2);
        seen_messages++;
        ROS_INFO("seen messages %d",seen_messages);
      }
      catch (const tf2::ExtrapolationException& e)
      {
        ROS_INFO("Need to see the past");
      }
      catch (const tf2::ConnectivityException& e)
      {
        ROS_INFO("No recent connection between base_link and odom");
      }
    }
    if(seen_messages>=4){
      detector.cluster();
      objectsCloud = detector.get_small_objects();
      objectsCloud.header = cloud2.header;
      small_objects_pub.publish(objectsCloud);

      objectsCloud = detector.get_big_objects();
      objectsCloud.header = cloud2.header;
      big_objects_pub.publish(objectsCloud);
      seen_messages = 0;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
