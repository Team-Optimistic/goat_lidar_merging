#pragma GCC diagnostic ignored "-Wignored-attributes"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <cmath>

#include "goat_lidar_merging/clusterDetection.h"

constexpr int clusterMaxDistance = 100;
constexpr int clusterRecord = 50;
bool message = false;
sensor_msgs::LaserScan scan_in;
sensor_msgs::PointCloud robot;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan)
{
  scan_in = *scan;
  scan_in.range_min = 0.42;
  message = true;
}

void makeRobotCloud()
{
  robot.header.frame_id = "/base_link";
  geometry_msgs::Point32 point;
  point.z = 0.0001;

  constexpr int chassisLength = int(12 * 25.4), clawRadius = int((22.5 * 25.4) / 25);
  constexpr int pointsForHalfChassisLength = chassisLength / (clusterMaxDistance / 2);
  constexpr int pointsFHCL = chassisLength / (clusterRecord / 2);

  //Draw square
  for (int i = -1 * pointsForHalfChassisLength; i <= pointsForHalfChassisLength; i++)
  {
    for (int j = -1 * pointsForHalfChassisLength; j <= pointsForHalfChassisLength; j++)
    {
      point.x = float(i * (chassisLength / pointsForHalfChassisLength)) / 1000;
      point.y = float(j * (chassisLength / pointsForHalfChassisLength)) / 1000;
      robot.points.push_back(point);
    }
  }

  //Draw semicircle
  for (int i = -1 * (pointsFHCL * 1.5); i <= pointsFHCL * 1.5; i++)
  {
    for (int j = 0; j <= int(cos(float(i)/(4.45*3.5)) * clawRadius); j++)
    {
      point.y = float(i * (chassisLength / pointsFHCL)) / 1000;
      point.x = float(j * (chassisLength / pointsFHCL)) / 1000 + (0.012192 + (12*25.4)/1000.0);
      robot.points.push_back(point);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "merger");
  clusterDetection detector(15, clusterMaxDistance, 150, 280);

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("lidar/scan", 10, scanCallback);
  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("goat/cloudz", 100),
  small_objects_pub = node.advertise<sensor_msgs::PointCloud2>("goat/small_objects", 10),
  big_objects_pub = node.advertise<sensor_msgs::PointCloud2>("goat/big_objects", 10);

  tf::TransformListener listener_;
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud cloud;
  sensor_msgs::PointCloud2 cloud2;
  sensor_msgs::PointCloud2 objectsCloud;
  makeRobotCloud();
  ros::Rate rate(500.0);
  int seen_messages = 6;
  while (node.ok())
  {
    try
    {
      if(message)
      {
        message = false;

        if (!listener_.waitForTransform(scan_in.header.frame_id,"/field", scan_in.header.stamp +
          ros::Duration().fromSec(scan_in.ranges.size()*scan_in.time_increment),ros::Duration(1.0)))
        {
          ROS_INFO("Gave up");
          continue;
        }

        constexpr int seperate_clouds = 120;
        const int points_per_cloud = scan_in.ranges.size()/seperate_clouds;
        int points_this_scan=0;
        ROS_INFO("Scan Time %1.2f",(1000.0* scan_in.ranges.size())*scan_in.time_increment);
        ROS_INFO("time increment  %1.6f",scan_in.time_increment);

        for (int i = 0; i < seperate_clouds; i++)
        {
          const int start_index = i * points_per_cloud;
          int end_index;
          if (start_index + points_per_cloud > scan_in.ranges.size())
          	end_index = scan_in.ranges.size();
          else
          	end_index = start_index + points_per_cloud;

          sensor_msgs::LaserScan temp;
          temp.header = scan_in.header;
          temp.header.stamp = scan_in.header.stamp +
          ros::Duration().fromSec(points_per_cloud*scan_in.time_increment);

          temp.time_increment = scan_in.time_increment;
          temp.range_min = scan_in.range_min;
          temp.range_max = scan_in.range_max;
          temp.angle_min = scan_in.angle_min + start_index * (M_PI/180.0);
          temp.angle_max = scan_in.angle_min + end_index * (M_PI/180.0);
          temp.angle_increment = scan_in.angle_increment;
          temp.ranges.assign(scan_in.ranges.begin()+start_index,
            scan_in.ranges.begin() + end_index);
          temp.intensities.assign(scan_in.intensities.begin() + start_index,
            scan_in.intensities.begin() + end_index);
          points_this_scan += temp.ranges.size();

          projector_.transformLaserScanToPointCloud("/field", temp, cloud, listener_);
          sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
          cloud2.header.frame_id = "/field";
          detector.add_Cloud(cloud2);
        }

        seen_messages++;
       // ROS_INFO("seen Points %d",points_this_scan);
        points_this_scan=0;//debuggin variable

      }

      if (seen_messages >= 4)
      {

        listener_.transformPointCloud("/field",ros::Time(0),robot,"/base_link",cloud);
        sensor_msgs::convertPointCloudToPointCloud2(cloud,cloud2);
        detector.add_Cloud(cloud2);

        detector.cluster();

        objectsCloud = detector.get_small_objects();
        objectsCloud.header = cloud2.header;
        small_objects_pub.publish(objectsCloud);

        objectsCloud = detector.get_big_objects();
        objectsCloud.header = cloud2.header;
        big_objects_pub.publish(objectsCloud);

        cloud2 = detector.get_cloud();
        cloud2.header = objectsCloud.header;
        cloud_pub.publish(cloud2);

        seen_messages = 0;
      }
    }
    catch (const tf2::ExtrapolationException& e)
    {
      ROS_INFO("Need to see the past");
    }
    catch (const tf2::ConnectivityException& e)
    {
      ROS_INFO("No recent connection between base_link and odom");
    }
    catch (const tf2::LookupException& e){
      ROS_INFO("Don't see required frames. Start static transforms.");
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
