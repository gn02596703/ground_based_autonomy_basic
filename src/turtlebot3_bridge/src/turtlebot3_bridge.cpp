#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
sensor_msgs::PointCloud2 registered_scan;
laser_geometry::LaserProjection projector_;

tf::Transform transform;

double odomTime = 0;
float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;


/**
 * This program manipulate re-map the message between ground autonomy and turtlebot3 to
 * let turtlebot3 using the ground autonomy function for navigation.
 *
 * Main work includes
 *   - convert /scan (from turtlebot, LaserScan, local frame) 
 *      to /registered_scan (to ground_autonomy, PointCoud2, global frame) 
 *   
 *   - convert /cmd_vel (from ground_autonomy, TwistStamped) 
 *      to /cmd_vel (to turtlebot, Twist)
 */

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odomTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  transform.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  transform.setOrigin(tf::Vector3(vehicleX, vehicleY, vehicleZ));

}

void laserCloudHandler(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan_in, cloud);   

    laserCloud->clear();
    pcl::fromROSMsg(cloud, *laserCloud);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "turtlebot3_bridge");

  ros::NodeHandle n;

  ros::Subscriber sub_Odometry = n.subscribe<nav_msgs::Odometry>("/odom", 5, odometryHandler);
  ros::Subscriber sub_LaserScan = n.subscribe<sensor_msgs::LaserScan>("/scan", 5, laserCloudHandler);
  
  ros::Publisher pub_RegisteredScan = n.advertise<sensor_msgs::PointCloud2>("/registered_scan", 5);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    
    sensor_msgs::PointCloud2 registedScan_out;
    sensor_msgs::PointCloud2 registedScan_base_link;
    pcl::toROSMsg(*laserCloud, registedScan_base_link);
    pcl_ros::transformPointCloud(std::string("/map"), transform, registedScan_base_link, registedScan_out);
    registedScan_out.header.stamp = ros::Time().fromSec(odomTime);
    //registedScan_out.header.frame_id = "base_link";
    pub_RegisteredScan.publish(registedScan_out);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}