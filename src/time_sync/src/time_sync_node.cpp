#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include "laser_registration.h"

LaserRegistration laser_registration(2);

bool flag = true;;

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(msg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    laser_registration.setImuInfo(msg->header.stamp.toSec(), roll, pitch, yaw, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI> laser;
    pcl::fromROSMsg(*msg, laser);

    pcl::io::savePCDFile("laser.pcd", laser);

    laser_registration.setPointCloud(0, laser);
    laser_registration.setLaserTimestamp(0, msg->header.stamp.toSec());
    laser_registration.setPointCloud(1, laser);
    laser_registration.setLaserTimestamp(1, msg->header.stamp.toSec());

    pcl::PointCloud<pcl::PointXYZI> total_laser;
    laser_registration.getFinalLaser(total_laser);

    std::cout << total_laser.points.size() << std::endl;
    pcl::io::savePCDFile("time_sync.pcd", total_laser);

    flag = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "registration");
    ros::NodeHandle nh;
    
    laser_registration.setLaserMatrix(0, Eigen::Matrix4f::Identity());
    laser_registration.setLaserMatrix(1, Eigen::Matrix4f::Identity());

    ros::Subscriber sub_laser = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_200", 100, laserCallback);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 100, imuCallback);

    while(flag)
    {
        ros::spinOnce();
    }

    return 0;
}

