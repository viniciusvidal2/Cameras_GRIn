#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

ros::Publisher ptc_pub;
laser_geometry::LaserProjection projector;

void s2ptc_callback(const sensor_msgs::LaserScanConstPtr& laser_readings)
{
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = laser_readings->header.frame_id;
    cloud.header.stamp    = laser_readings->header.stamp;
    projector.projectLaser(*laser_readings, cloud);
    ptc_pub.publish(cloud);
    ROS_INFO("Quantos pontos? %zu", cloud.data.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser2ptc");
    ros::NodeHandle nh;

    ros::Subscriber laser_sub = nh.subscribe("/scan", 1000, s2ptc_callback);
    ptc_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan2/PointCloud", 1000);

    ros::spin();

    return 0;
}
