#include <iostream>
#include "mapper/ocmap.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

sensor_msgs::LaserScan scan;
nav_msgs::Odometry odom;
nav_msgs::OccupancyGrid oc_grid;
bool scan_received = false;
bool odom_received = false;

void save_laser_scan(const sensor_msgs::LaserScan::ConstPtr &msg) {
    scan = *msg;
    scan_received = true;
}

void save_odom(const nav_msgs::Odometry::ConstPtr &msg) {
    odom = *msg;
    odom_received = true;
}

int main(int argc, char *argv[])
{
    OCMap ocmap;
    ros::init(argc, argv, "oc_grid_publisher_node");
    ros::NodeHandle node_handle;
    ros::Subscriber laser_scan_subscriber = node_handle.subscribe("scan", 1, save_laser_scan);
    ros::Subscriber odm_subscriber = node_handle.subscribe("estimated_odom", 1, save_odom);
    ros::Publisher oc_map_publisher = node_handle.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

    sleep(2);

    double frequency = 20.0;
    ros::Rate timer(frequency);
    while(ros::ok()) {
        if (scan_received && odom_received) {
            oc_grid = ocmap.update(odom.pose.pose, scan);
            oc_map_publisher.publish(oc_grid);
        }
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}
