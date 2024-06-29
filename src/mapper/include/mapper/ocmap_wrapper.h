#ifndef OCMAP_WRAPPER_H
#define OCMAP_WRAPPER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "mapper/ocmap.h"

class OCMapWrapper {
public:
    OCMapWrapper();
    virtual ~OCMapWrapper();

    void spin();

private:
    ros::NodeHandle nodehandler;
    ros::Subscriber laser_scan_sub_;
    ros::Subscriber odometry_sub_;
    ros::Publisher map_pub_;

    OCMap ocmap_;

    sensor_msgs::LaserScan::ConstPtr last_scan_;
    geometry_msgs::Pose last_pose_;

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom);
};

#endif /* OCMAP_WRAPPER_H */
