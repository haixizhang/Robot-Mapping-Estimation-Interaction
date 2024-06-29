#include "mapper/ocmap_wrapper.h"

OCMapWrapper::OCMapWrapper() : nodehandler(), ocmap_() {
    laser_scan_sub_ = nodehandler.subscribe<sensor_msgs::LaserScan>("scan", 10, &OCMapWrapper::laserScanCallback, this);
    odometry_sub_ = nodehandler.subscribe<nav_msgs::Odometry>("odom", 10, &OCMapWrapper::odometryCallback, this);
    map_pub_ = nodehandler.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
}

OCMapWrapper::~OCMapWrapper() {
}

void OCMapWrapper::spin() {
    ros::Rate rate(1); 
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

void OCMapWrapper::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	last_scan_ = scan;
}

void OCMapWrapper::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom) {
	last_pose_ = odom->pose.pose;

	if(last_scan_){
		ocmap_.update(last_pose_, *last_scan_);
		map_pub_.publish(ocmap_.ocmap);
		last_scan_.reset();
	}
}
