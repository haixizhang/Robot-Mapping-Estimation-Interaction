#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

geometry_msgs::Polygon waypoints;
bool waypoints_received = false;
nav_msgs::Odometry estimated_odom;

void handle_waypoints(const geometry_msgs::Polygon::ConstPtr& msg) {       
    waypoints = *msg;
    waypoints_received = true;
    std::cout << "received goals x: " << std::endl;
    return;
}

void handle_estimated_odom(const nav_msgs::Odometry::ConstPtr& msg) {
    estimated_odom = *msg;
    return;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "waypoints_publisher_node");
    ros::NodeHandle node_handle;
    ros::Subscriber waypoints_subscriber = node_handle.subscribe("executive/waypoints", 1, &handle_waypoints);
    ros::Subscriber estimated_odom_subscriber = node_handle.subscribe("estimated_odom", 1, &handle_estimated_odom);
    ros::Publisher goal_publisher = node_handle.advertise <geometry_msgs::Pose> ("goal", 1, true);
    sleep(1);
    
    geometry_msgs::Point32 cur_waypoint;
    int waypoint_idx = 0;
    int num_waypoints = 0;
    std::cout << "num_waypoints: " << num_waypoints << std::endl;
    double frequency = 10.0;
    ros::Rate timer(frequency);
    while(ros::ok()) {
        if(waypoints_received) {
            waypoint_idx = 0;
            num_waypoints = waypoints.points.size();
            cur_waypoint = waypoints.points[waypoint_idx];
            std::cout << "cur_waypoint x: " << cur_waypoint.x << std::endl;
            waypoints_received = false;
            geometry_msgs::Pose _goal;
            _goal.position.x = cur_waypoint.x;
            _goal.position.y = cur_waypoint.y;
            _goal.position.z = cur_waypoint.z;
            
            goal_publisher.publish(_goal);

        }
        double distance = sqrt(pow(cur_waypoint.x - estimated_odom.pose.pose.position.x, 2) + 
                                pow(cur_waypoint.y - estimated_odom.pose.pose.position.y, 2));
        std::cout << "distance to goal in goal publisher: " << distance << std::endl;
        if(distance <= cur_waypoint.z) {
            waypoint_idx += 1;
            std::cout << "waypoints reached!" << std::endl;
            if (waypoint_idx < num_waypoints) {
                std::cout << "publishing next waypoint!" << std::endl;
                cur_waypoint = waypoints.points[waypoint_idx];
                geometry_msgs::Pose _goal;
                _goal.position.x = cur_waypoint.x;
                _goal.position.y = cur_waypoint.y;
                _goal.position.z = cur_waypoint.z;
                goal_publisher.publish(_goal);
            }
        }
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}

