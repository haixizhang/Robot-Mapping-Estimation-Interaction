#include <iostream>
#include "navigation/navigator.h"
#include "ros/ros.h"
#include "navigation/Trajectories.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[]) {
    Navigator navigator;
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle node_handle;
    ros::Subscriber estimated_odom_subscriber = node_handle.subscribe("estimated_odom", 1, &Navigator::handle_odom, &navigator);
    ros::Subscriber map_subscriber = node_handle.subscribe("map", 1, &Navigator::handle_map, &navigator);
    ros::Subscriber goal_subscriber = node_handle.subscribe("goal", 1, &Navigator::handle_goal, &navigator);
    ros::Subscriber cmd_subscriber = node_handle.subscribe("mobile_base/commands/velocity", 1, &Navigator::handle_cmd, &navigator);
    ros::Publisher sampled_trajectories_publisher = node_handle.advertise <navigation::Trajectories> ("sampled_trajectories", 1, true);
    ros::Publisher nav_cmd_publisher = node_handle.advertise <geometry_msgs::Twist> ("mobile_base/commands/velocity", 1, true);
    sleep(1);
    double frequency = 20.0;
    ros::Rate timer(frequency);
    while (ros::ok()) {
        navigator.sample_trajectories();
        sampled_trajectories_publisher.publish(navigator.collect_trajectories());
        nav_cmd_publisher.publish(navigator.get_next_cmd());
        ros::spinOnce();
        timer.sleep();
    }
}

