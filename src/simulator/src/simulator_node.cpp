#include <iostream>
#include "ros/ros.h"
#include "simulator/simulator.h"
using namespace std;
int main( int argc, char* argv[] ){
    Simulator simulator;
    ros::init(argc, argv, "simulator_node");
    ros::NodeHandle node_handle;
    //ros::Subscriber command_subscriber = node_handle.subscribe("cmd_vel_mux/input/navi", 1, &Simulator::handle_command, &simulator);
    ros::Subscriber command_subscriber = node_handle.subscribe("mobile_base/commands/velocity", 1, &Simulator::handle_command, &simulator);
    ros::Publisher odometry_publisher = node_handle.advertise<nav_msgs::
                                                                  Odometry>("odom", 1, true);
    ros::Publisher scan_publisher = node_handle.advertise<sensor_msgs::
                                                              LaserScan>("scan", 1, true);   
    ros::Subscriber landmarks_subscriber = node_handle.subscribe("landmarks",
                                                                 1, &Simulator::handle_landmarks, &simulator);
    ros::Subscriber obstacles_subscriber = node_handle.subscribe("obstacles",
                                                                 1, &Simulator::handle_obstacles, &simulator);
    ros::Publisher simulated_obstacles_publisher = node_handle.advertise<geometry_msgs::Polygon>("simulated_obstacles", 1, true);
    ros::Publisher observed_landmarks_publisher = node_handle.advertise<perception::Landmarks>("observed_landmarks", 1, true);
    ros::Publisher observations_publisher = node_handle.advertise<perception::Observations>("observations", 1, true);
    ros::Subscriber map_subscriber = node_handle.subscribe("map", 1, &Simulator::handle_map, &simulator);


    sleep(1);
    double frequency = 10.0;
    ros::Rate timer(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        simulator.step(1.0 / frequency);
        simulator.update_observations();
        simulator.update_observed_landmarks();
        odometry_publisher.publish( simulator.odometry_msg() );
        observations_publisher.publish( simulator.observations() );
        observed_landmarks_publisher.publish( simulator.observed_landmarks() );
        scan_publisher.publish( simulator.scan_msg() );
        simulated_obstacles_publisher.publish( simulator.simulated_obstacles_msg());
        timer.sleep();
    }
    return 0;
}


