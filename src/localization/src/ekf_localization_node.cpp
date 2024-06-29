#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "localization/ekf_localization.h"
using namespace std;
int main(int argc, char* argv[])
{
    Eigen::VectorXd alpha = Eigen::VectorXd::Zero(6);
    alpha(0) = 0.05;
    alpha(1) = 0.05;
    alpha(2) = 0.05;
    alpha(3) = 0.05;
    alpha(4) = 0.15;
    alpha(5) = 0.15;
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(3, 3);
    q(0, 0) = 0.01 * 0.01;
    q(1, 1) = 0.01 * 0.01;
    q(2, 2) = 0.01 * 0.01;
    EKF_Localization ekf_localization(alpha, q);
    ros::init(argc, argv, "ekf_localization_node");
    ros::NodeHandle node_handle;
    ros::Subscriber odometry_subscriber = node_handle.subscribe("odom", 1, &EKF_Localization::handle_odometry, & ekf_localization);
    ros::Subscriber landmarks_subscriber = node_handle.subscribe("landmarks",1, &EKF_Localization::handle_landmarks, & ekf_localization);
    ros::Subscriber observations_subscriber = node_handle.subscribe("observations", 1, &EKF_Localization::handle_observations, & ekf_localization);
    ros::Publisher estimated_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("estimated_odom", 1, true);
    sleep(1);
    double frequency = 20.0;
    ros::Rate timer(frequency);
    while (ros::ok())
    {
        ekf_localization.step(1.0 / frequency);
        estimated_odometry_publisher.publish(ekf_localization.estimated_odometry());
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}

