#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "navigation/Trajectory.h"
#include "navigation/Trajectories.h"

class Navigator {
public:
    Navigator(double dt=0.1, double pred_duration=2);
    virtual ~Navigator();
    void handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void handle_odom(const nav_msgs::Odometry::ConstPtr &msg);
    void handle_goal(const geometry_msgs::Pose::ConstPtr &msg);
    void handle_cmd(const geometry_msgs::Twist::ConstPtr &msg);
    void sample_trajectories(); //generate trajectories and calcualte cost
    navigation::Trajectories collect_trajectories(void);
    geometry_msgs::Twist get_next_cmd(void);

private:
    double _dt;
    double _robot_radius;
    double _pred_duration;
    navigation::Trajectory generate_trajectory(const double v, const double w);
    geometry_msgs::Point32 update_motion(const double x, const double y, const double theta, const double v, const double w, const double dt);
    bool check_map(const double &x,const double &y,const double &radius,const double &threshold);
    double compute_trajectory_cost(navigation::Trajectory &trajectory, double v);
    double compute_goal_distance(navigation::Trajectory &trajectory);
    nav_msgs::Odometry _odom;
    nav_msgs::OccupancyGrid _ocmap;
    geometry_msgs::Pose _goal;
    geometry_msgs::Twist _cmd;
    geometry_msgs::Twist _next_cmd;
    navigation::Trajectories _trajectories;
};

#endif /* NAVIGATOR_H */

