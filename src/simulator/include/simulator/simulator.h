#ifndef SIMULATOR_H
#define SIMULATOR_H
#include <Eigen/Dense>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "perception/Landmarks.h"
#include "perception/Observations.h"
class Simulator
{
public:
    Simulator();
    virtual ~Simulator();
    void step(const double &dt);
    void update_observed_landmarks(void);
    void update_observations(void);
    void handle_command(const geometry_msgs::Twist::ConstPtr &msg);
    void handle_landmarks(const perception::Landmarks::ConstPtr &msg);
    void handle_obstacles(const geometry_msgs::Polygon::ConstPtr &msg);
    void handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void handle_estimated_odom(const nav_msgs::Odometry::ConstPtr &msg);
    bool check_map(const double &x,
                    const double &y,
                    const double &radius,
                    const double &threshold);
    nav_msgs::Odometry odometry_msg(void) const;
    sensor_msgs::LaserScan scan_msg(void) const;
    geometry_msgs::Polygon simulated_obstacles_msg(void) const;
    perception::Landmarks &landmarks(void) { return _landmarks; };
    perception::Landmarks &observed_landmarks(void) { return _observed_landmarks; };
    perception::Observations &observations(void) { return _observations; };
    geometry_msgs::Polygon &obstacles(void) { return _obstacles; };
  
protected:
    Eigen::Vector3d _estimated_odom;
    Eigen::Vector3d _x;
    Eigen::Vector3d _u;
    double _alpha1;
    double _alpha2;
    double _alpha3;
    double _alpha4;
    double _alpha5;
    double _alpha6;
    double _range_noise;
    double _bearing_noise;
    double _t;
    unsigned _num_scan_angles;
    double _scan_min_range;
    double _scan_max_range;
    double _scan_min_angle;
    double _scan_max_angle;
    perception::Landmarks _landmarks;
    perception::Landmarks _observed_landmarks;
    perception::Observations _observations;
    geometry_msgs::Polygon _obstacles;
    nav_msgs::OccupancyGrid _ocmap;
    double _observationMaxRange;
    double _observationMaxAngle;
    
};
#endif /* SIMULATOR_H */
