#ifndef EKF_LOCALIZATION_H
#define EKF_LOCALIZATION_H
#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "perception/Landmarks.h"
#include "perception/Observations.h"
class EKF_Localization
{
public:
    EKF_Localization(const Eigen::VectorXd &mu = Eigen::VectorXd::Zero(6),
                     const Eigen::MatrixXd &q = Eigen::MatrixXd::Zero(3, 3));
    virtual ~EKF_Localization();
    void handle_command(const geometry_msgs::Twist::ConstPtr &msg);
    void handle_odometry(const nav_msgs::Odometry::ConstPtr &msg);
    void handle_landmarks(const perception::Landmarks::ConstPtr &msg);
    void handle_observations(const perception::Observations::ConstPtr &msg);
    void step(const double &dt);
    nav_msgs::Odometry estimated_odometry(void) const;

protected:
    geometry_msgs::Twist _u;
    perception::Observations _z;
    Eigen::VectorXd _Mu;   
    Eigen::MatrixXd _Sigma;
    Eigen::VectorXd _alpha; 
    Eigen::MatrixXd _Q;     
    std::map<int, geometry_msgs::Point> _landmarks;
};
#endif /* EKF_LOCALIZATION_H */
