#include <Eigen/Dense>
#include "localization/ekf_localization.h"
#include "geometry_msgs/Point.h"

const double Threshold = 1e-6;

geometry_msgs::Quaternion
yaw_to_quaternion(const double &yaw)
{
    geometry_msgs::Quaternion quaternion;
    quaternion.w = cos(yaw / 2.0);
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(yaw / 2.0);
    return quaternion;
}
EKF_Localization::
    EKF_Localization(const Eigen::VectorXd &alpha,
                     const Eigen::MatrixXd &q) : _u(),
                                                 _landmarks(),
                                                 _z(),
                                                 _Mu(Eigen::VectorXd::Zero(3)),
                                                 _Sigma(Eigen::MatrixXd::Identity(3, 3)),
                                                 _alpha(alpha),
                                                 _Q(q)
{
}
EKF_Localization::
    ~EKF_Localization()
{
}
void EKF_Localization::
    handle_command(const geometry_msgs::Twist::ConstPtr &msg)
{
    _u = *msg;
    return;
}
void EKF_Localization::
    handle_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{
    _u = msg->twist.twist;
    return;
}
// Updates landmark information based on received messages
void EKF_Localization::
    handle_landmarks(const perception::Landmarks::ConstPtr &msg)
{
    for (unsigned int i = 0; i < msg->landmarks.size(); i++)
    {
        std::map<int, geometry_msgs::Point>::iterator it_landmark = _landmarks.find(
            msg->landmarks[i].signature);
        if (it_landmark != _landmarks.end())
        {
            it_landmark->second = msg->landmarks[i].pos;
        }
        else
        {
            _landmarks.insert(std::pair<int, geometry_msgs::Point>(msg->landmarks[i].signature, msg->landmarks[i].pos));
        }
    }
    return;
}
void EKF_Localization::
    handle_observations(const perception::Observations::ConstPtr &msg)
{
    _z = *msg;
    return;
}


// Main step function for the EKF (performs prediction and update steps)
void EKF_Localization::step(const double &dt)
{
    // Initialize Jacobian matrices for the motion model
    Eigen::MatrixXd G_t = Eigen::MatrixXd::Zero(3, 3); // State transition Jacobian
    Eigen::MatrixXd V_t = Eigen::MatrixXd::Zero(3, 2); // Control input Jacobian
    Eigen::MatrixXd M_t = Eigen::MatrixXd::Zero(2, 2); // Motion noise covariance
    
    Eigen::VectorXd motion_update = Eigen::VectorXd::Zero(3); // Motion update vector
    
    float u = _u.linear.x; // Linear velocity
    float w = _u.angular.z; // Angular velocity
    
    // Determine if the motion is simple (straight line)
    bool simple_motion = fabs(w) < Threshold;

    float r = u / w; // Radius of curvature
    
    float x = _Mu(0); // Current x position
    float y = _Mu(1); // Current y position
    float theta = _Mu(2); // Current orientation (theta)
    
    float theta_w_dt = theta + w * dt; // Orientation after dt
    
    // Sine and cosine values for the current and future orientation
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);
    float sin_theta_w_dt = sin(theta_w_dt);
    float cos_theta_w_dt = cos(theta_w_dt);

    // Set Jacobians and motion update based on whether motion is simple or not
    if (simple_motion) // If it's simple motion (straight line)
    {
        // Jacobian for state transition
        G_t << 1, 0, -u * sin_theta * dt,
              0, 1,  u * cos_theta * dt,
              0, 0, 1;

        // Jacobian for control input
        V_t << cos_theta * dt, 0,
              sin_theta * dt, 0,
              0, 0;

        // Motion update (linear motion)
        motion_update(0) = u * cos_theta * dt;
        motion_update(1) = u * sin_theta * dt;
    }
    else // If it's not simple motion (arc motion)
    {
        // Jacobian for state transition
        G_t << 1, 0, -r * sin_theta + r * sin_theta_w_dt,
              0, 1,  r * cos_theta - r * cos_theta_w_dt,
              0, 0,  1;

        // Jacobian for control input
        V_t << (-sin_theta + sin_theta_w_dt) / w, 
              u * (sin_theta - sin_theta_w_dt) / (w * w) + u * cos_theta_w_dt * dt / w,
              (cos_theta - cos_theta_w_dt) / w,
              -u * (cos_theta - cos_theta_w_dt) / (w * w) + u * sin_theta_w_dt * dt / w,
              0, dt;

        // Motion update (arc motion)
        motion_update(0) = -r * sin_theta + r * sin_theta_w_dt;
        motion_update(1) = r * cos_theta - r * cos_theta_w_dt;
        motion_update(2) = w * dt; // Angular motion
    }
    
    // Motion noise covariance
    M_t << _alpha(0) * pow(u, 2) + _alpha(1) * pow(w, 2), 
          0,
          0,
          _alpha(2) * pow(u, 2) + _alpha(3) * pow(w, 2);

    // Update the state estimate and covariance
    _Mu = _Mu + motion_update; // Updated state estimate
    _Sigma = G_t * _Sigma * G_t.transpose() + V_t * M_t * V_t.transpose(); // Updated covariance
    
    // Predicted state variables
    float x_pred = _Mu(0);
    float y_pred = _Mu(1);
    float theta_pred = _Mu(2);


    for (unsigned int i = 0; i < _z.observations.size(); i++)
    {
        Eigen::VectorXd observation = Eigen::VectorXd::Zero(3, 1);
        Eigen::MatrixXd H_t_i = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd S_t_i = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd K_t_i = Eigen::MatrixXd::Zero(3, 3);

        observation << _z.observations[i].range, 
                        _z.observations[i].bearing,
                        _z.observations[i].signature;
        // check if the observation belong to pre-defined landmarks or not
        if (_landmarks.count(observation(2)) == 0) {
            continue;
        }
        float landmark_x = _landmarks[observation(2)].x;
        float landmark_y = _landmarks[observation(2)].y;
        float q = pow(landmark_x - x_pred, 2) + pow(landmark_y - y_pred, 2);
        float sqrt_q = sqrt(q);
        Eigen::VectorXd observation_pred = Eigen::VectorXd::Zero(3, 1);
        observation_pred << sqrt(q),
                            atan2(landmark_y - y_pred, landmark_x - x_pred) - theta_pred,
                            observation(2);
        observation_pred(1) = atan2(sin(observation_pred(1)), cos(observation_pred(1)));
        H_t_i << -(landmark_x - x_pred) / sqrt_q, -(landmark_y - y_pred) / sqrt_q, 0,
                    (landmark_y - y_pred) / q, -(landmark_x - x_pred) / q, -1,
                    0, 0, 0;
        S_t_i = H_t_i * _Sigma * H_t_i.transpose() + _Q;
        K_t_i = _Sigma * H_t_i.transpose() * S_t_i.inverse();
        _Mu = _Mu + K_t_i * (observation - observation_pred);
        _Sigma = (Eigen::MatrixXd::Identity(3, 3) - K_t_i * H_t_i) * _Sigma;
    }
    _z.observations.clear();
    return;
}
nav_msgs::Odometry
EKF_Localization::
    estimated_odometry(void) const
{
    nav_msgs::Odometry msg;
    msg.pose.pose.position.x = _Mu(0);
    msg.pose.pose.position.y = _Mu(1);
    msg.pose.pose.orientation = yaw_to_quaternion(_Mu(2));
    return msg;
}

