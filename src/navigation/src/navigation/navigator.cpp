#include <iostream>
#include "navigation/navigator.h"
#include <vector>
#include <cmath>
#include <algorithm>

// Constants for THRESHOLD values
const double THRESHOLD = 1e-6;
const double HIGH_COST = 10000;
const double VELOCITY_COST = 100;
const double OBSTACLE_PENALTY = 500;


bool row_col_in_map(const nav_msgs::OccupancyGrid &map, const int &row, const int &col)
{
    if ((row < 0) || (col < 0))
    {
        return false;
    }
    else if ((row >= map.info.width) || (col >= map.info.height))
    {
        return false;
    }
    else
    {
        return true;
    }
}

double
row_to_x(const nav_msgs::OccupancyGrid &map, const int &row, const double &discretization)
{
    return (discretization * (double)(-((int)(map.info.width) - 1) /
                                          2.0 + row));
}
double
col_to_y(const nav_msgs::OccupancyGrid &map, const int &col, const double &discretization)
{
    return (discretization * (double)(-((int)(map.info.height) - 1) /
                                          2.0 + col));
}
int x_to_row(const nav_msgs::OccupancyGrid &map, const double &x, const double &discretization)
{
    return round(x / discretization + (double)((map.info.width - 1) / 2));
}
int y_to_col(const nav_msgs::OccupancyGrid &map, const double &y, const double &discretization)
{
    return round(y / discretization + (double)((map.info.height - 1) / 2));
}

bool Navigator::
check_map(const double &x,
             const double &y,
             const double &radius,
             const double &THRESHOLD)
{
    double discretization = _ocmap.info.resolution;
    int row = x_to_row(_ocmap, x, discretization);
    int col = y_to_col(_ocmap, y, discretization);
    int offset = ceil(radius / discretization);
    for (int i = -offset; i < offset; i++)
    {
        double dx = row_to_x(_ocmap, row + i, discretization) - x;
        for (int j = -offset; j < offset; j++)
        {
            double dy = col_to_y(_ocmap, col + j, discretization) - y;
            if (row_col_in_map(_ocmap, row + i, col + j) && (sqrt(dx * dx + dy * dy) < radius))
            {
                int index = (row + i) * _ocmap.info.height + col + j;
                if (_ocmap.data[index] > THRESHOLD)
                {
                    return false;
                }
            }
        }
    }
    return true;
}

double quaternion_to_yaw(const geometry_msgs::Quaternion &quaternion)
{
    // since beta  = 0, -pi/2 < beta < pi/2
    double x = quaternion.x;
    double y = quaternion.y;
    double z = quaternion.z;
    double eta = quaternion.w;

    // alpha = atan2(r21, r11)
    double r11 = 2 * (pow(eta, 2) + pow(x, 2)) - 1;
    double r21 = 2 * (x * y + eta * z);
    double alpha = atan2(r21, r11);
    return alpha;
}

Navigator::Navigator(double dt, double pred_duration) :
                         _dt(dt),
                         _pred_duration(pred_duration),
                         _robot_radius(0.4){
    geometry_msgs::Pose init_goal;
    init_goal.position.z = 0.1;
    _goal = init_goal;
}

Navigator::~Navigator() {}

void Navigator::
handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    _ocmap = *msg;
}

void Navigator::
handle_odom(const nav_msgs::Odometry::ConstPtr &msg) {
    _odom = *msg;
}

void Navigator::
handle_goal(const geometry_msgs::Pose::ConstPtr &msg) {
    _goal = *msg;
}

void Navigator::
handle_cmd(const geometry_msgs::Twist::ConstPtr &msg) {
    _cmd = *msg;
}

geometry_msgs::Point32 Navigator::update_motion(const double x, const double y, const double theta, const double v, const double w, const double dt) {
    bool simple_motion = false;
    if (fabs(w) < THRESHOLD) {
        simple_motion = true;
    }
    float r = v / w;
    float theta_w_dt = theta + w * dt;
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);
    float sin_theta_w_dt = sin(theta_w_dt);
    float cos_theta_w_dt = cos(theta_w_dt);
    double x_update = 0;
    double y_update = 0;
    double theta_update = 0;
	  if (simple_motion) {
		  x_update = v * cos_theta * dt;
		  y_update = v * sin_theta * dt;
	  }
	  else {
		  x_update = -r * sin_theta + r * sin_theta_w_dt;
		  y_update = r * cos_theta - r * cos_theta_w_dt;
		  theta_update = w * dt;
	  }
    geometry_msgs::Point32 point;
    point.x = x + x_update;
    point.y = y + y_update;
    return point;
}

navigation::Trajectory Navigator::generate_trajectory(double v, double w) {
    navigation::Trajectory traj;
    int num_steps = (int) (_pred_duration / _dt);
    int i = 0;
    for (i = 1; i < num_steps + 1; i++) {
        double dt = i * _dt;
        double x = _odom.pose.pose.position.x;
        double y = _odom.pose.pose.position.y;
        double theta = quaternion_to_yaw(_odom.pose.pose.orientation);
        traj.trajectory.push_back(update_motion(x, y, theta, v, w, dt));
    }
    return traj;
}


// Function to sample linear velocities within a range
std::vector<double> sample_linear_velocities(double v_cur, int num_steps, double step, double min_v, double max_v) {
    std::vector<double> v_samples;
    for (int i = -num_steps; i <= num_steps+1; ++i) {
        double v = v_cur + i * step;
        if (v >= min_v && v <= max_v) {
            v_samples.push_back(v);
        }
    }
    return v_samples;
}

// Function to sample angular velocities within a range
std::vector<double> sample_angular_velocities(double w_cur, int num_steps, double step, double w_max) {
    std::vector<double> w_samples;
    for (int i = -num_steps; i <= num_steps; ++i) {
        double w = w_cur + i * step;
        if (std::abs(w) <= w_max) {
            w_samples.push_back(w);
        }
    }
    return w_samples;
}

// Updated sample_trajectories function
void Navigator::sample_trajectories() {
    _trajectories.trajectories.clear();
    double distance_to_goal = sqrt(pow(_goal.position.x - _odom.pose.pose.position.x, 2) + pow(_goal.position.y - _odom.pose.pose.position.y, 2));
    std::cout << "Distance to goal: " << distance_to_goal << std::endl;

    if (distance_to_goal < 0.1) {
        _next_cmd.linear.x = 0;
        _next_cmd.angular.z = 0;
        std::cout << "Reached! Waiting ..."<< std::endl;
        sleep(2);
        return;
    }

    double v_cur = _cmd.linear.x;
    double w_cur = _cmd.angular.z;

    // Sample linear velocities
    auto v_samples = sample_linear_velocities(v_cur, 3 , 0.05, -0.01, 0.3);

    // Sample angular velocities with varying step based on linear velocity
    double w_max = 0.8;
    double w_step = w_max / 4 - 0.1 * std::abs(v_cur);
    auto w_samples = sample_angular_velocities(w_cur, 4, w_step, 0.8);

    double lowest_cost = std::numeric_limits<double>::max();
    geometry_msgs::Twist best_action;

    // Iterate over all possible trajectories and compute costs
    for (double v : v_samples) {
        for (double w : w_samples) {
            navigation::Trajectory traj = generate_trajectory(v, w);
            _trajectories.trajectories.push_back(traj);

            double cost = compute_trajectory_cost(traj, v);

            if (cost < lowest_cost) {
                best_action.linear.x = v;
                best_action.angular.z = w;
                lowest_cost = cost;
            }
        }
    }
    std::cout<< "Best_action V: " << best_action.linear.x << ", Best_action W: " <<best_action.angular.z<<", Current Cost: " << lowest_cost <<std::endl;
    _next_cmd = best_action;
    lowest_cost = std::numeric_limits<double>::max();
}


// Updated compute_trajectory_cost function
double Navigator::compute_trajectory_cost(navigation::Trajectory &trajectory, double v) {
    double cost_sum = 0;

    // Check each point in the trajectory for collisions or proximity to obstacles
    for (const geometry_msgs::Point32 &point : trajectory.trajectory) {
        if (!check_map(point.x, point.y, _robot_radius, 0)) {
            cost_sum += HIGH_COST;
            //std::cout << "Obstacles"  << std::endl;
            return cost_sum; // Early return if there's an obstacle
        }
    }

    // Add a penalty for trajectories ending far from the goal
    geometry_msgs::Point32 end_point = trajectory.trajectory.back();
    double distance = sqrt(pow(_goal.position.x - end_point.x, 2) + pow(_goal.position.y - end_point.y, 2));
    cost_sum += distance;

    // Add a penalty for reverse velocity
    if (v <=0) {
        cost_sum += VELOCITY_COST;
    }

    return cost_sum;
}


navigation::Trajectories Navigator::collect_trajectories(void) {
    return _trajectories;
}

geometry_msgs::Twist Navigator::get_next_cmd(void) {
    return _next_cmd;
}

