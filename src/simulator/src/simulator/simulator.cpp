#include <iostream>
#include <cmath>
#include "simulator/simulator.h"
using namespace std;

/**
 * x(0) = x position
 * x(1) = y position
 * x(2) = heading
 * u(0) = linear velocity (v)
 * u(1) = angular_velocity (w)
 */

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

bool Simulator::check_map(const double &x,
             const double &y,
             const double &radius,
             const double &threshold)
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
                if (_ocmap.data[index] > threshold)
                {
                    return false;
                }
            }
        }
    }
    return true;
}

geometry_msgs::Quaternion
yaw_to_quaternion(const double &yaw)
{
    geometry_msgs::Quaternion quaternion;
    // your implementation here
    quaternion.x = 0;
    quaternion.y = 0;
    float sign = 0;
    if (sin(yaw) >= 0)
    {
        sign = 1.0;
    }
    else
    {
        sign = -1.0;
    }
    quaternion.z = 0.5 * sign * sqrt(1 - cos(yaw) - cos(yaw) + 1);
    quaternion.w = 0.5 * sqrt(cos(yaw) + cos(yaw) + 1 + 1);
    std::cout << "yaw: " << yaw << ", z: " << quaternion.z << ", w: " << quaternion.w << std::endl;
    return quaternion;
}
Simulator::
    Simulator() : _x(0.0, 0.0, 0.0),
                  _u(0.0, 0.0, 0.0),
                  _num_scan_angles(128),
                  _scan_min_angle(-M_PI / 4.0),
                  _scan_max_angle(M_PI / 4.0),
                  _scan_min_range(0.1),
                  _scan_max_range(5.0)
{
    std::cout << "Simulator Created" << std::endl;
}
Simulator::
    ~Simulator()
{
}

void Simulator::
    step(const double &dt)
{
    _x(0) = _x(0) + _u(0) * cos(_x(2)) * dt;
    _x(1) = _x(1) + _u(0) * sin(_x(2)) * dt;
    _x(2) = _x(2) + _u(1) * dt;
    if (_x(2) < -M_PI){ _x(2) = _x(2) + 2 * M_PI;}
    else if (_x(2) > M_PI){ _x(2) = _x(2) - 2 * M_PI;}
    
    return;
}

float sample( float noise){	
	return -noise + 2.0 * noise * (rand() % 100) / 99.0;
}


nav_msgs::Odometry
Simulator::
    odometry_msg(void) const
{
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = _x(0);
    msg.pose.pose.position.y = _x(1);
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = yaw_to_quaternion(_x(2));
    msg.twist.twist.linear.x = _u(0);
    msg.twist.twist.angular.z = _u(1);
    return msg;
}

void Simulator::
    handle_command(const geometry_msgs::Twist::ConstPtr &msg)
{
    _u(0) = msg->linear.x;
    _u(1) = msg->angular.z;
    return;
}
void Simulator::
    handle_obstacles(const geometry_msgs::Polygon::ConstPtr &msg)
{
    _obstacles = *msg;
    return;
}
void Simulator::handle_landmarks(const perception::Landmarks::ConstPtr &msg) {
    _landmarks = *msg;
    return;    
}

void Simulator::handle_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    _ocmap = *msg;
    return;
}

void update_observations(void){
    return;
}

void Simulator::update_observed_landmarks(void) {
    _observed_landmarks.landmarks.clear();
    for (perception::Landmark landmark : _landmarks.landmarks) {
        double range = sqrt(pow(_x[0] - landmark.pos.x, 2) + pow(_x[1] - landmark.pos.y, 2));
        double bearing = atan2(landmark.pos.y - _x[1], landmark.pos.x - _x[0]) - _x[2];
        int signature = landmark.signature;
        if (Simulator::check_map(landmark.pos.x, landmark.pos.y, 0.5, -0.2)) {
            if (range > _scan_min_angle && range < _scan_max_range &&
                bearing > _scan_min_angle && bearing < _scan_max_angle) {
                    _observed_landmarks.landmarks.push_back(landmark);
            }
        }
    }
    return;
}

void Simulator::update_observations(void) {
    _observations.observations.clear();
    for (perception::Landmark landmark : _landmarks.landmarks) {
        double range = sqrt(pow(_x[0] - landmark.pos.x, 2) + pow(_x[1] - landmark.pos.y, 2));
        double bearing = atan2(landmark.pos.y - _x[1], landmark.pos.x - _x[0]) - _x[2];
        int signature = landmark.signature;
        for (geometry_msgs::Point32 obstacle : _obstacles.points) {
            float obstacle_x = obstacle.x;
            float obstacle_y = obstacle.y;
        }
        if (Simulator::check_map(landmark.pos.x, landmark.pos.y, 0.5, -0.2)) {
            if (range > _scan_min_angle && range < _scan_max_range &&
                bearing > _scan_min_angle && bearing < _scan_max_angle) {
                    perception::Observation observation;
                    observation.range = range;
                    observation.bearing = bearing;
                    observation.signature = signature;
                    _observations.observations.push_back(observation);
            }
        }
    }
    return;
}

sensor_msgs::LaserScan
Simulator::
    scan_msg(void) const
{
    sensor_msgs::LaserScan msg;
    msg.angle_min = -M_PI / 4.0;
    msg.angle_max = M_PI / 4.0;
    msg.angle_increment = (msg.angle_max - msg.angle_min) / (double)(_num_scan_angles - 1);
    msg.range_min = 0.1;
    msg.range_max = 5.0;

    vector<double> scan_angles(_num_scan_angles);
    for (unsigned int i = 0; i < _num_scan_angles; i++)
    {
        scan_angles[i] = msg.angle_min + (double)(i)*msg.angle_increment;
    }
    vector<double> obstacle_angles(_obstacles.points.size());
    vector<double> obstacle_distances(_obstacles.points.size());
    vector<double> obstacle_phimaxs(_obstacles.points.size());
    for (unsigned int i = 0; i < _obstacles.points.size(); i++)
    {
        obstacle_angles[i] = atan2(_obstacles.points[i].y - _x(1),
                                   _obstacles.points[i].x - _x(0)) -
                             _x(2);
        if (obstacle_angles[i] < -M_PI)
        {
            obstacle_angles[i] += 2.0 * M_PI;
        }
        else if (obstacle_angles[i] > M_PI)
        {
            obstacle_angles[i] -= 2.0 * M_PI;
        }
        obstacle_distances[i] = sqrt(pow(_obstacles.points[i].x - _x(0), 2.0) +
                                     pow(_obstacles.points[i].y - _x(1), 2.0));
        if (obstacle_distances[i] < _obstacles.points[i].z)
        {
            return msg;
        }
        obstacle_phimaxs[i] = atan2(_obstacles.points[i].z, sqrt(pow(obstacle_distances[i], 2.0) -
                                                                 pow(_obstacles.points[i].z, 2.0)));
    }
    for (unsigned int i = 0; i < scan_angles.size(); i++)
    {
        double min_range = msg.range_max;
        for (unsigned int j = 0; j < _obstacles.points.size(); j++)
        {
            if (obstacle_distances[j] < (msg.range_max + _obstacles.points[j].z))
            {
                if ((scan_angles[i] > (obstacle_angles[j] - obstacle_phimaxs[j])) && (scan_angles[i] < (obstacle_angles[j] +
                                                                                                        obstacle_phimaxs[j])))
                {
                    double phi = scan_angles[i] - obstacle_angles[j];
                    double a = 1.0 + pow(tan(phi), 2.0);
                    double b = -2.0 * obstacle_distances[j];
                    double c = pow(obstacle_distances[j], 2.0) - pow(_obstacles.points[j].z, 2.0);
                    // compute candidate intersection points
                    double x1 = (-b + sqrt(pow(b, 2.0) - 4 * a * c)) / (2.0 * a);
                    double y1 = tan(phi) * x1;
                    double d1squared = pow(x1, 2.0) + pow(y1, 2.0);
                    double x2 = (-b - sqrt(pow(b, 2.0) - 4 * a * c)) / (2.0 * a);
                    double y2 = tan(phi) * x2;
                    double d2squared = pow(x2, 2.0) + pow(y2, 2.0);
                    double range = 0.0;
                    if (d1squared < d2squared)
                    {
                        range = sqrt(d1squared);
                    }
                    else
                    {
                        range = sqrt(d2squared);
                    }
                    if (range < min_range)
                    {
                        min_range = range;
                    }
                }
            }
        }
        if (min_range > msg.range_min)
        {
            msg.ranges.push_back(std::min(min_range + sample(0.001), (double)(msg.range_max)));
        }
        else
        {
            msg.ranges.push_back(0.0);
        }
    }
    return msg;
}
geometry_msgs::Polygon
Simulator::simulated_obstacles_msg(void) const
{
    return _obstacles;
}
