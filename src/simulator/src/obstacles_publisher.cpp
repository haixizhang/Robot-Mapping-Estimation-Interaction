#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
using namespace std;

int main(int argc, char *argv[])
{
    geometry_msgs::Polygon obstacles;
    unsigned int num_obstacles = 50;
    double min_x = -10.0;
    double max_x = 10.0;
    double min_y = -10.0;
    double max_y = 10.0;
    double min_radius = 0.3;
    double max_radius = 0.7;
    for (unsigned int i = 0; i < num_obstacles; i++)
    {
        obstacles.points.push_back(geometry_msgs::Point32());
        obstacles.points.back().x = min_x + (double)(rand() % 101) / (100.0) * (max_x - min_x);
        obstacles.points.back().y = min_y + (double)(rand() % 101) / (100.0) * (max_y - min_y);
        obstacles.points.back().z = min_radius + (double)(rand() % 101) / (100.0) * (max_radius - min_radius);
    }

    // test
    geometry_msgs::Point32 point1;
    point1.x = -2;
    point1.y = -4;
    point1.z = 0.5;
    geometry_msgs::Point32 point2;
    point2.x = 0;
    point2.y = -4;
    point2.z = 0.5;
    obstacles.points.push_back(point1);
    obstacles.points.push_back(point2);
    ros::init(argc, argv, "obstacles_publisher_node");
    ros::NodeHandle node_handle;
    ros::Publisher obstacles_publisher = node_handle.advertise<geometry_msgs::Polygon>("obstacles", 1, true);
    obstacles_publisher.publish(obstacles);
    sleep(1);
    return 0;
}
