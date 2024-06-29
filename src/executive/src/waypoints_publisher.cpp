#include <iostream>
#include "ros/ros.h"

#include "geometry_msgs/Polygon.h"

using namespace std;

int
main( int argc, char* argv[] ){

  geometry_msgs::Polygon waypoints;

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 2.0;  
  waypoints.points.back().y = 0.0;  
  waypoints.points.back().z = 0.1;  

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 4.0;
  waypoints.points.back().y = 0.0;
  waypoints.points.back().z = 0.1;  

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 3.0;
  waypoints.points.back().y = 3.0;
  waypoints.points.back().z = 0.1;  

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 4.00;
  waypoints.points.back().y = 7.00;
  waypoints.points.back().z = 0.1;  

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;
  waypoints.points.back().y = 7.0;
  waypoints.points.back().z = 0.1;  

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = -5.00;
  waypoints.points.back().y = 7.00;
  waypoints.points.back().z = 0.1;  

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = -5.00;
  waypoints.points.back().y = 0.00;
  waypoints.points.back().z = 0.1;  

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 0.0;
  waypoints.points.back().y = 0.0;
  waypoints.points.back().z = 0.1;  

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = -3.5;
  waypoints.points.back().y = 7.0;
  waypoints.points.back().z = 0.1;  

  waypoints.points.push_back( geometry_msgs::Point32() );
  waypoints.points.back().x = 4.0;
  waypoints.points.back().y = 0.0;
  waypoints.points.back().z = 0.1;

  ros::init( argc, argv, "waypoints_publisher" );
  ros::NodeHandle node_handle;
  ros::Publisher waypoints_publisher = node_handle.advertise< geometry_msgs::Polygon >( "executive/waypoints", 1, true );

  sleep( 1 );
  ROS_INFO("Waypoints published!" );
  waypoints_publisher.publish( waypoints );
  
  sleep( 1 );

  return 0;
}
