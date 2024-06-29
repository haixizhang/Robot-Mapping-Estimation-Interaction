#include <iostream>
#include "ros/ros.h"
#include "interprocess_communication/interprocess_communication.h"

using namespace std;

int main( int argc, char* argv[] ){
	InterprocessCommunication interprocess_communication;
	ros::init( argc, argv, "interprocess_communication_node" );
	ros::NodeHandle node_handle;
	ros::Subscriber update_subscriber = node_handle.subscribe( "update", 1, &
		InterprocessCommunication::handle_update, &interprocess_communication );
	
	ros::Rate timer( 1 );
	while(ros::ok() ){
		ros::spinOnce();
		timer.sleep();
	}

	return 0;
}

