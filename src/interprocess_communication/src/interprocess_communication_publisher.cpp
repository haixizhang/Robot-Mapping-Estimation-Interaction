#include <iostream>
#include "ros/ros.h"
#include "interprocess_communication/interprocess_communication.h"
#include "interprocess_communication/Update.h"

using namespace std;
int main( int argc, char* argv[] ){
	ros::init( argc, argv, "interprocess_communication_publisher" );
	ros::NodeHandle node_handle;
	ros::Publisher update_publisher = node_handle.advertise<
		interprocess_communication::Update >( "update", 1, true );
	sleep( 1 );
	cout << "creating message" << endl;
	interprocess_communication::Update msg;
	msg.id = "test";
	msg.x = 5.0;
	msg.y = 2.0;
	msg.z = 1.0;
	cout << "message:" << endl << msg;	
	cout << "publishing message" << endl;
	update_publisher.publish( msg );
	sleep(1);
	cout << "done" << endl;
	
	return 0;
}

