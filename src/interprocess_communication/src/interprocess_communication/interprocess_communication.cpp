#include <iostream>
#include <Eigen/Dense>
#include "interprocess_communication/interprocess_communication.h"

using namespace std;

InterprocessCommunication::
InterprocessCommunication(){
}

InterprocessCommunication::
~InterprocessCommunication(){
}

void
InterprocessCommunication::
handle_update(const interprocess_communication::Update::ConstPtr& msg){	
  cout << "got_update_msg_with_id_" << msg->id << "_and_values_(x,y,z)=(" << msg->x << ","<< msg->y << "," << msg->z << ")" << endl;
	Eigen::Vector3d a;
	a(0) = msg->x;
	a(1) = msg->y;
	a(2) = msg->z;
	cout << "a[" << a.size() << "]:{" << a(0) << "," << a(1) << "," << a(2) << ",]" << endl;
  Eigen::Vector3d b = 2*a;
	cout << "b[" << b.size() << "]:{" << b(0) << "," << b(1) << "," << b(2) << ",]" << endl;
	return;


}
