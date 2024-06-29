#include "ros/ros.h"
#include  "interprocess_communication/Update.h"

class InterprocessCommunication {
	public:
		InterprocessCommunication();
		virtual ~InterprocessCommunication();
		void handle_update (const interprocess_communication::Update::ConstPtr& msg);
};
