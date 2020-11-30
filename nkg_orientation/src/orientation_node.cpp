#include "nkg_orientation/orientation_node.h"
#include <ros/callback_queue.h>

int main(int argc, char **argv){
	// ROS Initialization
	ros::init(argc, argv, "nkg_orientation_node");
	ros::NodeHandle nh;
	ros::CallbackQueue ori_queue;
	nh.setCallbackQueue(&ori_queue);
	ros::AsyncSpinner spinner(0, &ori_queue);
	spinner.start();

	orientation_util::OrientationUtil util(nh);
	if(!util.start()){
		ROS_ERROR("Cannot start orientation_node");
		return -1;
	}

	ros::waitForShutdown();
	return 0;
}
