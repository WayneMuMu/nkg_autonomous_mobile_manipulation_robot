#include "nkg_recognition/recognition_node.h"

int main(int argc, char **argv){
	// ROS Initialization
	ros::init(argc, argv, "recognition_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(0);
	spinner.start();

	recognition_util::RecognitionUtil util(nh);
	if(!util.start()){
		ROS_ERROR("Cannot start recognition_node");
		return -1;
	}

	ros::waitForShutdown();
	return 0;
}
