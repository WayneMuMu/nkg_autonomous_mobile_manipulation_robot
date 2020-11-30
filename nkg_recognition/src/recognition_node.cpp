#include "nkg_recognition/recognition_node.h"

int main(int argc, char **argv){
	// ROS Initialization
	ros::init(argc, argv, "recognition_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(0);
	spinner.start();

	recognition_util::RecognitionUtil util(nh);
	util.start();
	
	ROS_INFO("Hello World!");

	ros::waitForShutdown();
	return 0;
}
