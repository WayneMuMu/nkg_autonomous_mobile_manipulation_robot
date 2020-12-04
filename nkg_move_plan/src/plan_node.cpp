/*********************************************************************
 *	Software License Agreement (BSD License)
 *
 *	All rights reserved.
 *
 *	Author: 	Yu-Wen Chen
 *	Date: 		2020/09
 *	Version: 	1.0.0
 *********************************************************************/
#include "nkg_move_plan/plan_node.h"
#include "nkg_demo_msgs/Table.h"
#include "moveit/macros/console_colors.h"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

std::unique_ptr<move_plan::Arm> arm;
std::unique_ptr<ros::ServiceClient> table_client;
std::vector<float> table_rec(7,0);

void someTests(){
	// Test!
	ROS_INFO(MOVEIT_CONSOLE_COLOR_CYAN "Start Testing!" MOVEIT_CONSOLE_COLOR_RESET);
/*
//*********************************** for motionToFrom demo
	std::vector<std::vector<double> > motions;
	std::vector<double> motion;

	motion.resize(4);
	motion[0] = 0.125;
	motion[1] = -1.0048;
	motion[2] = 1.3139;
	motion[3] = 0;
	motions.push_back(motion);

	motion[1] = 0.3;
	motion[2] = 1.2;
	motions.push_back(motion);
//************************************

//*********************************** for moveTo demo
	std::vector<std::vector<double> > moves;
	std::vector<double> move;
	move.resize(4);
	move[0] = 0.125;
	move[1] = -1;
	move[2] = 0.2;
	move[3] = 0;
	moves.push_back(move);

	move[1] = 1.3;
	move[2] = 0.5;
	moves.push_back(move);
//************************************

	std::pair<uint8_t, move_plan::dataType> cmd;
	cmd = std::make_pair(move_plan::CMD::RESETPLAN, true);
	arm->add2CmdQueue(cmd);
	cmd = std::make_pair(move_plan::CMD::SETMOTIONMODE, (uint8_t)move_plan::MOTION::LINE);
	arm->add2CmdQueue(cmd);
	cmd = std::make_pair(move_plan::CMD::MOVETO, motions[1]);
	arm->add2CmdQueue(cmd);
	cmd = std::make_pair(move_plan::CMD::EXECTO, false);
	cmd = std::make_pair(move_plan::CMD::MOTIONTOFROM, motions);
	arm->add2CmdQueue(cmd);
	cmd = std::make_pair(move_plan::CMD::EXECTO, true);
	arm->add2CmdQueue(cmd);
	arm->execCmdQueue();
/*/
	// TODO modification
	nkg_demo_msgs::Table::Request  req;
	nkg_demo_msgs::Table::Response res;
	req.cleaned_table.assign(table_rec.cbegin(), table_rec.cend());
	if(table_client->call(req,res)){
		if(res.ready){	// table aligned
			if(res.update){	// table updated
				ROS_WARN("Let's Clean");
				std::vector<geometry_msgs::Pose> poses;
				poses.resize(2);
				tf2::Quaternion quat;
				quat.setRPY(0, 0, res.table[6] * M_PI/180);
				tf2::Transform trans(quat, tf2::Vector3(res.table[3],res.table[4],res.table[5]));
				tf2::Vector3 p;

				geometry_msgs::Pose pose;
				pose.orientation.w = 1;
			
				p = trans * tf2::Vector3(-res.table[0]/2, -res.table[1]/2, res.table[2]/2 + 0.15);
				pose.position.x = p.getX();
				pose.position.y = p.getY();
				pose.position.z = p.getZ();
				poses[0] = pose;
				p = trans * tf2::Vector3(res.table[0]/2, res.table[1]/2, res.table[2]/2 + 0.15);
				pose.position.x = p.getX();
				pose.position.y = p.getY();
				pose.position.z = p.getZ();
				poses[1] = pose;

				std::pair<uint8_t, move_plan::dataType> cmd(move_plan::CMD::MOTIONTOFROM, poses);
				arm->add2CmdQueue(cmd);
				cmd = std::make_pair(move_plan::CMD::EXECTO, true);
				arm->add2CmdQueue(cmd);
				arm->execCmdQueue();
				table_rec.assign(res.table.begin(), res.table.end());
			}
			else
				ROS_WARN("Most region has been cleaned.");
		}
		else{
			// TODO moving base
			ROS_ERROR("Should Move Base!");
		}
	}
	else{
		ROS_ERROR("Table service fails!");
	}

}

int main(int argc, char **argv){
	// ROS initialization
	ros::init(argc, argv, "plan_node");
	bool test = false;
	for (size_t i=0; i<argc; ++i){
		std::string param(argv[i]);
		if (param.compare("test") == 0)
			test = true;
	}
	ros::AsyncSpinner spinner(0);
	spinner.start();

	// Manipulator initialization
	arm.reset(new move_plan::Arm);
	arm->start();

	// Construct service for table query
	ros::NodeHandle nh;
	table_client.reset(new ros::ServiceClient());
	*table_client = nh.serviceClient<nkg_demo_msgs::Table>("get_table");
	table_client->waitForExistence(ros::Duration(5.0));

	ros::Duration(10.0).sleep();
	if(test)
		someTests();

	ros::waitForShutdown();
	return 0;
}
