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
#include "nkg_demo_msgs/GetTable.h"
#include "nkg_demo_msgs/GetObjects.h"
//#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::unique_ptr<move_plan::Arm> arm;
std::unique_ptr<ros::ServiceClient> table_client, objs_client;
std::vector<float> table_rec(7,0);
typedef boost::variant<bool, geometry_msgs::Pose, std::vector<double>, std::string, move_plan::MOTION, std::vector<geometry_msgs::Pose>, std::vector<std::vector<double> >, std::pair<std::string, std::vector<moveit_msgs::Grasp> >, std::pair<std::string, std::vector<moveit_msgs::PlaceLocation> >, double > dataType;

void planTests(){
	// Test!
	ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_CYAN "Start planTests!" MOVEIT_CONSOLE_COLOR_RESET);

/*********************************** for motionToFrom demo
	std::vector<std::vector<double> > motions;
	std::vector<double> motion;

	motion.resize(4);
	motion[0] = 0.125;
	motion[1] = -1.0048;
	motion[2] = 1.3139;
	motion[3] = 0;
	motions.emplace_back(motion);

	motion[1] = 0.3;
	motion[2] = 1.2;
	motions.emplace_back(motion);
 ************************************/

/*********************************** for moveTo demo
	std::vector<std::vector<double> > moves;
	std::vector<double> move;
	move.resize(4);
	move[0] = 0.125;
	move[1] = -1;
	move[2] = 0.2;
	move[3] = 0;
	moves.emplace_back(move);

	move[1] = 1.3;
	move[2] = 0.5;
	moves.emplace_back(move);
 ************************************/

/*********************************** some cmd examples
	std::pair<move_plan::CMD, dataType> cmd;
	cmd = std::make_pair(move_plan::CMD::RESETPLAN, true);
	cmd = std::make_pair(move_plan::CMD::SETMOTIONMODE, move_plan::MOTION::LINE);
	cmd = std::make_pair(move_plan::CMD::MOVETO, moves);
	cmd = std::make_pair(move_plan::CMD::EXECTO, false);
	cmd = std::make_pair(move_plan::CMD::MOTIONTOFROM, motions);
	cmd = std::make_pair(move_plan::CMD::EXECTO, true);
 ************************************/

	// TODO modification
	nkg_demo_msgs::GetTable::Request  req;
	nkg_demo_msgs::GetTable::Response res;
	req.cleaned_table.assign(table_rec.cbegin(), table_rec.cend());
	if(table_client->call(req,res)){
		if(res.ready){	// table aligned
			if(res.update){	// table updated
				ROS_WARN_STREAM("Let's Clean");
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

				std::pair<move_plan::CMD, dataType> cmd(move_plan::CMD::MOTIONTOFROM, poses);
				arm->add2CmdQueue(cmd);
				cmd = std::make_pair(move_plan::CMD::EXECTO, true);
				arm->add2CmdQueue(cmd);
				arm->execCmdQueue();
				table_rec = std::move(res.table);
			}
			else
				ROS_WARN_STREAM("Most region has been cleaned.");
		}
		else{
			// TODO moving base
			ROS_ERROR_STREAM("Should Move Base!");
		}
	}
	else{
		ROS_ERROR_STREAM("GetTable service fails!");
	}
}

void gripperTests(){
	// Test!
	ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_CYAN "Start gripperTests!" MOVEIT_CONSOLE_COLOR_RESET);

	// TODO modification
	nkg_demo_msgs::GetObjects::Request  req;
	nkg_demo_msgs::GetObjects::Response res;

	struct Cuboid {	// unit: m
		geometry_msgs::Pose pose;
		int classId;
		std::array<double, 3> dim;
		std::string name;
	};

	if(objs_client->call(req,res)){
		std::vector<Cuboid> cuboids;
		tf2::Quaternion quat;
		for (size_t i=0; i<res.obj_name.size(); ++i){
			Cuboid cu;
			cu.classId = res.obj_type[i];
			cu.name = res.obj_name[i];
			std::copy_n(res.obj_detail.data.begin()+7*i+3, 3, cu.dim.begin());
			cu.pose.position.x = res.obj_detail.data[7*i];
			cu.pose.position.y = res.obj_detail.data[7*i+1];
			cu.pose.position.z = res.obj_detail.data[7*i+2];
			quat.setRPY(0, 0, res.obj_detail.data[7*i+6] * M_PI/180);
			cu.pose.orientation = tf2::toMsg(quat);
			cuboids.emplace_back(cu);
		}
		// sort with respect to obj size
		std::sort(cuboids.begin(), cuboids.end(),
			[](const Cuboid& a, const Cuboid& b)->bool{return a.dim[0]*a.dim[1]<b.dim[0]*b.dim[1];});
		// construct grasps and test from smallest obj
		for (const auto& cu : cuboids){

		}
	}
	else{
		ROS_ERROR_STREAM("GetObjects service fails!");
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

	// manipulator initialization
	arm.reset(new move_plan::Arm);
	arm->start();

	// construct service for table query
	ros::NodeHandle nh;
	table_client.reset(new ros::ServiceClient());
	objs_client.reset(new ros::ServiceClient());
	*table_client = nh.serviceClient<nkg_demo_msgs::GetTable>("get_table");
	*objs_client = nh.serviceClient<nkg_demo_msgs::GetObjects>("get_objects");
	table_client->waitForExistence(ros::Duration(5.0));
	objs_client->waitForExistence(ros::Duration(5.0));

	ros::Duration(10.0).sleep();
	if(test){
		planTests();
//		gripperTests();
	}

	ros::waitForShutdown();
	return 0;
}
