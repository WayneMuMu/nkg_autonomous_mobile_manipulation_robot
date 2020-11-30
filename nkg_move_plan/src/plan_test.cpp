/*********************************************************************
 *	Software License Agreement (BSD License)
 *
 *	All rights reserved.
 *
 *	Author: 	Yu-Wen Chen
 *	Date: 		2020/09
 *	Version: 	1.0.0
 *********************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "plan_test_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(0);
	spinner.start();

	ROS_INFO("Hello World!");
	ros::Publisher pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

	while (pub.getNumSubscribers() < 1){
		ros::Duration(0.05).sleep();
	}

	// construct obstacle
	moveit_msgs::CollisionObject object;
	object.header.frame_id = "dobot_m1_base_link";
	object.id = "box";

	// pose of primitive
	geometry_msgs::Pose pose;
	pose.orientation.w = 1;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.position.x = -0.25;
	pose.position.y = 0.42;
	pose.position.z = 0.15;

	// size of primitive
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.03;
	primitive.dimensions[1] = 0.03;
	primitive.dimensions[2] = 0.3;

	// obj_list collects obstacles
	std::vector<moveit_msgs::CollisionObject> obj_list;
	object.primitives.push_back(primitive);
	object.primitive_poses.push_back(pose);
	object.operation = object.ADD;
	obj_list.push_back(object);
	
	// add to planning_scene through diff_publisher
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects = obj_list;
	planning_scene.is_diff = true;
	pub.publish(planning_scene);

	const double stepx = 0.01, stepy = 0.01, stepz = 0.01;
	char key;
	ROS_WARN("ENTER 5 to Quit!\n");

	system("stty raw");
	while(1){
		key = std::cin.get();
		switch(key-'0'){
			case 3:
				planning_scene.world.collision_objects[0].primitive_poses[0].position.z -= stepz;
				planning_scene.world.collision_objects[0].operation = object.MOVE;
				pub.publish(planning_scene);
				break;
			case 9:
				planning_scene.world.collision_objects[0].primitive_poses[0].position.z += stepz;
				planning_scene.world.collision_objects[0].operation = object.MOVE;
				pub.publish(planning_scene);
				break;
			case 4:
				planning_scene.world.collision_objects[0].primitive_poses[0].position.x -= stepx;
				planning_scene.world.collision_objects[0].operation = object.MOVE;
				pub.publish(planning_scene);
				break;
			case 6:
				planning_scene.world.collision_objects[0].primitive_poses[0].position.x += stepx;
				planning_scene.world.collision_objects[0].operation = object.MOVE;
				pub.publish(planning_scene);
				break;
			case 2:
				planning_scene.world.collision_objects[0].primitive_poses[0].position.y -= stepy;
				planning_scene.world.collision_objects[0].operation = object.MOVE;
				pub.publish(planning_scene);
				break;
			case 8:
				planning_scene.world.collision_objects[0].primitive_poses[0].position.y += stepy;
				planning_scene.world.collision_objects[0].operation = object.MOVE;
				pub.publish(planning_scene);
				break;
			case 1:
				planning_scene.world.collision_objects[0].operation = object.REMOVE;
				pub.publish(planning_scene);
				break;
			default:
				pub.publish(planning_scene);
				break;				
		}
		if (key-'0' == 5)
			break;
	}	
	system("stty cooked");
	system("clear");

	return 0;
}
