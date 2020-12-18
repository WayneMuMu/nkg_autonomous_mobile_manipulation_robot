#ifndef NKG_GRIPPER_UTIL_H
#define NKG_GRIPPER_UTIL_H

#include "moveit/plan_execution/plan_execution.h"
#include "moveit/move_group/move_group_context.h"
#include "nkg_move_plan/name_def.h"

#include "moveit/pick_place/pick_place.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

namespace move_plan
{
class Gripper{
public:
	Gripper(move_group::MoveGroupContext*, moveit::core::RobotState*);
	Gripper(const Gripper&) = delete;
	Gripper& operator=(const Gripper&) = delete;
	void start();
	bool planAndExecGrasp(const std::string&, const std::vector<moveit_msgs::Grasp>&);
	bool planAndExecPlace(const std::string&, const std::vector<moveit_msgs::PlaceLocation>&);
	bool gripperTo(double);
	void resetGripper();

private:
	void configParam();
	bool planGrasp(const moveit_msgs::PickupGoal&, plan_execution::ExecutableMotionPlan&);
	bool planPlace(const moveit_msgs::PlaceGoal&, plan_execution::ExecutableMotionPlan&);
	bool objStillThere();
	bool objCanBeThere();

	planning_scene_monitor::PlanningSceneMonitorPtr _psm;
	move_group::MoveGroupContext *_context;
	pick_place::PickPlacePtr _pick_place;
	moveit::core::RobotState *_grasp_end_state;
	double _regrasp_delay, _replace_delay, _plan_time;
	int _regrasp_attempts, _replace_attempts;
	std::unique_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction> > _ac;
	bool _grasp_executed, _place_executed;
};

}
#endif
