/*********************************************************************
 *	Software License Agreement (BSD License)
 *
 *	All rights reserved.
 *
 *	Author: 	Yu-Wen Chen
 *	Date: 		2020/09
 *	Version: 	1.0.0
 *********************************************************************/
#include "nkg_move_plan/gripper_util.h"

#define GRASP_GROUP "gripper"

namespace move_plan
{
Gripper::Gripper(move_group::MoveGroupContext* context, moveit::core::RobotState* st): _context(context), _grasp_end_state(st){
	// Configure params
	configParam();

	// Planning Scene Monitor
	_psm = _context->planning_scene_monitor_;

	// Pick & Place
	_pick_place.reset(new pick_place::PickPlace(_context->planning_pipeline_));
}

void Gripper::start(){
	_ac.reset(new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>("gripper_controller/gripper_cmd", true));
	_ac->waitForServer(ros::Duration(5.0));
	// Record status of current state
	resetGoal();
}

void Gripper::configParam(){
	ros::NodeHandle _n("~");
	_n.param("regraspDelay", _regrasp_delay, 0.01);
	_n.param("regraspAttempts", _regrasp_attempts, 10);
	_n.param("replaceDelay", _replace_delay, 0.01);
	_n.param("replaceAttempts", _replace_attempts, 10);
}

bool Gripper::planAndExecGrasp(const moveit_msgs::PickupGoal& goal){
	plan_execution::PlanExecution::Options opt;
	opt.replan_ = true;
	opt.replan_attempts_ = _regrasp_attempts;
	opt.replan_delay_ = _regrasp_delay;
	opt.before_execution_callback_ = boost::bind(&Gripper::beforePlanGrasp, this);
	opt.plan_callback_ = boost::bind(&Gripper::planGrasp, this, boost::cref(goal), _1);

	plan_execution::ExecutableMotionPlan plan;
	_context->plan_execution_->planAndExecute(plan, opt);

	return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

void Gripper::beforePlanGrasp(){
	ROS_INFO_STREAM("Start to plan grasp.");
}

bool Gripper::planGrasp(const moveit_msgs::PickupGoal& goal, plan_execution::ExecutableMotionPlan& plan){
	planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
	pick_place::PickPlanPtr pick_plan;
  
	try{
		pick_plan = _pick_place->planPick(lscene, goal);
	}
	catch (std::exception& ex){
		ROS_ERROR_STREAM_NAMED("manipulation", "Pick threw an exception: " << ex.what());
		return false;
	}

	if (pick_plan){
		const std::vector<pick_place::ManipulationPlanPtr>& success = pick_plan->getSuccessfulManipulationPlans();
		if (success.empty())
			plan.error_code_ = pick_plan->getErrorCode();
    	else{
			const pick_place::ManipulationPlanPtr& result = success.front();
			plan.plan_components_ = result->trajectories_;
			plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
		}
	}
	else
		plan.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;

	return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

bool Gripper::planAndExecPlace(const moveit_msgs::PlaceGoal& goal){
	plan_execution::PlanExecution::Options opt;
	opt.replan_ = true;
	opt.replan_attempts_ = _replace_attempts;
	opt.replan_delay_ = _replace_delay;
	opt.before_execution_callback_ = boost::bind(&Gripper::beforePlanPlace, this);
	opt.plan_callback_ = boost::bind(&Gripper::planPlace, this, boost::cref(goal), _1);

	plan_execution::ExecutableMotionPlan plan;
	_context->plan_execution_->planAndExecute(plan, opt);

	return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

void Gripper::beforePlanPlace(){
	ROS_INFO_STREAM("Start to plan place.");
}

bool Gripper::planPlace(const moveit_msgs::PlaceGoal& goal, plan_execution::ExecutableMotionPlan& plan){
	planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
	pick_place::PlacePlanPtr place_plan;
  
	try{
		place_plan = _pick_place->planPlace(lscene, goal);
	}
	catch (std::exception& ex){
		ROS_ERROR_STREAM_NAMED("manipulation", "Place threw an exception: " << ex.what());
		return false;
	}

	if (place_plan){
		const std::vector<pick_place::ManipulationPlanPtr>& success = place_plan->getSuccessfulManipulationPlans();
		if (success.empty())
			plan.error_code_ = place_plan->getErrorCode();
    	else{
			const pick_place::ManipulationPlanPtr& result = success.front();
			plan.plan_components_ = result->trajectories_;
			plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
		}
	}
	else
		plan.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;

	return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

bool Gripper::gripperTo(const double val, const bool queued){
	if (!queued){
		control_msgs::GripperCommandGoal goal;
		goal.command.position = val;
		return execTo(goal);
	}
	else{
		_goal.command.position = val;
		return true;
	}
}

bool Gripper::execTo(control_msgs::GripperCommandGoal& goal){
	_ac->sendGoal(goal);
	_ac->waitForResult(ros::Duration(2.0));

	// result
	bool status;
	if (_ac->getState() == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
		status = true;
	else{
		ROS_ERROR_STREAM("Fail to execute gripper!");
		status = false;
	}
	resetGoal();
	return status;
}

bool Gripper::execTo(){
	return execTo(_goal);
}

void Gripper::resetGoal(){
	*_grasp_end_state = *_psm->getStateMonitor()->getCurrentState();
}

}	// namespace move_plan
