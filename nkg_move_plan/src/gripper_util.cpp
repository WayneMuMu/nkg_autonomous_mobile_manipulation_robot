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
#include <thread>

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
	resetGripper();
}

void Gripper::configParam(){
	ros::NodeHandle _n("~");
	_n.param<double>("regraspDelay", _regrasp_delay, 0.01);
	_n.param<int>("regraspAttempts", _regrasp_attempts, 10);
	_n.param<double>("replaceDelay", _replace_delay, 0.01);
	_n.param<int>("replaceAttempts", _replace_attempts, 10);
	_n.param<double>("planTime", _plan_time, 0.1);
}

bool Gripper::planAndExecGrasp(const std::string& obj, const std::vector<moveit_msgs::Grasp>& grasps){
	moveit_msgs::PickupGoal goal;
	goal.target_name = obj;
	goal.group_name = "arm";
	goal.end_effector = "dobot_m1_gripper_link";
	goal.support_surface_name = "table";
	goal.possible_grasps = grasps;
	goal.allow_gripper_support_collision = true;
	goal.allowed_planning_time = _plan_time;
    goal.planning_options.plan_only = false;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = true;
    goal.planning_options.replan_delay = _regrasp_delay;
	goal.planning_options.replan_attempts = _regrasp_attempts;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

	plan_execution::PlanExecution::Options opt;
	opt.replan_ = goal.planning_options.replan;
	opt.replan_attempts_ = goal.planning_options.replan_attempts;
	opt.replan_delay_ = goal.planning_options.replan_delay;
	opt.before_execution_callback_ = [this](){ _grasp_executed = false; };
	opt.plan_callback_ = boost::bind(&Gripper::planGrasp, this, boost::cref(goal), _1);
	opt.done_callback_ = [this](){ _grasp_executed = true; };

	plan_execution::ExecutableMotionPlan plan;
//	_context->plan_execution_->planAndExecute(plan, goal.planning_options.planning_scene_diff, opt);

	// start an executing thread
	void (plan_execution::PlanExecution::*func)(plan_execution::ExecutableMotionPlan&, const moveit_msgs::PlanningScene&, const plan_execution::PlanExecution::Options&) 
												= &plan_execution::PlanExecution::planAndExecute;
	std::thread exec(func, _context->plan_execution_, 
				std::ref(plan), std::cref(goal.planning_options.planning_scene_diff), std::cref(opt));
	//	check whether the obj is still there, else do preemption
	ros::Rate r(100);
	while (ros::ok() && !_grasp_executed){
		if (!objStillThere()){
			ROS_WARN_STREAM("Grasp object has been moved, so let's stop!");
			_context->plan_execution_->stop();
			break;
		}
		r.sleep();
	}
	exec.join();

	return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
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

bool Gripper::planAndExecPlace(const std::string& obj, 
								const std::vector<moveit_msgs::PlaceLocation>& locations){
	moveit_msgs::PlaceGoal goal;
	goal.group_name = "arm";
	goal.attached_object_name = obj;
	goal.support_surface_name = "table";
	goal.place_locations = locations;
	goal.allow_gripper_support_collision = true;
	goal.allowed_planning_time = _plan_time;
	goal.planning_options.plan_only = false;
	goal.planning_options.look_around = false;
	goal.planning_options.replan = true;
	goal.planning_options.replan_delay = _replace_delay;
	goal.planning_options.replan_attempts = _replace_attempts;
	goal.planning_options.planning_scene_diff.is_diff = true;
	goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

	plan_execution::PlanExecution::Options opt;
	opt.replan_ = goal.planning_options.replan;
	opt.replan_attempts_ = goal.planning_options.replan_attempts;
	opt.replan_delay_ = goal.planning_options.replan_delay;
	opt.before_execution_callback_ = [this](){ _place_executed = false; };
	opt.plan_callback_ = boost::bind(&Gripper::planPlace, this, boost::cref(goal), _1);
	opt.done_callback_ = [this](){ _place_executed = true; };

	plan_execution::ExecutableMotionPlan plan;
//	_context->plan_execution_->planAndExecute(plan, goal.planning_options.planning_scene_diff, opt);

	// start an executing thread
	void (plan_execution::PlanExecution::*func)(plan_execution::ExecutableMotionPlan&, const moveit_msgs::PlanningScene&, const plan_execution::PlanExecution::Options&) 
												= &plan_execution::PlanExecution::planAndExecute;
	std::thread exec(func, _context->plan_execution_, 
				std::ref(plan), std::cref(goal.planning_options.planning_scene_diff), std::cref(opt));
	//	check whether the obj can be placed there, else do preemption
	ros::Rate r(100);
	while (ros::ok() && !_place_executed){
		if (!objCanBeThere()){
			ROS_WARN_STREAM("Place space has been occupied, so let's stop!");
			_context->plan_execution_->stop();
			break;
		}
		r.sleep();
	}
	exec.join();

	return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
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

bool Gripper::objStillThere(){

}

bool Gripper::objCanBeThere(){

}

bool Gripper::gripperTo(double val){
	control_msgs::GripperCommandGoal goal;
	goal.command.position = val;
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
	resetGripper();
	return status;
}

void Gripper::resetGripper(){
	*_grasp_end_state = *_psm->getStateMonitor()->getCurrentState();
}

}	// namespace move_plan
