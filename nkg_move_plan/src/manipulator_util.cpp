/*********************************************************************
 *	Software License Agreement (BSD License)
 *
 *	All rights reserved.
 *
 *	Author: 	Yu-Wen Chen
 *	Date: 		2020/09
 *	Version: 	1.0.0
 *********************************************************************/
#include "nkg_move_plan/manipulator_util.h"
#include "moveit/robot_state/cartesian_interpolator.h"
#include "moveit/kinematic_constraints/utils.h"
#include "moveit/robot_state/conversions.h"
#include "moveit_msgs/GetCartesianPath.h"
#include "moveit_msgs/GetPositionFK.h"
#include <tf2_eigen/tf2_eigen.h>

#include <cmath>

namespace move_plan
{
Manipulator::Manipulator(move_group::MoveGroupContext* context, moveit::core::RobotState* st): _motion_mode(MOTION::LINE), _context(context), _plan_end_state(st){
	// retrieve parameters
	configParam();

	// Planning Scene Monitor
	_psm = _context->planning_scene_monitor_;

	// Robot Model
	_robot_model = _psm->getRobotModel();

	// Joint Model Group
	_jnt_model_group = _plan_end_state->getJointModelGroup(PLAN_GROUP);

	// defined group_states in SRDF
	_srdf_names = _jnt_model_group->getDefaultStateNames();

#if DRAW
	// visualization
	_draw_plan.header.frame_id = _draw_exec.header.frame_id = _draw_pts.header.frame_id = BASE_LINK;
	_draw_plan.ns = _draw_exec.ns = _draw_pts.ns = "planned_and_executed";
	_draw_plan.action = _draw_exec.action = _draw_pts.action = visualization_msgs::Marker::ADD;
	_draw_plan.type = _draw_exec.type = visualization_msgs::Marker::LINE_STRIP;
	_draw_pts.type = visualization_msgs::Marker::POINTS;
	_draw_plan.id = 0;
	_draw_exec.id = 1;
	_draw_pts.id = 2;
	_draw_plan.scale.x = 0.003;
	_draw_exec.scale.x = 0.005;
	_draw_pts.scale.x = 0.015;
	_draw_pts.scale.y = 0.015;
	_draw_plan.color.g = 1.0;
	_draw_exec.color.r = 1.0;
	_draw_pts.color.b = 1.0;
	_draw_plan.color.a = _draw_exec.color.a = _draw_pts.color.a = 1.0;
#endif
}

void Manipulator::configParam(){
	ros::NodeHandle _n("~");
	_n.param<double>("posTolerance", _pos_tol, 0.005);
	_n.param<double>("oriTolerance", _ori_tol, 0.005);
	_n.param<double>("planTime", _plan_time, 0.1);
	_n.param<double>("replanDelay", _replan_delay, 0.01);
	_n.param<int>("replanAttempts", _replan_attempts, 50);
	_n.param<int>("planAttempts", _plan_attempts, 3);
	_n.param<int>("replanJump", _replan_jump, 5);
	_n.param<bool>("careAllTargets", _care, false);
}

void Manipulator::start(){
#if DRAW
	ros::NodeHandle n;
	_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 5);
#endif
	_xyz_srv = n.advertiseService("get_xyz", &Manipulator::getXYZ, this);
	// Record status of current Plan
	resetPlan();
}

bool Manipulator::getXYZ(nkg_demo_msgs::GetXYZ::Request &req, nkg_demo_msgs::GetXYZ::Response &res){
	moveit::core::RobotState temp(_robot_model);
	temp.setJointGroupPositions(_jnt_model_group, req.joints);
	geometry_msgs::Pose pose = tf2::toMsg(temp.getGlobalLinkTransform(EEF_GROUP));
	res.cartesian.resize(3);
	res.cartesian[0] = pose.position.x;
	res.cartesian[1] = pose.position.y;
	res.cartesian[2] = pose.position.z;
	return true;
}

void Manipulator::resetPlan(){
	*_plan_end_state = *_psm->getStateMonitor()->getCurrentState();
	_plan_traj.clear();
	_replan_num = 0;
	_latest_goals.clear();
	_latest_motion.clear();
#if DRAW
	_first_draw = true;
	_first_targets = true;
	_draw_plan.points.clear();
	_draw_exec.points.clear();
	_draw_pts.points.clear();
#endif
}

bool Manipulator::hasSrdfName(const std::string& name) const{
	for (std::vector<std::string>::const_iterator p=_srdf_names.cbegin(); p!=_srdf_names.cend(); ++p){
		if (name.compare(*p) == 0){
			return true;
		}
	}
	return false;
}

bool Manipulator::moveTo(const geometry_msgs::Pose& pose){
	geometry_msgs::PoseStamped pose_stamp;
	pose_stamp.header.frame_id = BASE_LINK;
	pose_stamp.pose = pose;
	moveit_msgs::Constraints goal_msgs = kinematic_constraints::constructGoalConstraints(EEF_GROUP, pose_stamp, 0.005, 0.005);
	return moveTo(goal_msgs);
}

bool Manipulator::moveTo(const std::string& name){
	if(!hasSrdfName(name)){
		ROS_ERROR_STREAM("No name: " << name << "in SRDF!");
		return false;
	}
	else{
		moveit::core::RobotState goal(_robot_model);
		goal.setToDefaultValues(_jnt_model_group, name);
		return moveTo(goal);
	}
}

bool Manipulator::moveTo(const std::vector<double>& joints){
	moveit::core::RobotState goal(_robot_model);
	if ((_jnt_model_group->getActiveJointModelNames()).size() != joints.size()){
		ROS_ERROR_STREAM("Mismatch number of joints!");
		return false;
	}
	goal.setJointGroupPositions(_jnt_model_group, joints);
	return moveTo(goal);	
}

bool Manipulator::moveTo(const moveit::core::RobotState& goal){
	moveit_msgs::Constraints goal_msgs = kinematic_constraints::constructGoalConstraints(goal, _jnt_model_group, _pos_tol, _ori_tol);
	return moveTo(goal_msgs);
}

bool Manipulator::moveTo(const moveit_msgs::Constraints& goal_msgs){
	// prepare to plan
	planning_interface::MotionPlanRequest req;

	// msg for StartMessage in MotionPlanRequest
    moveit_msgs::RobotState start_msg;
	moveit::core::robotStateToRobotStateMsg(*_plan_end_state, start_msg);
		
	// create MotionPlanRequest
	req.group_name = PLAN_GROUP;
	req.allowed_planning_time = _plan_time;
	req.start_state = start_msg;
	req.goal_constraints.emplace_back(goal_msgs);
//	req.planner_id = "SBL";
//	req.workspace_parameters;
//	req.path_constraints;
//	req.num_planning_attempts;
//	req.max_velocity_scaling_factor;
//	req.max_acceleration_scaling_factor;

	return moveTo(req);
}

bool Manipulator::moveTo(planning_interface::MotionPlanRequest& req){
	if (planTo(req)){
		_latest_goals.emplace_back(req);
		return true;
	}
	else
		return false;
}

bool Manipulator::planTo(planning_interface::MotionPlanRequest& req){
	size_t plan_num = 0;
	while (plan_num < _plan_attempts){
		// plan and write
		planning_scene_monitor::LockedPlanningSceneRO lscene(_psm);
		bool solved = false;
		planning_interface::MotionPlanResponse res;
		try{
			solved = _context->planning_pipeline_->generatePlan(lscene, req, res);
		}
		catch (std::exception& ex){
		    ROS_ERROR_STREAM("Planning pipeline threw an exception: " << ex.what() << "!");
			return false;	
		}

		if (solved && res.trajectory_){
			generateTraj(res.trajectory_);
			return true;
		}
		++plan_num;
	}
	// plan fails
	ROS_ERROR_STREAM("Plan fails! Has planed " << plan_num << " times!");
	return false;
}

void Manipulator::generateTraj(robot_trajectory::RobotTrajectoryPtr& traj){
	// Note the order, since traj becomes NULLPtr after std::move()
	*_plan_end_state = traj->getLastWayPoint();	
	_plan_traj.emplace_back(std::move(traj));
}

bool Manipulator::execTo(bool motion){
	if (motion){
		if (_latest_motion.empty() || _plan_traj.empty()){
			ROS_ERROR_STREAM("No plan to execute! Forget to motionTo?");
			return false;
		}
	}
	else{
		if (_latest_goals.empty() || _plan_traj.empty()){
			ROS_ERROR_STREAM("No plan to execute! Forget to moveTo or actually need execTo(true)?");
			return false;
		}
	}

	// smoothing
	calcIPTP();

#if DRAW	
	// draw planned path
	if (_first_draw){
		drawPath();
		_first_draw = false;
	}
#endif

	// construct plan
	plan_execution::ExecutableMotionPlan plan;
	plan.plan_components_.resize(_plan_traj.size());
	for (size_t i=0; i<_plan_traj.size(); ++i){
		plan.plan_components_[i].trajectory_ = std::move(_plan_traj[i]);
		plan.plan_components_[i].description_ = "plan"+std::to_string(i);
		plan.plan_components_[i].trajectory_monitoring_ = true;
	}

	// start execution thread
	ros::Rate r(100);
	std::pair<int, int> cur_idx(0,0), return_idx(0,0);
	_executed = _succeed = _replan = false;
	boost::thread t(&Manipulator::execThread, this, plan);
	while (ros::ok() && !_executed){
		return_idx  = _context->trajectory_execution_manager_-> getCurrentExpectedTrajectoryIndex();
		if (return_idx.first != -1)
			cur_idx = return_idx;
#if DRAW
		moveit::core::RobotState temp(*_psm->getStateMonitor()->getCurrentState());
		geometry_msgs::Pose pose = tf2::toMsg(temp.getGlobalLinkTransform(EEF_GROUP));
		_draw_exec.points.emplace_back(pose.position);
		_draw_exec.header.stamp = ros::Time::now();
		_marker_pub.publish(_draw_exec);
#endif
		r.sleep();
	}
	t.join();

	// result	
	if (_succeed){
		ROS_INFO_STREAM("EXEC SUCCEEDS!");
		resetPlan();
		return true;
	}
	else if ((_replan_num < _replan_attempts) && _replan){
		ROS_WARN_STREAM("Env changes. Stop "<<_replan_delay<<"s then "<<_replan_num<<"-th replan!");
		// replanning delay
		ros::Duration(_replan_delay).sleep();
		if (motion)
			return replanMotion(cur_idx);
		else
			return replanMove(cur_idx);
	}
	else{
		ROS_ERROR_STREAM("Execution fails!");
		resetPlan();
		return false;
	}
}

#if DRAW
void Manipulator::drawPath(){
	moveit::core::RobotState temp(_robot_model);
	geometry_msgs::Pose pose;

	for (size_t i=0; i<_plan_traj.size(); ++i){
		for (size_t j=0; j<_plan_traj[i]->getWayPointCount(); ++j){
			temp = _plan_traj[i]->getWayPoint(j);
			pose = tf2::toMsg(temp.getGlobalLinkTransform(EEF_GROUP));
			_draw_plan.points.emplace_back(pose.position);
			if (j == 0)
				_draw_pts.points.emplace_back(pose.position);
		}
	}

	if (!_draw_plan.points.empty()){
		_draw_plan.header.stamp = ros::Time::now();
		_marker_pub.publish(_draw_plan);
		_draw_pts.header.stamp = ros::Time::now();
		_marker_pub.publish(_draw_pts);
	}
	else
		ROS_ERROR_STREAM("_draw_plan empty!");
}
#endif

void Manipulator::execThread(plan_execution::ExecutableMotionPlan& plan){
	moveit_msgs::MoveItErrorCodes err = _context->plan_execution_->executeAndMonitor(plan);
	if (err.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
		_succeed = true;
	else if (err.val == moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)
		_replan = true;
	_executed = true;
}

void Manipulator::calcIPTP(){
	// Combine into one to calculate IPTP
	robot_trajectory::RobotTrajectory temp(_robot_model, PLAN_GROUP);

	// Record number of waypoints in _plan_traj[] to recover _plan_traj
	std::vector<size_t> way_pts_num(_plan_traj.size(), 0);
	for (size_t i=0; i<_plan_traj.size(); ++i){
		way_pts_num[i] = _plan_traj[i]->getWayPointCount();
		temp.append(*_plan_traj[i], 0);
	}

	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	if (!iptp.computeTimeStamps(temp))
		ROS_WARN_STREAM("IPTP fails!");
	else{
		size_t idx = 0;
		for (size_t i=0; i<_plan_traj.size(); ++i){
			_plan_traj[i]->clear();
			for (size_t j=0; j<way_pts_num[i]; ++j){
					_plan_traj[i]->addSuffixWayPoint(temp.getWayPointPtr(idx), temp.getWayPointDurationFromPrevious(idx));
				++idx;
			}
		}
		// hack to fix time for start and end point (suppose there are duplicate points)
		size_t last_idx = _plan_traj.back()->getWayPointCount() - 1;
		if (_plan_traj.back()->getWayPointDurationFromPrevious(last_idx) == 0.0)
			_plan_traj.back()->setWayPointDurationFromPrevious(last_idx, 0.001);
		if (_plan_traj[0]->getWayPointDurationFromPrevious(1) == 0.0)
			_plan_traj[0]->setWayPointDurationFromPrevious(1, 0.001);
	}
}

bool Manipulator::replanMove(const std::pair<int, int>& cur_idx){
	// remove executed goals
	if (cur_idx.first)
		_latest_goals.erase(_latest_goals.begin()+cur_idx.first-1);

	// plan from current state
	std::vector<planning_interface::MotionPlanRequest> temp_goals;
	temp_goals = std::move(_latest_goals);
	_latest_goals.clear();		// put it into defined valid state
	_plan_traj.clear();
	*_plan_end_state = *_psm->getStateMonitor()->getCurrentState();
	_plan_end_state->update();

	// check current status
	ros::Time t = ros::Time::now();
	bool keep = false;
	while (ros::Time::now()-t < ros::Duration(5.0)){
		if (!isPtValid(*_plan_end_state))
			ROS_ERROR_STREAM("Currently stuck in obstacle!");
		else{
			keep = true;
			break;
		}
		ros::Duration(1.0).sleep();
	}
	if (!keep)
		return false;

	for (size_t i=0; i<temp_goals.size(); ++i){
		if (!moveTo(temp_goals[i])){
			ROS_ERROR_STREAM("Replan Fails at Goal["<<i<<"]");
			if (_care){
				resetPlan();
				return false;
			}
		}
	}
	return execTo(false);
}

bool Manipulator::replanMotion(const std::pair<int, int>& cur_idx){
	// boundary check
	std::pair<int, int> start_idx = std::make_pair(cur_idx.first, cur_idx.second+1);
	if (start_idx.second >= _latest_motion[start_idx.first]->getWayPointCount()){
		if (start_idx.first+1 >= _latest_motion.size()){
			ROS_ERROR_STREAM("Replan index error!");
			return false;
		}
		else
			start_idx = std::make_pair(start_idx.first+1, 0);
	}

	// plan from current state
	_plan_traj.clear();
	*_plan_end_state = *_psm->getStateMonitor()->getCurrentState();
	_plan_end_state->update();

	// check current status
	ros::Time t = ros::Time::now();
	bool keep = false;
	while (ros::Time::now()-t < ros::Duration(5.0)){
		if (!isPtValid(*_plan_end_state))
			ROS_ERROR_STREAM("Currently stuck in obstacle!");
		else{
			keep = true;
			break;
		}
		ros::Duration(1.0).sleep();
	}
	if (!keep)
		return false;
	else
		*_plan_end_state = *_psm->getStateMonitor()->getCurrentState();

	// Two kinds of replan scenarios
	if (!moveTo(_latest_motion[start_idx.first]->getWayPoint(start_idx.second))){
		size_t j = start_idx.second+1;
		// find a "far" reachable pt, and construct rest plan
		for (size_t i=start_idx.first; i<_latest_motion.size(); ++i){
			for (; j<_latest_motion[i]->getWayPointCount(); j+=_replan_jump){
				if(isPtValid(_latest_motion[i]->getWayPoint(j))){
					if (connectTo(_latest_motion[i]->getWayPoint(j))){
						// replan succeeds, append rest plan and try to execute
						// in case current at _latest_motion[i]->getWayPoint(j) -> empty for back()
						if (_plan_traj.empty())
							_plan_traj.emplace_back(std::make_shared<robot_trajectory::RobotTrajectory>(_robot_model, PLAN_GROUP));
						for (size_t k=j+1; k<_latest_motion[i]->getWayPointCount(); ++k){
							// Use 0 instead of _latest_motion[i]->getWayPointDurationFromPrevious(k)
							// since execTo will execute IPTP
							_plan_traj.back()->addSuffixWayPoint(_latest_motion[i]->getWayPointPtr(k), 0.0);
						}
						// append rest plan
						for (size_t l=i+1; l<_latest_motion.size(); ++l)
							_plan_traj.emplace_back(std::move(_latest_motion[l]));
						// update latest planned motion
						_latest_motion.assign(_plan_traj.cbegin(), _plan_traj.cend());
						*_plan_end_state = _plan_traj.back()->getLastWayPoint();	
						_latest_goals.clear();	// using moveTo will construct _latest_goals record
						return execTo(true);
					}
					else
						ROS_WARN_STREAM("Can be next, but plan fails.");
				}
				else
					ROS_WARN_STREAM("CANNOT be next.");
			}
			j = 0;
		}
	}
	else{ // find the "following" reachable segment, and dissect there
		// in case current at _latest_motion[i]->getWayPoint(j) -> empty for back()
		if (_plan_traj.empty())
			_plan_traj.emplace_back(std::make_shared<robot_trajectory::RobotTrajectory>(_robot_model, PLAN_GROUP));
		size_t j = start_idx.second+1;
		for (; j<_latest_motion[start_idx.first]->getWayPointCount(); ++j){
			if (isPtValid(_latest_motion[start_idx.first]->getWayPoint(j)))
				_plan_traj.back()->addSuffixWayPoint(_latest_motion[start_idx.first]->getWayPointPtr(j),0.0);
			else
				break;	// now j indicates the start index of points left
		}
		// enter only when there are points left in _latest_motion[start_idx.first]
		if (j<_latest_motion[start_idx.first]->getWayPointCount())
			_plan_traj.emplace_back(std::make_shared<robot_trajectory::RobotTrajectory>(_robot_model, PLAN_GROUP));
		// enter only when there are points left in _latest_motion[start_idx.first]
		for (; j<_latest_motion[start_idx.first]->getWayPointCount(); ++j)
			_plan_traj.back()->addSuffixWayPoint(_latest_motion[start_idx.first]->getWayPointPtr(j),0.0);
		// append rest plan
		for (size_t i = start_idx.first+1; i<_latest_motion.size(); ++i)
			_plan_traj.emplace_back(std::move(_latest_motion[i]));
		// update latest planned motion
		_latest_motion.assign(_plan_traj.cbegin(), _plan_traj.cend());
		*_plan_end_state = _plan_traj.back()->getLastWayPoint();
		_latest_goals.clear();	// using moveTo will construct _latest_goals record, so clear it
		return execTo(true);
	}
	// replan fails
	resetPlan();
	if (_care){
		ROS_ERROR_STREAM("Replan motion fails!");
		return false;
	}
	else{
		ROS_WARN_STREAM("Cannot motion further");
		return true;
	}
}

bool Manipulator::connectTo(const moveit::core::RobotState& next){
	/*************************************************************************************************
	TODO one can customize this function, current is at *_plan_end_state, target to connect is at next
	*************************************************************************************************/
	return moveTo(next);
}

bool Manipulator::isPtValid(const moveit::core::RobotState& st){
	collision_detection::CollisionRequest req;
	collision_detection::CollisionResult res;
	req.verbose = false;
	req.group_name = PLAN_GROUP;
	planning_scene_monitor::LockedPlanningSceneRO lscene(_psm);
	lscene->getCollisionEnv()->checkRobotCollision(req, res, st, lscene->getAllowedCollisionMatrix());
	return !res.collision;
}

bool Manipulator::motionTo(const std::vector<std::vector<double> >& joints){
	// check number of joint values
	for (size_t i=0; i<joints.size(); ++i){
		if ((_jnt_model_group->getActiveJointModelNames()).size() != joints[i].size()){
			ROS_ERROR_STREAM("Mismatch number of joints at "<<i<<"-th!");
			return false;
		}
	}
	if (_motion_mode == MOTION::RAND){
		for (size_t i=0; i<joints.size(); ++i){
			if(!moveTo(joints[i])){
				ROS_ERROR_STREAM("Motion plan fails at joints["<<i<<"]!");
				if (_care){
					resetPlan();
					return false;
				}
			}
		}
		_latest_motion.assign(_plan_traj.cbegin(), _plan_traj.cend());
		_latest_goals.clear();
		return true;
	}
	else{
		std::vector<geometry_msgs::Pose> jnt2poses;
		moveit::core::RobotState temp(_robot_model);
		jnt2poses.resize(joints.size());
		for (size_t i=0; i<joints.size(); ++i){
			temp.setJointGroupPositions(_jnt_model_group, joints[i]);
			jnt2poses[i] = tf2::toMsg(temp.getGlobalLinkTransform(EEF_GROUP));
		}
		return motionTo(jnt2poses);
	}
}

bool Manipulator::motionTo(const std::vector<geometry_msgs::Pose>& poses){
	switch(_motion_mode){
		case MOTION::RAND:{
			for (size_t i=0; i<poses.size(); ++i){
				if(!moveTo(poses[i])){
					ROS_ERROR_STREAM("Motion plan fails at poses["<<i<<"]!");
					if (_care){
						resetPlan();
						return false;
					}
				}
			}
			break;
		}
		case MOTION::LINE:{
			if (!calcMotionLines(poses))
				return false;
			break;
		}
		case MOTION::CIRC:{
			if (!calcMotionCircs(poses))
				return false;
			break;
		}
	}

	// motion plan succeeds
	_latest_motion.assign(_plan_traj.cbegin(), _plan_traj.cend());
	_latest_goals.clear();
	return true;
}

bool Manipulator::motionToFrom(const std::vector<double>& joint){
	moveit::core::RobotState temp(_robot_model);
	temp.setJointGroupPositions(_jnt_model_group, joint);
	geometry_msgs::Pose pose = tf2::toMsg(temp.getGlobalLinkTransform(EEF_GROUP));
	return motionToFrom(pose);
}

bool Manipulator::motionToFrom(const geometry_msgs::Pose& pose){
	geometry_msgs::Pose current = tf2::toMsg(_psm->getStateMonitor()->getCurrentState()->getGlobalLinkTransform(EEF_GROUP));

	return motionToFrom(pose, current);
}

bool Manipulator::motionToFrom(const std::vector<double>& joint1, const std::vector<double>& joint2){
	moveit::core::RobotState temp1(_robot_model), temp2(_robot_model);
	temp1.setJointGroupPositions(_jnt_model_group, joint1);
	geometry_msgs::Pose to = tf2::toMsg(temp1.getGlobalLinkTransform(EEF_GROUP));
	temp2.setJointGroupPositions(_jnt_model_group, joint2);
	geometry_msgs::Pose from = tf2::toMsg(temp2.getGlobalLinkTransform(EEF_GROUP));
	return motionToFrom(to, from);
}

bool Manipulator::motionToFrom(const geometry_msgs::Pose& ps1, const geometry_msgs::Pose& ps2){
	geometry_msgs::Pose to_pose = ps1, from_pose = ps2;
	ROS_INFO_STREAM("GET to:("<<to_pose.position.x<<", "<<to_pose.position.y<<", "<<to_pose.position.z<<"), from:("<<from_pose.position.x<<", "<<from_pose.position.y<<", "<<from_pose.position.z<<")");
	// check whether in workspace
	moveIntoWS(to_pose, DIRECT::R);
	moveIntoWS(from_pose, DIRECT::R);
	ROS_INFO_STREAM("SET to:("<<to_pose.position.x<<", "<<to_pose.position.y<<", "<<to_pose.position.z<<"), from:("<<from_pose.position.x<<", "<<from_pose.position.y<<", "<<from_pose.position.z<<")");

#if DRAW
	// draw source and target
	_draw_pts.points.resize(2);
	_draw_pts.points[0] = from_pose.position;
	_draw_pts.points[1] = to_pose.position;
	_draw_pts.header.stamp = ros::Time::now();
	_marker_pub.publish(_draw_pts);
#endif

	std::vector<geometry_msgs::Pose> poses;
	geometry_msgs::Pose msg;
	msg.orientation.w = 1;
	msg.orientation.x = 0;
	msg.orientation.y = 0;
	msg.orientation.z = 0;
	msg.position = from_pose.position;
	switch(_motion_mode){
		case MOTION::RAND:{		// DOBOT M1
			std::random_device rd;
			std::default_random_engine gen(rd());
			std::uniform_real_distribution<double> r_unif(0.0, 1.0), z_unif(ZMIN, ZMAX);
			double r, theta;
			size_t sample_num = 5;
			moveit::core::RobotState temp(_robot_model);
			poses.emplace_back(from_pose);
			// semi-uniform sampling within ring , sample_num random waypoints, note the offset 0.1
			for (size_t i=0; i<sample_num; ++i){
				ros::Time start = ros::Time::now();
				bool valid = false;
				do{
					r = std::sqrt(r_unif(gen) *(RMAX*RMAX - RMIN*RMIN) + RMIN*RMIN);
					theta = r_unif(gen) * M_PI;
					msg.position.z = z_unif(gen);
					msg.position.x = r * cos(theta);
					msg.position.y = r * sin(theta) + 0.1;
					if(temp.setFromIK(_jnt_model_group, msg, EEF_GROUP)){
						temp.update();
						valid = isPtValid(temp);
					}
				}while(!valid && ros::Time::now() - start < ros::Duration(0.1) );
				if (valid)
					poses.emplace_back(msg);
				else
					ROS_ERROR_STREAM("Timeout at "<<i+1<<"-th sampling!");
			}
			poses.emplace_back(to_pose);
			break;
		}
		case MOTION::LINE:{
#if DRAW
			visualization_msgs::Marker unmoved, moved;
			moved.header.frame_id = unmoved.header.frame_id = BASE_LINK;
			moved.ns = unmoved.ns = "planned_and_executed";
			moved.action = unmoved.action = visualization_msgs::Marker::ADD;
			moved.type = unmoved.type = visualization_msgs::Marker::POINTS;
			unmoved.id = 3;
			moved.id = 4;
			moved.scale.x = unmoved.scale.x = 0.02;
			moved.scale.y = unmoved.scale.y = 0.02;
			unmoved.color.r = 1.0;
			unmoved.color.b = 1.0;
			unmoved.color.a = 1.0;
			moved.color.r = 0.3;
			moved.color.g = 0.5;
			moved.color.b = 0.7;
			moved.color.a = 1.0; 
#endif
			double xmin, xmax, ymin, ymax;
			int xdir, ydir;
			if (to_pose.position.x < from_pose.position.x){
				xmin = to_pose.position.x;
				xmax = from_pose.position.x;
				xdir = -1;
			}
			else{
				xmin = from_pose.position.x;
				xmax = to_pose.position.x;
				xdir = 1;
			}
			if (to_pose.position.y < from_pose.position.y){
				ymin = to_pose.position.y;
				ymax = from_pose.position.y;
				ydir = -1;
			}
			else{
				ymin = from_pose.position.y;
				ymax = to_pose.position.y;
				ydir = 1;
			}
			bool checky = (xmax-xmin > ymax-ymin)? true : false, horiz = checky;

			// moving resolution: 0.05, and suppose the bigger side solution is larger than 0.05
			DIRECT direct;
			if (checky){
				direct = DIRECT::X;
			}
			else{
				direct = DIRECT::Y;
			}
			poses.emplace_back(from_pose);

			// debug: prevent infinite loop
//			ros::Time start = ros::Time::now();

			// TODO optimize?
			while(std::abs(msg.position.x-to_pose.position.x) > 1e-4 || std::abs(msg.position.y-to_pose.position.y) > 1e-4){
				if(horiz){
					if (checky){
						if (xdir > 0)
							msg.position.x = xmax;
						else
							msg.position.x = xmin;
						xdir = -xdir;
					}
					else
						msg.position.x += xdir*0.05;
				}
				else{
					if (!checky){
						if (ydir > 0)
							msg.position.y = ymax;
						else
							msg.position.y = ymin;
						ydir = -ydir;
					}
					else
						msg.position.y += ydir*0.05;
				}
				// check boundary
				if (!checky){
					if (msg.position.x < xmin)
						msg.position.x = xmin;
					else if (msg.position.x > xmax)
						msg.position.x = xmax;
				}
				else{
					if (msg.position.y < ymin)
						msg.position.y = ymin;
					else if (msg.position.y > ymax)
						msg.position.y = ymax;
				}
#if DRAW
				unmoved.points.emplace_back(msg.position);
#endif
				// move waypoint into workspace
				moveIntoWS(msg, direct);
#if DRAW
				moved.points.emplace_back(msg.position);				
#endif
				poses.emplace_back(msg);
				horiz = !horiz;

#if DRAW
				_marker_pub.publish(unmoved);
				_marker_pub.publish(moved);
#endif
				// debug: prevent infinite loop
/*
				if (ros::Time::now() - start > ros::Duration(2.0)){
					ROS_ERROR("Infinite Loop?");
					ROS_ERROR("Points: %lu", poses.size());
					break;
				}
*/
			}

			poses.emplace_back(to_pose);
			break;
		}
		case MOTION::CIRC:{
			// TODO
			break;
		}
	}

	return motionTo(poses);
}

void Manipulator::moveIntoWS(geometry_msgs::Pose& pose, DIRECT direct){	// DOBOT M1
	if (pose.position.z > ZMAX)
		pose.position.z = ZMAX;
	else if (pose.position.z < ZMIN)
		pose.position.z = ZMIN;

	// note the offset (0.1) from BASE_LINK
	double dist = std::sqrt(pose.position.x * pose.position.x + (pose.position.y - 0.1) * (pose.position.y - 0.1));
	if (dist < RMIN)
		direct = DIRECT::R;
	if (direct == DIRECT::X){
		if (dist > RMAX){
			if (std::abs(pose.position.y - 0.1) > RMAX)
				direct = DIRECT::R;
			else{
				double newx = std::sqrt(RMAX*RMAX - (pose.position.y - 0.1) * (pose.position.y - 0.1));
				pose.position.x = (pose.position.x > 0)? newx : -newx;
			}
		}
	}
	
	if (direct == DIRECT::Y){
		if (dist > RMAX){
			if (std::abs(pose.position.x) > RMAX)
				direct = DIRECT::R;
			else{
				double newy = std::sqrt(RMAX*RMAX - pose.position.x * pose.position.x);
				pose.position.y = (pose.position.y -0.1 > 0)? newy + 0.1 : -newy + 0.1;
			}
		}
	}

	if (direct == DIRECT::R){
		if (dist > RMAX){
			pose.position.x *= RMAX / dist;
			pose.position.y = (pose.position.y - 0.1 ) * RMAX / dist + 0.1;
		}
		else if (dist < RMIN){
			pose.position.x *= RMIN / dist;
			pose.position.y = (pose.position.y - 0.1 ) * RMIN / dist + 0.1;
		}
	}
}

bool Manipulator::calcMotionLines(const std::vector<geometry_msgs::Pose>& poses){	
	std::vector<moveit::core::RobotStatePtr> temp_traj;
	Eigen::Isometry3d eig_pose;
	double fraction;

	// move to first achievable point
	size_t i=0;
	for (; i<poses.size(); ++i){
		if(!moveTo(poses[i])){
			ROS_ERROR_STREAM("Fail to moveTo poses["<<i<<"] on Line!");
			if (_care){
				resetPlan();
				return false;
			}
		}
		else{
			++i;	// line constrcution from next index in the following
			break;
		}
	}

	for (; i<poses.size(); ++i){
		tf2::fromMsg(poses[i], eig_pose);
		fraction = moveit::core::CartesianInterpolator::computeCartesianPath(_plan_end_state, _jnt_model_group, temp_traj, _plan_end_state->getLinkModel(EEF_GROUP), eig_pose, true, moveit::core::MaxEEFStep(0.01), moveit::core::JumpThreshold(0));
		
		// accept the cartesian plan and plan the rest
		if (!temp_traj.empty()){
			robot_trajectory::RobotTrajectoryPtr rt = std::make_shared<robot_trajectory::RobotTrajectory>(_robot_model, PLAN_GROUP);
			for (const moveit::core::RobotStatePtr& traj_state : temp_traj)
				rt->addSuffixWayPoint(traj_state, 0.0);
			generateTraj(rt);
		}
		if (fraction != 1){
			ROS_WARN_STREAM("Cartesian plan: "<<100*fraction<<"%. Try moveTo poses["<<i<<"]!");
			// Cartesian plan lefts few steps, try moveTo
			if(!moveTo(poses[i])){
				ROS_ERROR_STREAM("MoveTo fails for rest steps to poses["<<i<<"]");
				if (_care){
					resetPlan();
					return false;
				}
			}
		}
	}
	return true;
}

bool Manipulator::calcMotionCircs(const std::vector<geometry_msgs::Pose>& poses){
	// TODO
	ROS_ERROR_STREAM("MOTION::CIRC not implement yet!");
	return false;
}

}	// namespace move_plan
