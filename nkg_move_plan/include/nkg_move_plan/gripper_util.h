#ifndef NKG_GRIPPER_UTIL_H
#define NKG_GRIPPER_UTIL_H

#include "nkg_move_plan/manipulator_util.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/move_group/move_group_context.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_model/robot_model.h"

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
	bool planAndExecGrasp(const moveit_msgs::PickupGoal&);
	bool planAndExecPlace(const moveit_msgs::PlaceGoal&);
	bool gripperTo(const double, const bool queued=false);
	bool execTo();
	void resetGoal();

private:
	void configParam();
	void beforePlanGrasp();
	void beforePlanPlace();
	bool planGrasp(const moveit_msgs::PickupGoal&, plan_execution::ExecutableMotionPlan&);
	bool planPlace(const moveit_msgs::PlaceGoal&, plan_execution::ExecutableMotionPlan&);
	bool execTo(control_msgs::GripperCommandGoal& goal);

	planning_scene_monitor::PlanningSceneMonitorPtr _psm;
	move_group::MoveGroupContext *_context;
	pick_place::PickPlacePtr _pick_place;
	moveit::core::RobotState *_grasp_end_state;
	double _regrasp_delay, _replace_delay;
	int _regrasp_attempts, _replace_attempts;
	std::unique_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction> > _ac;
	control_msgs::GripperCommandGoal _goal;
};

}
#endif
