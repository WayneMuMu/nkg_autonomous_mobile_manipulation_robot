#ifndef NKG_MANIPULATOR_UTIL_H
#define NKG_MANIPULATOR_UTIL_H

#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit/planning_pipeline/planning_pipeline.h"
#include "moveit/plan_execution/plan_execution.h"
#include "moveit/move_group/move_group_context.h"
#include "moveit/move_group/capability_names.h"

#include "nkg_demo_msgs/XYZ.h"

#define DRAW 1

namespace move_plan{
enum MOTION: uint8_t{
	RAND = 0,
	LINE = 1,
	CIRC = 2 
};
enum DIRECT: uint8_t{
	X = 0,
	Y = 1,
	R = 2 
};

static const char* DEFAULT_CAPABILITIES[] = {
   "move_group/MoveGroupCartesianPathService",
   "move_group/MoveGroupKinematicsService",
   "move_group/MoveGroupExecuteTrajectoryAction",
   "move_group/MoveGroupMoveAction",
   "move_group/MoveGroupPickPlaceAction",
   "move_group/MoveGroupPlanService",
   "move_group/MoveGroupQueryPlannersService",
   "move_group/MoveGroupStateValidationService",
   "move_group/MoveGroupGetPlanningSceneService",
   "move_group/ApplyPlanningSceneService",
   "move_group/ClearOctomapService",
};

class Manipulator{
public:
	Manipulator(move_group::MoveGroupContext*, moveit::core::RobotState*);
	Manipulator(const Manipulator&) = delete;
	Manipulator& operator=(const Manipulator&) = delete;

	void start();
	void resetPlan();
	bool moveTo(const geometry_msgs::Pose&);						// plan directly to pose
	bool moveTo(const std::vector<double>&);						// plan directly to joints value
	bool moveTo(const std::string&);								// plan directly to srdf name
	bool moveTo(planning_interface::MotionPlanRequest&);			// advanced plan
	bool execTo(const bool motion=false);							// exec after moveTo/motionTo(From
	void setMotionMode(const uint8_t mode){ if (mode<3) _motion_mode = mode;
											else ROS_ERROR("Wrong Mode!"); };	// for motionTo(From
	bool motionToFrom(const geometry_msgs::Pose&);								// plan motion
	bool motionToFrom(const std::vector<double>&);								// plan motion
	bool motionToFrom(const geometry_msgs::Pose&, const geometry_msgs::Pose&);	// plan motion
	bool motionToFrom(const std::vector<double>&, const std::vector<double>&);	// plan motion
	bool motionTo(const std::vector<geometry_msgs::Pose>&);						// plan motion
	bool motionTo(const std::vector<std::vector<double> >&);					// plan motion

	bool getXYZ(nkg_demo_msgs::XYZ::Request&, nkg_demo_msgs::XYZ::Response&);

private:
	void configParam();
	bool hasSrdfName(const std::string&) const;
	bool moveTo(const moveit::core::RobotState&);					// plan directly to robot state
	bool moveTo(const moveit_msgs::Constraints&);					// plan directly to goal msg
	bool planTo(planning_interface::MotionPlanRequest&);
	bool connectTo(const moveit::core::RobotState&);				// TODO customized connection!!
	void generateTraj(robot_trajectory::RobotTrajectoryPtr&);		// append traj & update end state
	void execThread(plan_execution::ExecutableMotionPlan&);			// execution and monitor thread
	bool replanMove(const std::pair<int, int>&);					// env changes, replan moveTo
	bool replanMotion(const std::pair<int, int>&);					// env changes, replan motionTo
	void calcIPTP();												// time smoothing
	bool isPtValid(const moveit::core::RobotState&);				// check robot state valid or not
	bool calcMotionLines(const std::vector<geometry_msgs::Pose>&);	// calc Cartesian path
	bool calcMotionCircs(const std::vector<geometry_msgs::Pose>&);	// calc Circular  path
	void moveIntoWS(geometry_msgs::Pose&, uint8_t);					// move target into workspace
#if DRAW
	void drawPath();		// draw rviz markers
#endif

	std::vector<std::string> _srdf_names;							// defined group_state in srdf
	uint8_t _motion_mode, _replan_num;								// # has replaned for recent plan
	move_group::MoveGroupContext *_context;
	const moveit::core::JointModelGroup *_jnt_model_group;
	moveit::core::RobotModelConstPtr _robot_model;
	planning_scene_monitor::PlanningSceneMonitorPtr _psm;			// CORE!!!
	std::vector<robot_trajectory::RobotTrajectoryPtr> _plan_traj;	// record planned trajectory
	moveit::core::RobotState *_plan_end_state;						// record planned end state
	std::vector<planning_interface::MotionPlanRequest> _latest_goals;// for replanning moveTo
	std::vector<robot_trajectory::RobotTrajectoryPtr> _latest_motion;// for replanning motionTo
	double _pos_tol, _ori_tol, _plan_time, _replan_delay;
	int _replan_attempts, _plan_attempts, _replan_jump;
	bool _executed, _succeed, _replan, _care;
	ros::ServiceServer _xyz_srv;
#if DRAW
	// draw markers
	ros::Publisher _marker_pub;
	visualization_msgs::Marker _draw_plan, _draw_exec, _draw_pts;
	bool _first_draw, _first_targets;
#endif
};

class MovingBase{
public:
	MovingBase();
	~MovingBase();
	void start();

private:
	void configParam();


};


}
#endif
