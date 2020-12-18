#ifndef NKG_ARM_UTIL_H
#define NKG_ARM_UTIL_H

#include "nkg_move_plan/manipulator_util.h"
#include "nkg_move_plan/gripper_util.h"
#include "moveit/move_group/move_group_capability.h"
#include "moveit/move_group/capability_names.h"
#include "moveit/macros/console_colors.h"
#include <tf2_ros/transform_listener.h>
#include <boost/tokenizer.hpp>
#include <boost/variant.hpp>
#include "moveit/move_group_interface/move_group_interface.h"

namespace move_plan
{
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

enum class CMD{
	RESETPLAN,
	MOVETO,
	MOTIONTO,
	MOTIONTOFROM,
	SETMOTIONMODE,
	EXECTO,
	RESETGRIPPER,
	GRASP,
	PLACE,
	CLOSE
};

class Arm{
typedef boost::variant<bool, geometry_msgs::Pose, std::vector<double>, std::string, MOTION, std::vector<geometry_msgs::Pose>, std::vector<std::vector<double> >, std::pair<std::string, std::vector<moveit_msgs::Grasp> >, std::pair<std::string, std::vector<moveit_msgs::PlaceLocation> >, double > dataType;
public:
	Arm();
	Arm(const Arm&) = delete;
	Arm& operator=(const Arm&) = delete;
	void start();
	void startPSM();
	void stopPSM();
	void add2CmdQueue(const std::pair<CMD, dataType>& cmd){ _cmds.push_back(cmd); };
	bool execCmdQueue();
	void cleanCmdQueue(){ _cmds.clear(); };

	std::unique_ptr<Manipulator> manipulator;
	std::unique_ptr<Gripper> gripper;
	std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

private:
	void configParam();
	void configMoveGroupCap();

	std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> _tfl;
	std::shared_ptr<pluginlib::ClassLoader<move_group::MoveGroupCapability> > _capability_plugin_loader;
	std::vector<move_group::MoveGroupCapabilityPtr> _capabilities;
	planning_scene_monitor::PlanningSceneMonitorPtr _psm;
	move_group::MoveGroupContextPtr _context;
	std::deque<std::pair<CMD, dataType> > _cmds;
	moveit::core::RobotStatePtr _rec_state_ptr;
};

}
#endif
