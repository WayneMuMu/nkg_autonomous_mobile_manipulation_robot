#ifndef NKG_ARM_UTIL_H
#define NKG_ARM_UTIL_H

#include "nkg_move_plan/manipulator_util.h"
#include "nkg_move_plan/gripper_util.h"
#include <pluginlib/class_loader.h>
#include "moveit/move_group/capability_names.h"
#include "moveit/move_group/move_group_capability.h"
#include <boost/tokenizer.hpp>
#include "moveit/macros/console_colors.h"
#include "moveit/move_group/node_name.h"
#include <tf2_ros/transform_listener.h>
#include <boost/variant.hpp>

namespace move_plan
{
enum CMD: uint8_t{
	RESETPLAN = 0,
	MOVETO = 1,
	MOTIONTO = 2,
	MOTIONTOFROM = 3,
	SETMOTIONMODE = 4,
	EXECTO = 5
};

typedef boost::variant<bool, geometry_msgs::Pose, std::vector<double>, std::string, uint8_t, std::vector<geometry_msgs::Pose>, std::vector<std::vector<double> > > dataType;

class Arm{
public:
	Arm();
	Arm(const Arm&) = delete;
	Arm& operator=(const Arm&) = delete;
	void start();
	void startPSM();
	void stopPSM();
	void add2CmdQueue(const std::pair<uint8_t, dataType>& cmd){ _cmds.push_back(cmd); };
	bool execCmdQueue();
	void cleanCmdQueue(){ _cmds.clear(); };

	std::unique_ptr<Manipulator> manipulator;
	std::unique_ptr<Gripper> gripper;

private:
	void configParam();
	void configMoveGroupCap();

	std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> _tfl;
	std::shared_ptr<pluginlib::ClassLoader<move_group::MoveGroupCapability> > _capability_plugin_loader;
	std::vector<move_group::MoveGroupCapabilityPtr> _capabilities;
	planning_scene_monitor::PlanningSceneMonitorPtr _psm;
	move_group::MoveGroupContextPtr _context;
	std::vector<std::pair< uint8_t, dataType> > _cmds;
	moveit::core::RobotStatePtr _rec_state_ptr;
};

}
#endif
