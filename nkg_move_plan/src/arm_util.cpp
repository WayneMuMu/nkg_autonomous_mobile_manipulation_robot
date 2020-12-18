/*********************************************************************
 *	Software License Agreement (BSD License)
 *
 *	All rights reserved.
 *
 *	Author: 	Yu-Wen Chen
 *	Date: 		2020/09
 *	Version: 	1.0.0
 *********************************************************************/
#include "nkg_move_plan/arm_util.h"

namespace move_plan
{
Arm::Arm(){
	// tf_buffer
	_tf_buffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));
	_tfl = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

	// Planning Scene Monitor
	_psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description",_tf_buffer);
	if (!_psm->getPlanningScene())
		ROS_ERROR("PSM has no planning_scene!");
    printf(MOVEIT_CONSOLE_COLOR_CYAN "Starting planning scene monitors...\n" MOVEIT_CONSOLE_COLOR_RESET);
	startPSM();
	//_psm->startPublishingPlanningScene( planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE);
    printf(MOVEIT_CONSOLE_COLOR_CYAN "Planning scene monitors started.\n" MOVEIT_CONSOLE_COLOR_RESET);

	// MoveGroupContext
	_context = std::make_shared<move_group::MoveGroupContext>(_psm, true, false);

	// Recorded RobotState
	_rec_state_ptr = _psm->getStateMonitor()->getCurrentState();

	// Configure params
	configParam();

	// Configure MoveGroupCapability
	configMoveGroupCap();

	// Manipulator
	manipulator.reset(new Manipulator(_context.get(), _rec_state_ptr.get()));

	// Gripper
	gripper.reset(new Gripper(_context.get(), _rec_state_ptr.get()));

	// move_group
	move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLAN_GROUP));
}

void Arm::start(){
	manipulator->start();
	gripper->start();
}

void Arm::startPSM(){
	_psm->startSceneMonitor();
	_psm->startWorldGeometryMonitor();
	_psm->startStateMonitor();
}

void Arm::stopPSM(){
	_psm->stopSceneMonitor();
	_psm->stopWorldGeometryMonitor();
	_psm->stopStateMonitor();
}

void Arm::configParam(){

}

void Arm::configMoveGroupCap(){
	ros::NodeHandle _n("~");
	try{
		_capability_plugin_loader = std::make_shared< pluginlib::ClassLoader<move_group::MoveGroupCapability> >("moveit_ros_move_group", "move_group::MoveGroupCapability");
	}
	catch (pluginlib::PluginlibException& ex){
		ROS_FATAL_STREAM("Exception while creating plugin loader for move_group capabilities: " << ex.what());
		return;
	}

	std::set<std::string> capabilities;

	// add default capabilities
	for (const char* capability : DEFAULT_CAPABILITIES)
		capabilities.insert(capability);

	// add capabilities listed in ROS parameter
	std::string capability_plugins;
	if (_n.getParam("capabilities", capability_plugins)){
		boost::char_separator<char> sep(" ");
		boost::tokenizer<boost::char_separator<char> > tok(capability_plugins, sep);
		capabilities.insert(tok.begin(), tok.end());
    }

	// drop capabilities that have been explicitly disabled
	if (_n.getParam("disable_capabilities", capability_plugins)){
		boost::char_separator<char> sep(" ");
		boost::tokenizer<boost::char_separator<char> > tok(capability_plugins, sep);
		for (boost::tokenizer<boost::char_separator<char> >::iterator cap_name = tok.begin(); cap_name != tok.end();
           ++cap_name)
			capabilities.erase(*cap_name);
	}

	for (const std::string& capability : capabilities){
		try{
			ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_CYAN "Loading '"<<capability<<"'...\n" MOVEIT_CONSOLE_COLOR_RESET);
			move_group::MoveGroupCapabilityPtr cap = _capability_plugin_loader->createUniqueInstance(capability);
			cap->setContext(_context);
			cap->initialize();
			_capabilities.emplace_back(cap);
		}
		catch (pluginlib::PluginlibException& ex){
			ROS_ERROR_STREAM("Exception while loading move_group capability '" << capability << "': " << ex.what());
		}
    }

	std::stringstream ss;
	ss << std::endl;
	ss << std::endl;
	ss << "********************************************************" << std::endl;
	ss << "* Using MoveGroup: " << std::endl;
	for (const move_group::MoveGroupCapabilityPtr& cap : _capabilities)
		ss << "*     - " << cap->getName() << std::endl;
	ss << "********************************************************" << std::endl;
	ROS_INFO_STREAM(ss.str());
	// Status()
    if (_context){
		if (_context->status()){
			if (_capabilities.empty())
				printf(MOVEIT_CONSOLE_COLOR_BLUE "\nmove_group is running but no capabilities are "
                                           "loaded.\n\n" MOVEIT_CONSOLE_COLOR_RESET);
			else
				printf(MOVEIT_CONSOLE_COLOR_GREEN "\nYou can start planning now!\n\n" MOVEIT_CONSOLE_COLOR_RESET);
			fflush(stdout);
		}
	}
	else
		ROS_ERROR_STREAM("No MoveGroup context created. Nothing will work.");
}

bool Arm::execCmdQueue(){
	bool keep = true;
	while(!_cmds.empty()){
		std::pair<CMD, dataType> cmd = _cmds.front();
		switch(cmd.first){
			case CMD::RESETPLAN :
				manipulator->resetPlan();
				break;
			case CMD::MOVETO :
				if (cmd.second.type() == typeid(geometry_msgs::Pose))
					keep = manipulator->moveTo(boost::get<geometry_msgs::Pose>(cmd.second));
				else if (cmd.second.type() == typeid(std::vector<double>))
					keep = manipulator->moveTo(boost::get<std::vector<double> >(cmd.second));
				else if (cmd.second.type() == typeid(std::string))
					keep = manipulator->moveTo(boost::get<std::string>(cmd.second));
				else{
					ROS_ERROR_STREAM("Wrong dataType for moveTo!");
					keep = false;
				}
				break;
			case CMD::MOTIONTO :
				if (cmd.second.type() == typeid(std::vector<geometry_msgs::Pose>))
					keep = manipulator->motionTo(boost::get<std::vector<geometry_msgs::Pose> >(cmd.second));
				else if (cmd.second.type() == typeid(std::vector<std::vector<double> >))
					keep = manipulator->motionTo(boost::get<std::vector<std::vector<double> > >(cmd.second));
				else{
					ROS_ERROR_STREAM("Wrong dataType for motionTo!");
					keep = false;
				}
				break;
			case CMD::MOTIONTOFROM :
				if (cmd.second.type() == typeid(geometry_msgs::Pose))
					keep = manipulator->motionToFrom(boost::get<geometry_msgs::Pose>(cmd.second));
				else if (cmd.second.type() == typeid(std::vector<double>))
					keep = manipulator->motionToFrom(boost::get<std::vector<double> >(cmd.second));
				else if (cmd.second.type() == typeid(std::vector<geometry_msgs::Pose>)){
					std::vector<geometry_msgs::Pose> poses = boost::get<std::vector<geometry_msgs::Pose> >(cmd.second);
					keep = manipulator->motionToFrom(poses[1], poses[0]);
				}
				else if (cmd.second.type() == typeid(std::vector<std::vector<double> >)){
					std::vector<std::vector<double> > jnts = boost::get<std::vector<std::vector<double> > >(cmd.second);
					keep = manipulator->motionToFrom(jnts[1], jnts[0]);
				}
				else{
					ROS_ERROR_STREAM("Wrong dataType for motionToFrom!");
					keep = false;
				}
				break;
			case CMD::SETMOTIONMODE :
				if (cmd.second.type() == typeid(MOTION))
					manipulator->setMotionMode(boost::get<MOTION>(cmd.second));
				else{
					ROS_ERROR_STREAM("Wrong dataType for setMotionMode!");
					keep = false;
				}
				break;
			case CMD::EXECTO :
				if (cmd.second.type() == typeid(bool))
					keep = manipulator->execTo(boost::get<bool>(cmd.second));
				else{
					ROS_ERROR_STREAM("Wrong dataType for execTo!");
					keep = false;
				}
				break;
			case CMD::RESETGRIPPER :
				gripper->resetGripper();
				break;
			case CMD::GRASP :
				if (cmd.second.type() == typeid(std::pair<std::string, std::vector<moveit_msgs::Grasp> >)){
					std::pair<std::string, std::vector<moveit_msgs::Grasp> > data = boost::get<std::pair<std::string, std::vector<moveit_msgs::Grasp> > >(cmd.second);
					keep = gripper->planAndExecGrasp(data.first, data.second);
				}
				else{
					ROS_ERROR_STREAM("Wrong dataType for planAndExecGrasp!");
					keep = false;
				}
				break;
			case CMD::PLACE :
				if (cmd.second.type() == typeid(std::pair<std::string, std::vector<moveit_msgs::PlaceLocation> >)){
					std::pair<std::string, std::vector<moveit_msgs::PlaceLocation> > data = boost::get<std::pair<std::string, std::vector<moveit_msgs::PlaceLocation> > >(cmd.second);
					keep = gripper->planAndExecPlace(data.first, data.second);
				}
				else{
					ROS_ERROR_STREAM("Wrong dataType for planAndExecPlace!");
					keep = false;
				}
				break;
			case CMD::CLOSE :
				if (cmd.second.type() == typeid(double))
					keep = gripper->gripperTo(boost::get<double>(cmd.second));
				else{
					ROS_ERROR_STREAM("Wrong dataType for gripperTo!");
					keep = false;
				}				
				break;
			default:
				ROS_ERROR_STREAM("Wrong CMD!");
				break;
		}
		if (!keep)	
				break;
		_cmds.pop_front();
	}
	if (!keep)
		ROS_ERROR_STREAM("Remain "<<_cmds.size()<<" cmds in CmdQueue!");
	else
		ROS_INFO_STREAM("CmdQueue all executed");
	return keep;
}

}	// namespace move_plan
