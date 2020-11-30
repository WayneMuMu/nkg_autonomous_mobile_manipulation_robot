#ifndef NKG_CA_CONTROLLER_H
#define NKG_CA_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <control_toolbox/pid_gains_setter.h>

#include <hardware_interface/joint_command_interface.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <ros/time.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_msgs/Float64MultiArray.h>

namespace nkg_ca_controller
{

  class JointTrajectoryController: public joint_trajectory_controller::JointTrajectoryController<
	trajectory_interface::QuinticSplineSegment<double>, hardware_interface::VelocityJointInterface>
  {

  public:
    JointTrajectoryController();
    ~JointTrajectoryController();
	void update(const ros::Time& time, const ros::Duration& period) override;

  protected:
	void customizedInit() override;	// override empty function added in joint_trajectory_controller
	void senseAndCommand(const ros::Time&, const ros::Duration&, 
							const typename Segment::State&, const typename Segment::State&);
	void repVelCB(const std_msgs::Float64MultiArray&);	//CallBack of repuslive velocity topic
	
	typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
	std::vector<PidPtr> pids;
	std::vector<double> velocity_ff;
	std::vector<double> rep_vel;

	ros::NodeHandle nh;
	ros::Subscriber rep_vel_sub;
  };

} // namespace

#endif
