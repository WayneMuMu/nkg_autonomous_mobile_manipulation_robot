#include "nkg_ca_controller/nkg_ca_controller.h"
#include <pluginlib/class_list_macros.h>

namespace nkg_ca_controller {

	JointTrajectoryController::JointTrajectoryController(){}

	JointTrajectoryController::~JointTrajectoryController(){}

void JointTrajectoryController::customizedInit(){
	ROS_WARN("Catching nkg_ca_controller PIDs");
	hw_iface_adapter_.getPIDMembers(pids, velocity_ff);
	rep_vel.resize(joints_.size());
	rep_vel_sub = nh.subscribe("repulsive_velocity", 5, &JointTrajectoryController::repVelCB, this);
}

void JointTrajectoryController::update(const ros::Time& time, const ros::Duration& period){
	// Get currently followed trajectory
	TrajectoryPtr curr_traj_ptr;
	curr_trajectory_box_.get(curr_traj_ptr);
	Trajectory& curr_traj = *curr_traj_ptr;

	// Update time data
	TimeData time_data;
	time_data.time   = time;                                     // Cache current time
	time_data.period = period;                                   // Cache current control period
	time_data.uptime = time_data_.readFromRT()->uptime + period; // Update controller uptime
	time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

	// NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
	// trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
	// The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
	// control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
	// If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time we
	// fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts in the
	// next control cycle, leaving the current cycle without a valid trajectory.

	// Update current state and state error
	for (unsigned int i = 0; i < joints_.size(); ++i){
		current_state_.position[i] = joints_[i].getPosition();
		current_state_.velocity[i] = joints_[i].getVelocity();
		// There's no acceleration data available in a joint handle

		typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], time_data.uptime.toSec(), desired_joint_state_);

		//TODO for TEST final point only!!!!
//		typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], (--curr_traj[i].end())->endTime(), desired_joint_state_);		

		if (curr_traj[i].end() == segment_it){
			// Non-realtime safe, but should never happen under normal operation
			ROS_ERROR_NAMED(name_, "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
			return;
		}
		desired_state_.position[i] = desired_joint_state_.position[0];
		desired_state_.velocity[i] = desired_joint_state_.velocity[0];
		desired_state_.acceleration[i] = desired_joint_state_.acceleration[0]; ;

		state_joint_error_.position[0]= angles::shortest_angular_distance(current_state_.position[i], desired_joint_state_.position[0]);
		state_joint_error_.velocity[0]= desired_joint_state_.velocity[0] - current_state_.velocity[i];
		state_joint_error_.acceleration[0] = 0.0;

		state_error_.position[i] = angles::shortest_angular_distance(current_state_.position[i], desired_joint_state_.position[0]);
		state_error_.velocity[i] = desired_joint_state_.velocity[0] - current_state_.velocity[i];
		state_error_.acceleration[i] = 0.0;

		// TODO Check tolerances
		const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
		if (rt_segment_goal && rt_segment_goal == rt_active_goal_){
		// Check tolerances
			if (time_data.uptime.toSec() < segment_it->endTime()){
				// Currently executing a segment: check path tolerances
				const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& joint_tolerances = segment_it->getTolerances();
				if (!joint_trajectory_controller::checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance)){
					rt_segment_goal->preallocated_result_->error_code =
					control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
					rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
					rt_active_goal_.reset();
					successful_joint_traj_.reset();
				}
			}
			else if (segment_it == --curr_traj[i].end()){
				// Controller uptime
				const ros::Time uptime = time_data_.readFromRT()->uptime;

				// Checks that we have ended inside the goal tolerances
				const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& tolerances = segment_it->getTolerances();
				const bool inside_goal_tolerances = joint_trajectory_controller::checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance);

				if (inside_goal_tolerances){
					successful_joint_traj_[i] = 1;
				}
				else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance){
					// Still have some time left to meet the goal state tolerances
				}
        		else{
					rt_segment_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
					rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
					rt_active_goal_.reset();
					successful_joint_traj_.reset();
				}
			}
		}
	}

	// If there is an active goal and all segments finished successfully then set goal as succeeded
	RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
	if (current_active_goal && successful_joint_traj_.count() == joints_.size()){
		current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
		current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
		current_active_goal.reset(); // do not publish feedback
		rt_active_goal_.reset();
		successful_joint_traj_.reset();
	}

	// Hardware interface adapter: Generate and send commands
//	hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period, desired_state_, state_error_);
	senseAndCommand(time_data.uptime, time_data.period, desired_state_, state_error_);

	// Set action feedback
	if (current_active_goal){
		current_active_goal->preallocated_feedback_->header.stamp = time_data_.readFromRT()->time;
		current_active_goal->preallocated_feedback_->desired.positions     = desired_state_.position;
		current_active_goal->preallocated_feedback_->desired.velocities    = desired_state_.velocity;
		current_active_goal->preallocated_feedback_->desired.accelerations = desired_state_.acceleration;
		current_active_goal->preallocated_feedback_->actual.positions      = current_state_.position;
		current_active_goal->preallocated_feedback_->actual.velocities     = current_state_.velocity;
		current_active_goal->preallocated_feedback_->error.positions       = state_error_.position;
		current_active_goal->preallocated_feedback_->error.velocities      = state_error_.velocity;
		current_active_goal->setFeedback( current_active_goal->preallocated_feedback_ );
	}

	// Publish state
	publishState(time_data.uptime);
}

void JointTrajectoryController::repVelCB(const std_msgs::Float64MultiArray& rep_vel_msg){
	rep_vel = rep_vel_msg.data;
}

//TODO nkg: collision avoidance and command sending
void JointTrajectoryController::senseAndCommand(const ros::Time& /*time*/, const ros::Duration& period, const typename Segment::State& desired_state, const typename Segment::State& state_error){
	const unsigned int n_joints = joints_.size();
	// pre-conditions
	assert(n_joints == state_error.position.size());
	assert(n_joints == state_error.velocity.size());

	// Update PIDs
	for (unsigned int i = 0; i < n_joints; ++i){
		const double attr_vel = (desired_state.velocity[i] * velocity_ff[i]) + pids[i]->computeCommand(state_error.position[i], state_error.velocity[i], period);
		joints_[i].setCommand(attr_vel + rep_vel[i]);
	}
}

} // namespace

PLUGINLIB_EXPORT_CLASS( 
    nkg_ca_controller::JointTrajectoryController,
    controller_interface::ControllerBase)
