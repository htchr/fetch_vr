#ifndef HEAD_HPP
#define HEAD_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>

class Head {
public:
	Head();
	void panTilt(double pan, double tilt);

private:
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> pan_client_;
};

#endif // HEAD_HPP

